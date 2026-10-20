"""PICO operations against the shared arm-target and gripper service contract."""
import hashlib
import json
import math
import time
from pathlib import Path
from bindu_interfaces.msg import GripperState
from bindu_interfaces.srv import GripperControl, Lease
from bindu_hardware.snowbot_gripper import SIDES, GROUPS, JOINTS, validate_config
from .common import seconds, stamp


class PositionOperations:
    def __init__(self, node, path):
        self.node = node
        path = Path(path)
        self.config_hash = hashlib.sha256(path.read_bytes()).hexdigest()
        self.config = validate_config(json.loads(path.read_text()), node.profile)
        if self.config.get('simulation_only', False) is not node.simulation:
            raise ValueError('PICO_GRIPPER_CONFIG_MODE_MISMATCH')
        self.states = {}
        self.lease = None
        self.sequence = 0
        self.leases = node.create_client(Lease, 'gripper/lease')
        self.control = node.create_client(GripperControl, 'gripper/control')
        node.create_subscription(GripperState, 'gripper/state', self.receive, 10)

    def receive(self, msg):
        n = self.node
        if (msg.side in SIDES and n.matches_mode(msg) and msg.profile_hash == n.profile.digest and
                msg.config_hash == self.config_hash):
            self.states[msg.side] = msg

    def reading(self, side):
        n = self.node
        msg = self.states.get(side)
        if (msg is None or n.state is None or msg.instance_id != n.state.instance_id or
                not msg.feedback_valid or not msg.calibrated or
                not 0 <= n.now()-seconds(msg.stamp) <= .2 or
                not 0 <= n.now()-seconds(msg.feedback_stamp) <= .2 or
                not all(math.isfinite(v) for v in (msg.measured_position, msg.measured_velocity))):
            raise RuntimeError('PICO_GRIPPER_FEEDBACK_OR_CALIBRATION_REQUIRED:'+side)
        if msg.state == 'FAULT':
            raise RuntimeError(msg.code)
        return msg

    def modes(self, *, require_stationary=True):
        result = []
        for side in SIDES:
            try:
                msg = self.reading(side)
                # During loaded following, contact can change velocity while
                # the opening stays inside its calibrated band. Switching and
                # stop completion still require stationary measured feedback.
                if not require_stationary and msg.state in ('MOVING','STOPPING'):
                    result.append('unknown');continue
                result.append(next((mode for mode in ('closed', 'virtual', 'open') if
                    mode in self.config['grippers'][side] and
                    abs(msg.measured_position-self.config['grippers'][side][mode]) <=
                    self.config['position_tolerance'] and
                    (not require_stationary or abs(msg.measured_velocity) <= self.config['velocity_tolerance'])), 'unknown'))
            except RuntimeError:
                result.append('unknown')
        return tuple(result)

    @property
    def axial_available(self):
        return self.config['calibrated'] and 'virtual' in self.config['grippers']['right']

    def grasp_target(self, virtual):
        if not self.axial_available:
            raise RuntimeError('PICO_AXIAL_GRASP_UNAVAILABLE')
        return {JOINTS['right']:self.config['grippers']['right']['virtual' if virtual else 'closed']}

    def stopped(self):
        # A gripper fault is never hidden by the arm executor being idle.
        return all(s.state not in ('MOVING','STOPPING','FAULT') for s in self.states.values())

    async def request(self, operation, side='', **kwargs):
        n = self.node
        self.sequence += 1
        response = await n.wait(self.control.call_async(GripperControl.Request(
            operation=operation, side=side, lease_id=self.lease.lease_id,
            epoch=self.lease.epoch, sequence=self.sequence, stamp=stamp(n.now()),
            valid_for=.25, **kwargs)))
        if not response.accepted:
            raise RuntimeError(response.code)

    async def release(self):
        if self.lease is None:
            return
        n, lease = self.node, self.lease
        try:
            started = n.now()
            response = await n.wait(self.leases.call_async(Lease.Request(
                operation='release', lease_id=lease.lease_id, epoch=lease.epoch)))
            if not response.ok:
                raise RuntimeError(response.code)
            end = time.monotonic()+self.config['stop_timeout']+.5
            while True:
                states = [self.reading(s) for s in SIDES]
                if all(seconds(s.feedback_stamp) > started and s.epoch > lease.epoch and
                       not s.lease_active and s.state not in ('MOVING','STOPPING') for s in states):
                    break
                if time.monotonic() > end:
                    raise RuntimeError('PICO_GRIPPER_STOP_UNCONFIRMED')
                await n.pause()
        finally:
            self.lease = None

    async def move_grippers(self, intent, *, direct=False):
        n = self.node
        targets={s:m for s,m in zip(SIDES,intent.value) if not direct or m is not None}
        if not targets or any(m not in ('open','closed') for m in targets.values()):
            raise RuntimeError('PICO_GRIPPER_TARGET_INVALID')
        for side in SIDES:
            self.reading(side)
        lease = await n.wait(self.leases.call_async(Lease.Request(operation='acquire',
            owner=n.instance_id, resources=[GROUPS[s] for s in targets])))
        if not lease.ok:
            raise RuntimeError(lease.code)
        self.lease, self.sequence = lease, 0
        try:
            n.current(intent)
            await self.request('authorize_direct' if direct else 'authorize_loading', load_state=n.control.load,
                               external_support=False if direct else n.control.supported)
            expected = {}
            for side, mode in targets.items():
                n.current(intent)
                if mode not in ('open', 'closed'):
                    raise RuntimeError('PICO_AXIAL_GRASP_UNAVAILABLE')
                await self.request('open' if mode == 'open' else 'close', side)
                expected[side] = self.config['grippers'][side][mode]
            issued = n.now()
            end, renew_at = time.monotonic()+self.config['motion_timeout']+1., 0.
            while time.monotonic() < end:
                n.current(intent)
                states = [self.reading(s) for s in targets]
                if any(s.state in ('FAILED','FAULT') for s in states):
                    raise RuntimeError('PICO_GRIPPER_OPERATION_FAILED')
                if all(s.position_reached and s.epoch == lease.epoch and s.control_sequence >= self.sequence and
                       seconds(s.feedback_stamp) > issued and
                       abs(s.measured_position-expected[s.side]) <= self.config['position_tolerance'] for s in states):
                    return
                if time.monotonic() >= renew_at:
                    response = await n.wait(self.leases.call_async(Lease.Request(operation='renew',
                        lease_id=lease.lease_id, epoch=lease.epoch)))
                    if not response.ok:
                        raise RuntimeError(response.code)
                    renew_at = time.monotonic()+.2
                await n.pause()
            raise RuntimeError('PICO_GRIPPER_FEEDBACK_TIMEOUT')
        finally:
            await self.release()

    async def switch_grasp(self, intent):
        """Position-mode change only; no contact, force or slip confirmation."""
        n = self.node
        if not self.axial_available:
            raise RuntimeError('PICO_AXIAL_GRASP_UNAVAILABLE')
        if self.modes()[0] != 'closed':
            raise RuntimeError('PICO_LEFT_GRIPPER_NOT_CLOSED')
        target = self.config['grippers']['right']['virtual' if intent.value else 'closed']
        lease = await n.wait(self.leases.call_async(Lease.Request(operation='acquire',
            owner=n.instance_id, resources=[GROUPS['right']])))
        if not lease.ok:
            raise RuntimeError(lease.code)
        self.lease, self.sequence = lease, 0
        try:
            n.current(intent)
            await self.request('authorize_grasp')
            n.current(intent)
            await self.request('virtual' if intent.value else 'rigid', 'right')
            issued = n.now()
            end, renew_at = time.monotonic()+self.config['motion_timeout']+1., 0.
            while time.monotonic() < end:
                n.current(intent)
                if self.modes()[0] != 'closed':
                    raise RuntimeError('PICO_LEFT_GRIPPER_NOT_CLOSED')
                state = self.reading('right')
                if state.state in ('FAILED', 'FAULT'):
                    raise RuntimeError('PICO_GRIPPER_OPERATION_FAILED')
                if (state.position_reached and state.epoch == lease.epoch and
                        state.control_sequence >= self.sequence and seconds(state.feedback_stamp) > issued and
                        abs(state.measured_position-target) <= self.config['position_tolerance']):
                    return
                if time.monotonic() >= renew_at:
                    response = await n.wait(self.leases.call_async(Lease.Request(operation='renew',
                        lease_id=lease.lease_id, epoch=lease.epoch)))
                    if not response.ok:
                        raise RuntimeError(response.code)
                    renew_at = time.monotonic()+.2
                await n.pause()
            raise RuntimeError('PICO_GRIPPER_FEEDBACK_TIMEOUT')
        finally:
            await self.release()
