"""PICO operations against the shared arm-target and gripper service contract."""
import hashlib
import json
import math
import time
from pathlib import Path
from bindu_interfaces.msg import GripperState
from bindu_interfaces.srv import GripperControl, Lease
from bindu_hardware.snowbot_gripper import SIDES, GROUPS, JOINTS, validate_config, target_position_tolerance
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
        self.lease_instance = None
        self.acquire_future = None
        self.release_pending = False
        self.stop_unconfirmed = False
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
        if self.stop_unconfirmed:
            raise RuntimeError('PICO_GRIPPER_STOP_UNCONFIRMED')
        msg = self.states.get(side)
        if (msg is None or n.state is None or msg.instance_id != n.state.instance_id or
                self.lease_instance is not None and msg.instance_id != self.lease_instance or
                not msg.feedback_valid or not (msg.calibrated or self.config.get('allow_uncalibrated_direct', False)) or
                not 0 <= n.now()-seconds(msg.stamp) <= .2 or
                not 0 <= n.now()-seconds(msg.feedback_stamp) <= .2 or
                not all(math.isfinite(v) for v in (msg.measured_position, msg.measured_velocity))):
            raise RuntimeError('PICO_GRIPPER_FEEDBACK_OR_CALIBRATION_REQUIRED:'+side)
        if msg.state == 'FAULT':
            raise RuntimeError(msg.code)
        return msg

    def modes(self, *, require_stationary=True):
        if not self.config['calibrated']:
            return ('unknown', 'unknown')  # Manual endpoints do not confirm a tool grasp.
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
        if (self.stop_unconfirmed or self.acquire_future is not None or self.release_pending or
                self.lease is not None):
            return False
        try:
            return all(s.state not in ('MOVING','STOPPING','FAULT') and
                abs(s.measured_velocity) <= self.config['velocity_tolerance']
                for s in (self.reading(side) for side in SIDES))
        except RuntimeError:
            return False

    def lease_states(self):
        states = [self.reading(side) for side in SIDES]
        if self.lease is None or not all(s.epoch == self.lease.epoch and s.lease_active and
                s.owner == self.node.instance_id for s in states):
            raise RuntimeError('PICO_GRIPPER_LEASE_FEEDBACK_MISMATCH')
        return states

    async def request(self, operation, side='', **kwargs):
        n = self.node
        self.lease_states()
        self.sequence += 1
        response = await n.wait(self.control.call_async(GripperControl.Request(
            operation=operation, side=side, lease_id=self.lease.lease_id,
            epoch=self.lease.epoch, sequence=self.sequence, stamp=stamp(n.now()),
            valid_for=.25, **kwargs)))
        if not response.accepted:
            raise RuntimeError(response.code)

    async def acquire(self, resources):
        if self.stop_unconfirmed or self.acquire_future is not None:
            raise RuntimeError('PICO_GRIPPER_STOP_UNCONFIRMED')
        if self.lease is not None:
            raise RuntimeError('PICO_GRIPPER_LEASE_ACTIVE')
        n = self.node
        for side in SIDES:
            self.reading(side)
        self.lease_instance = n.state.instance_id
        end = time.monotonic()+2.
        try:
            future = self.leases.call_async(Lease.Request(operation='acquire',
                owner=n.instance_id, resources=list(resources)))
            self.acquire_future = future
            lease = await n.wait(future, timeout=max(0., end-time.monotonic()))
        except Exception as exc:
            # A timed-out request may still acquire at the original executor.
            # Retain its Future and latch; a replacement's idle is no evidence.
            self.stop_unconfirmed = True
            raise RuntimeError('PICO_GRIPPER_STOP_UNCONFIRMED:ACQUIRE_RESULT_UNKNOWN') from exc
        self.acquire_future = None
        if not lease.ok:
            self.lease_instance = None
            raise RuntimeError(lease.code)
        self.lease, self.sequence = lease, 0
        try:
            if lease.profile_hash != n.profile.digest:
                raise RuntimeError('PICO_GRIPPER_PROFILE_MISMATCH')
            while True:
                if time.monotonic() >= end:
                    raise RuntimeError('PICO_GRIPPER_LEASE_FEEDBACK_TIMEOUT')
                try:
                    self.lease_states()
                    break
                except RuntimeError:
                    if time.monotonic() >= end:
                        raise RuntimeError('PICO_GRIPPER_LEASE_FEEDBACK_TIMEOUT')
                    await n.pause()
        except Exception:
            await self.release()
            raise
        return lease

    async def release(self):
        if self.stop_unconfirmed:
            raise RuntimeError('PICO_GRIPPER_STOP_UNCONFIRMED')
        if self.lease is None:
            return
        n, lease = self.node, self.lease
        started = n.now()
        self.release_pending = True
        end = time.monotonic()+self.config['stop_timeout']+.5
        try:
            response = await n.wait(self.leases.call_async(Lease.Request(
                operation='release', lease_id=lease.lease_id, epoch=lease.epoch)),
                timeout=max(0., min(2., end-time.monotonic())))
            already_revoked=(not response.ok and response.code=='GRIPPER_STALE_LEASE' and
                not response.lease_id and response.profile_hash==n.profile.digest and
                response.epoch==lease.epoch+1)
            if ((not response.ok and not already_revoked) or
                    response.profile_hash != n.profile.digest or response.epoch != lease.epoch+1):
                raise RuntimeError('PICO_GRIPPER_RELEASE_AUTHORITY_UNKNOWN')
            while time.monotonic() < end:
                states = [self.reading(s) for s in SIDES]
                if all(s.instance_id == self.lease_instance and seconds(s.feedback_stamp) > started and
                       s.epoch == lease.epoch+1 and not s.lease_active and
                       s.state not in ('MOVING','STOPPING') and
                       abs(s.measured_velocity) <= self.config['velocity_tolerance'] for s in states):
                    self.lease = None
                    self.lease_instance = None
                    self.release_pending = False
                    return
                await n.pause()
            raise RuntimeError('PICO_GRIPPER_STOP_UNCONFIRMED')
        except Exception as exc:
            self.stop_unconfirmed = True
            # Keep the original ownership until confirmed; neither B, a late
            # reply nor a reconnect may turn an unknown stop into permission.
            raise RuntimeError('PICO_GRIPPER_STOP_UNCONFIRMED:'+str(exc)) from exc

    async def move_grippers(self, intent, *, direct=False):
        n = self.node
        targets={s:m for s,m in zip(SIDES,intent.value) if not direct or m is not None}
        if not targets or any(m not in ('open','closed') for m in targets.values()):
            raise RuntimeError('PICO_GRIPPER_TARGET_INVALID')
        lease = await self.acquire([GROUPS[s] for s in (SIDES if direct else targets)])
        try:
            n.current(intent)
            if direct:
                n.control.gripper_update_token=intent.token
            await self.request('authorize_direct' if direct else 'authorize_loading', load_state=n.control.load,
                               external_support=False if direct else n.control.supported)
            expected, sent_modes, failed, renew_at = {}, {}, {}, 0.
            while True:
                n.current(intent)
                if direct:
                    targets={s:m for s,m in zip(SIDES,n.control.pending.value) if m is not None}
                for side, mode in targets.items():
                    if sent_modes.get(side)==mode:
                        continue
                    n.current(intent)
                    if mode not in ('open','closed'):
                        raise RuntimeError('PICO_GRIPPER_TARGET_INVALID')
                    await self.request('open' if mode=='open' else 'close',side)
                    sent_modes[side]=mode
                    failed.pop(side,None)
                    expected[side]=(self.config['grippers'][side][mode],self.sequence,n.now(),
                                    time.monotonic()+self.config['motion_timeout']+1.)
                n.current(intent)  # A stop may have arrived while a request awaited its reply.
                states={s:self.reading(s) for s in expected}
                for side,s in states.items():
                    if s.state=='FAULT':
                        raise RuntimeError('PICO_GRIPPER_OPERATION_FAILED')
                    if (s.state=='FAILED' and s.epoch==lease.epoch and
                            s.control_sequence>=expected[side][1] and
                            seconds(s.stamp)>expected[side][2]):
                        if not direct or s.code!='POSITION_TIMEOUT':
                            raise RuntimeError('PICO_GRIPPER_OPERATION_FAILED')
                        # FAILED/POSITION_TIMEOUT follows confirmed local hold.
                        # Keep the partner's command/lease and await a NEW input
                        # for this side. Never retry or report this target reached.
                        if side not in failed:
                            n.event('PICO_GRIPPER_SIDE_FAILED',json.dumps(dict(
                                side=side,code=s.code,target=expected[side][0],
                                measured=s.measured_position,epoch=s.epoch,
                                control_sequence=s.control_sequence)))
                        failed[side]=s.code
                if direct:
                    n.control.reason=('GRIPPER_POSITION_TIMEOUT:'+','.join(sorted(failed)) if failed else
                                      'TRIGGERS_CONTROL_INDIVIDUAL_GRIPPERS')
                reached={side:(s.position_reached and s.epoch==lease.epoch and
                    s.control_sequence>=expected[side][1] and seconds(s.feedback_stamp)>expected[side][2] and
                    abs(s.measured_position-expected[side][0])<=target_position_tolerance(self.config,direct=direct))
                    for side,s in states.items()}
                if reached and not failed and all(reached.values()):
                    return
                if any(not reached[side] and side not in failed and
                       time.monotonic()>=expected[side][3] for side,s in states.items()):
                    raise RuntimeError('PICO_GRIPPER_FEEDBACK_TIMEOUT')
                if time.monotonic() >= renew_at:
                    response = await n.wait(self.leases.call_async(Lease.Request(operation='renew',
                        lease_id=lease.lease_id, epoch=lease.epoch)))
                    if not response.ok:
                        raise RuntimeError(response.code)
                    renew_at = time.monotonic()+.2
                await n.pause()
        finally:
            if direct:
                n.control.gripper_update_token=None
            await self.release()

    async def switch_grasp(self, intent):
        """Position-mode change only; no contact, force or slip confirmation."""
        n = self.node
        if not self.axial_available:
            raise RuntimeError('PICO_AXIAL_GRASP_UNAVAILABLE')
        if self.modes()[0] != 'closed':
            raise RuntimeError('PICO_LEFT_GRIPPER_NOT_CLOSED')
        target = self.config['grippers']['right']['virtual' if intent.value else 'closed']
        lease = await self.acquire([GROUPS['right']])
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
