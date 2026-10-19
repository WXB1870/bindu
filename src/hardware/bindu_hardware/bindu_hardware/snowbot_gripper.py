"""Fenced Snowbot gripper position control; no ROS, IK or force-grasp claims."""
import math
import secrets
import time

SIDES = ('left', 'right')
GROUPS = {'left': 'left_tool_axis', 'right': 'right_tool_axis'}
JOINTS = {'left': 'arm_l_joint_8', 'right': 'arm_r_joint_8'}
MASKS = {'left': 1 << 7, 'right': 1 << 15}


def validate_config(config, profile):
    if type(config.get('schema_version')) is not int or config['schema_version'] != 1 or type(config.get('calibrated')) is not bool:
        raise ValueError('GRIPPER_CONFIG_SCHEMA')
    for key in ('position_tolerance', 'velocity_tolerance', 'settle_time', 'motion_timeout',
                'stop_timeout', 'feedback_max_age', 'command_ttl'):
        value = config[key]
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError('GRIPPER_CONFIG_TIME_OR_TOLERANCE:'+key)
    if config['feedback_max_age'] > .2 or not .05 <= config['command_ttl'] <= .4:
        raise ValueError('GRIPPER_CONFIG_FRESHNESS')
    if config['settle_time'] > config['motion_timeout'] or config['stop_timeout'] > 5 or config['motion_timeout'] > 30:
        raise ValueError('GRIPPER_CONFIG_TIMEOUT')
    for side in SIDES:
        item = config['grippers'][side]
        if profile.groups[GROUPS[side]] != [JOINTS[side]]:
            raise ValueError('GRIPPER_CONFIG_LAYOUT')
        lo, hi = profile.limits[JOINTS[side]]
        if any(type(item[k]) not in (float, int) or not math.isfinite(item[k])
               for k in ('open', 'closed', 'kp', 'kd', 'max_speed')):
            raise ValueError('GRIPPER_CONFIG_VALUE')
        if not lo <= item['open'] <= hi or not lo <= item['closed'] <= hi or item['open'] == item['closed']:
            raise ValueError('GRIPPER_CONFIG_LIMIT')
        if not 0 < item['kp'] <= 1000 or not 0 <= item['kd'] <= 1000:
            raise ValueError('GRIPPER_CONFIG_GAIN')
        if not .001 <= item['max_speed'] <= 1:
            raise ValueError('GRIPPER_CONFIG_SPEED')
    return config


class GripperController:
    def __init__(self, profile, config, send, health, now, arms_idle, *, enabled=False, steady=time.monotonic):
        self.profile, self.cfg = profile, validate_config(config, profile)
        self.send, self.health, self.now, self.steady, self.arms_idle = send, health, now, steady, arms_idle
        self.enabled = enabled
        self.lease_id = self.owner = ''
        self.epoch, self.sequence, self.wire_sequence = 0, -1, 0
        self.command_stamp = float('-inf')
        self.groups = ()
        self.expires = self.steady_expires = 0.
        self.authorized = self.supported = False
        self.load = 'unknown'
        self.feedback_stamp = self.received_at = self.received_steady = 0.
        self.q, self.v, self.current = {}, {}, {}
        self.invalid, self.fault = '', ''
        self.last_tick = None
        self.vr = {}
        self.authorized_at = 0.
        self.trigger_opened = dict.fromkeys(SIDES, False)
        self.channels = {s: dict(state='IDLE', code='NO_COMMAND', touched=False, target=0.,
            hold=0., ack=0, sent_at=0., sample=0, deadline=0., steady_deadline=0.,
            settled=None, feedback_sequence=0., result='CANCELED', result_code='HELD') for s in SIDES}

    @property
    def busy(self):
        return bool(self.lease_id or self.fault or any(c['state'] in ('MOVING', 'STOPPING') for c in self.channels.values()))

    def observe(self, q, v, current, stamp):
        now = self.now()
        names = tuple(JOINTS.values())
        if (not math.isfinite(stamp) or stamp <= self.feedback_stamp or not 0 <= now-stamp <= self.cfg['feedback_max_age']
                or any(n not in q or n not in v for n in names)
                or any(not math.isfinite(x) for x in [*(q[n] for n in names), *(v[n] for n in names), *current.values()])
                or set(current) != set(SIDES)):
            self.invalid = 'GRIPPER_FEEDBACK_INVALID'
            raise ValueError(self.invalid)
        self.feedback_stamp, self.received_at, self.received_steady = stamp, now, self.steady()
        if any(not self.profile.limits[n][0] <= q[n] <= self.profile.limits[n][1] for n in names):
            self.invalid = 'GRIPPER_FEEDBACK_OUT_OF_LIMIT'
            raise ValueError(self.invalid)
        self.q, self.v, self.current, self.invalid = dict(q), dict(v), dict(current), ''

    def require_feedback(self):
        age = self.cfg['feedback_max_age']
        if (self.invalid or not self.feedback_stamp or not 0 <= self.now()-self.feedback_stamp <= age
                or not 0 <= self.steady()-self.received_steady <= age):
            raise RuntimeError(self.invalid or 'GRIPPER_FEEDBACK_STALE')
        health = self.health()
        if health.get('gripper_control_enabled') is not True:
            raise RuntimeError('GRIPPER_DRIVER_PATCH_REQUIRED')
        sequences, holds = health.get('gripper_sequence'), health.get('gripper_hold_position')
        if (not isinstance(sequences, list) or len(sequences) != 2 or
                any(type(x) is not int or x < 0 for x in sequences) or
                not isinstance(holds, list) or len(holds) != 2 or
                any(type(x) not in (float, int) or not math.isfinite(x) for x in holds) or
                type(health.get('gripper_hold_mask')) is not int or not 0 <= health['gripper_hold_mask'] <= 3 or
                type(health.get('gripper_latched_mask')) is not int or not 0 <= health['gripper_latched_mask'] <= 3):
            raise RuntimeError('GRIPPER_DRIVER_HEALTH_INVALID')
        selected = [s for s in SIDES if GROUPS[s] in self.groups]
        for side in selected:
            if health.get('gripper_latched_mask', 0) & (1 << SIDES.index(side)):
                raise RuntimeError('GRIPPER_DRIVER_LATCHED:'+side)
        return health

    def require_write(self):
        if not self.enabled:
            raise RuntimeError('GRIPPER_COMMANDS_DISABLED')
        if not self.cfg['calibrated']:
            raise RuntimeError('GRIPPER_CALIBRATION_REQUIRED')
        if self.fault:
            raise RuntimeError(self.fault)
        if not self.arms_idle():
            raise RuntimeError('GRIPPER_REQUIRES_STOPPED_ARMS')
        return self.require_feedback()

    def check_lease(self, lease_id, epoch):
        if not self.lease_id or lease_id != self.lease_id or epoch != self.epoch:
            raise RuntimeError('GRIPPER_STALE_LEASE')
        if self.now() >= self.expires or self.steady() >= self.steady_expires:
            self.stop('GRIPPER_LEASE_EXPIRED', revoke=True)
            raise RuntimeError('GRIPPER_LEASE_EXPIRED')

    def lease(self, operation, owner='', groups=(), lease_id='', epoch=0):
        if operation == 'acquire':
            if self.busy:
                raise RuntimeError(self.fault or 'GRIPPER_BUSY')
            if not owner or len(owner) > 128 or not groups or len(set(groups)) != len(groups) or set(groups)-set(GROUPS.values()):
                raise RuntimeError('GRIPPER_LEASE_LAYOUT')
            self.groups = tuple(groups)
            try:
                self.require_write()
            except Exception:
                self.groups = ()
                raise
            self.lease_id, self.owner = secrets.token_hex(16), owner
            self.epoch += 1
            self.sequence, self.authorized, self.vr = -1, False, {}
            self.command_stamp = float('-inf')
            for c in self.channels.values():
                c['touched'] = False
        else:
            self.check_lease(lease_id, epoch)
            if operation == 'release':
                self.stop('GRIPPER_RELEASED', revoke=True)
                return
            if operation != 'renew':
                raise RuntimeError('GRIPPER_LEASE_OPERATION')
            self.require_write()
        self.expires, self.steady_expires = self.now()+2., self.steady()+2.

    def command(self, operation, side, lease_id, epoch, sequence, stamp, valid_for, *, position=0.,
                load_state='unknown', external_support=False):
        self.check_lease(lease_id, epoch)
        self.require_write()
        if (type(sequence) is not int or sequence <= self.sequence or not math.isfinite(stamp)
                or stamp < self.command_stamp or not math.isfinite(valid_for) or not 0 < valid_for <= .4 or not 0 <= self.now()-stamp < valid_for):
            raise RuntimeError('GRIPPER_STALE_COMMAND')
        if operation == 'authorize_loading':
            if (side or load_state not in ('empty', 'held', 'unknown') or type(external_support) is not bool or
                    load_state=='unknown' and not external_support):
                raise RuntimeError('GRIPPER_LOADING_CONTEXT')
            self.load, self.supported, self.authorized = load_state, external_support, True
            self.authorized_at = self.now()
            self.trigger_opened = dict.fromkeys(SIDES, False)
            self.vr = {}  # Every new authorization must see a physical trigger release.
        elif operation == 'hold':
            self.stop('GRIPPER_HOLD_REQUESTED', revoke=False)
        elif operation in ('open', 'close', 'position'):
            if operation == 'position' and self.load != 'empty' and not self.supported:
                raise RuntimeError('GRIPPER_POSITION_REQUIRES_SUPPORT')
            target = self.cfg['grippers'][side]['open' if operation == 'open' else 'closed'] if side in SIDES else 0.
            self.move(side, position if operation == 'position' else target)
        else:
            raise RuntimeError('GRIPPER_OPERATION')
        self.sequence = sequence
        self.command_stamp = stamp

    def move(self, side, target):
        self.require_write()
        if side not in SIDES or GROUPS[side] not in self.groups or not self.authorized:
            raise RuntimeError('GRIPPER_LOADING_NOT_AUTHORIZED')
        if any(c['state'] == 'STOPPING' for c in self.channels.values()):
            raise RuntimeError('GRIPPER_STOPPING')
        joint = JOINTS[side]
        lo, hi = self.profile.limits[joint]
        if not math.isfinite(target) or not lo <= target <= hi:
            raise RuntimeError('GRIPPER_LIMIT')
        closed = self.cfg['grippers'][side]['closed']
        if abs(target-closed) > abs(self.q[joint]-closed)+self.cfg['position_tolerance'] and self.load != 'empty' and not self.supported:
            raise RuntimeError('GRIPPER_OPEN_REQUIRES_SUPPORT')
        c = self.channels[side]
        health = self.require_feedback()
        ack = self._send(side, target, False)
        c.update(state='MOVING', code='TARGET_SENT', touched=True, target=target, ack=ack,
                 sent_at=self.now(), sample=health['sample'], settled=None, feedback_sequence=self.feedback_stamp,
                 deadline=self.now()+self.cfg['motion_timeout'], steady_deadline=self.steady()+self.cfg['motion_timeout'])

    def _send(self, side, position, hold):
        self.wire_sequence += 1
        self.send(side, position, hold, self.wire_sequence)
        return self.wire_sequence

    def fail(self, code):
        self.fault = 'GRIPPER_STOP_UNCONFIRMED:'+code
        self.lease_id, self.owner, self.authorized = '', '', False
        for c in self.channels.values():
            if c['touched']:
                c.update(state='FAULT', code=self.fault)

    def _hold(self, side, result, code):
        c = self.channels[side]
        if not c['touched'] or c['state'] == 'FAULT':
            return
        if c['state'] == 'STOPPING':
            return  # Repeated faults do not refresh the first stop deadline.
        health = self.require_feedback()
        position = self.q[JOINTS[side]]
        ack = self._send(side, position, True)
        c.update(state='STOPPING', code=code, hold=position, ack=ack, sent_at=self.now(), sample=health['sample'],
                 deadline=self.now()+self.cfg['stop_timeout'], steady_deadline=self.steady()+self.cfg['stop_timeout'],
                 result=result, result_code=code, settled=None, feedback_sequence=self.feedback_stamp)

    def stop(self, code, *, revoke):
        self.authorized = False
        errors = []
        for side in SIDES:
            try:
                self._hold(side, 'CANCELED', code)
                if not self.channels[side]['touched']:
                    self.channels[side].update(state='CANCELED', code=code)
            except RuntimeError as exc:
                errors.append(str(exc))
        if revoke:
            self.lease_id, self.owner = '', ''
            self.epoch += 1
        if errors:
            self.fail(','.join(errors))

    def tick(self):
        now, steady = self.now(), self.steady()
        if self.last_tick is not None and (now < self.last_tick or now-self.last_tick > .1):
            if self.busy:
                self.stop('GRIPPER_CLOCK_OR_TICK_GAP', revoke=True)
        self.last_tick = now
        if self.lease_id and (now >= self.expires or steady >= self.steady_expires):
            self.stop('GRIPPER_LEASE_EXPIRED', revoke=True)
        if self.lease_id and not self.arms_idle():
            self.stop('GRIPPER_ARMS_ACTIVE', revoke=True)
        if self.lease_id and self.vr and now-self.authorized_at > .25:
            if any(s not in self.vr or not 0 <= now-self.vr[s][3] <= .25 for s in SIDES if GROUPS[s] in self.groups):
                self.stop('GRIPPER_VR_INPUT_TIMEOUT', revoke=True)
        for side, c in self.channels.items():
            if c['state'] not in ('MOVING', 'STOPPING'):
                continue
            try:
                health = self.require_feedback()
                if now >= c['deadline'] or steady >= c['steady_deadline']:
                    if c['state'] == 'STOPPING':
                        raise RuntimeError('HOLD_TIMEOUT')
                    self._hold(side, 'FAILED', 'POSITION_TIMEOUT')
                    continue
                ack = health.get('gripper_sequence', [])
                i = SIDES.index(side)
                target = health['gripper_hold_position'][i] if c['state'] == 'STOPPING' else c['target']
                arrived = (len(ack) == 2 and ack[i] >= c['ack'] and health['sample'] > c['sample']
                    and health.get('operation_enabled_mask', 0) & MASKS[side]
                    and self.feedback_stamp > c['sent_at']
                    and abs(self.q[JOINTS[side]]-target) <= self.cfg['position_tolerance']
                    and abs(self.v[JOINTS[side]]) <= self.cfg['velocity_tolerance'])
                if c['state'] == 'STOPPING':
                    arrived = arrived and bool(health['gripper_hold_mask'] & (1 << i))
                if c['state'] == 'MOVING':
                    self._send(side, target, False)
                # Only advancing measured samples can establish a stable interval.
                if self.feedback_stamp > c['feedback_sequence']:
                    c['feedback_sequence'] = self.feedback_stamp
                    c['settled'] = (steady if c['settled'] is None else c['settled']) if arrived else None
                    if c['settled'] is not None and steady-c['settled'] >= self.cfg['settle_time']:
                        if c['state'] == 'MOVING':
                            self._hold(side, 'SUCCEEDED', 'POSITION_REACHED')
                        else:
                            c.update(state=c['result'], code=c['result_code'])
            except RuntimeError as exc:
                self.stop(str(exc), revoke=True)
                self.fail(str(exc))

    def input(self, frame):
        """Optional trigger control, explicitly bound to a leased VR source id."""
        if not self.lease_id or self.owner != frame.source_id or frame.side not in SIDES:
            return
        if GROUPS[frame.side] not in self.groups:
            return
        if (not self.authorized or not frame.valid or frame.stop or frame.init or frame.mapping != 'pico_snowbot_v1'
                or frame.buttons or frame.virtual_grip or frame.grip >= .3
                or not 0 <= self.now()-frame.stamp <= self.cfg['feedback_max_age']
                or len(frame.button_edges) != 8):
            if self.authorized:
                self.stop('GRIPPER_VR_STOP_OR_INVALID', revoke=True)
            return
        previous = self.vr.get(frame.side)
        if previous and frame.seq <= previous[0]:
            return
        if previous and frame.stamp < previous[3]:
            self.stop('GRIPPER_VR_TIME_REVERSED', revoke=True)
            return
        counts = tuple(frame.button_edges[6:8])
        if (any(type(x) is not int or x < 0 for x in counts) or counts[0]-counts[1] not in (0, 1)
                or not all(math.isfinite(x) and 0 <= x <= 1 for x in (frame.trigger, frame.grip))):
            self.stop('GRIPPER_VR_COUNTER_INVALID', revoke=True)
            return
        armed = bool(previous and previous[2]) or frame.trigger < .3
        self.vr[frame.side] = (frame.seq, counts, armed, frame.stamp)
        self.expires, self.steady_expires = self.now()+.25, self.steady()+.25
        if previous:
            delta = tuple(a-b for a, b in zip(counts, previous[1]))
            if any(x < 0 or x > 1 for x in delta) or all(delta):
                self.stop('GRIPPER_VR_EDGE_AMBIGUOUS', revoke=True)
                return
            if armed and (delta[0] or delta[1] and self.trigger_opened[frame.side]):
                try:
                    self.move(frame.side, self.cfg['grippers'][frame.side]['open' if delta[0] else 'closed'])
                    self.trigger_opened[frame.side] = bool(delta[0])
                except RuntimeError as exc:
                    self.stop(str(exc), revoke=True)

    def snapshot(self, side):
        c = self.channels[side]
        return dict(side=side, joint_name=JOINTS[side], state=c['state'], code=c['code'],
            lease_active=bool(self.lease_id), owner=self.owner, epoch=self.epoch,
            control_sequence=max(0, self.sequence), loading_authorized=self.authorized,
            trigger_armed=bool(self.authorized and side in self.vr and self.vr[side][2]
                and 0 <= self.now()-self.vr[side][3] <= .25),
            measured_position=self.q.get(JOINTS[side], 0.), measured_velocity=self.v.get(JOINTS[side], 0.),
            measured_current=self.current.get(side, 0.), target_position=c['target'],
            position_reached=c['state'] == 'SUCCEEDED', grasp_state='unknown',
            feedback_stamp=self.feedback_stamp, feedback_valid=bool(self.feedback_stamp and not self.invalid and
                0 <= self.now()-self.feedback_stamp <= self.cfg['feedback_max_age'] and
                0 <= self.steady()-self.received_steady <= self.cfg['feedback_max_age']),
            calibrated=self.cfg['calibrated'])
