"""Fenced Snowbot gripper position control; no ROS, IK or force-grasp claims."""
import math
import secrets
import time

SIDES = ('left', 'right')
GROUPS = {'left': 'left_tool_axis', 'right': 'right_tool_axis'}
JOINTS = {'left': 'arm_l_joint_8', 'right': 'arm_r_joint_8'}
MASKS = {'left': 1 << 7, 'right': 1 << 15}


def native_command(position, config):
    """118 protocol: velocity is a reference, not the configured speed limit."""
    values = [float(position), 0., 0., float(config['kp']), float(config['kd'])]
    if not all(math.isfinite(value) for value in values):
        raise ValueError('GRIPPER_COMMAND_NONFINITE')
    return values


def target_position_tolerance(config, *, direct=False):
    """Manual endpoint accuracy is independent of hold and calibrated grasp checks."""
    return config.get('direct_position_tolerance', config['position_tolerance']) if direct else config['position_tolerance']


def validate_config(config, profile):
    if type(config.get('schema_version')) is not int or config['schema_version'] != 1 or type(config.get('calibrated')) is not bool:
        raise ValueError('GRIPPER_CONFIG_SCHEMA')
    if type(config.get('independent_trigger_control', False)) is not bool:
        raise ValueError('GRIPPER_CONFIG_TRIGGER_POLICY')
    if type(config.get('allow_uncalibrated_direct', False)) is not bool:
        raise ValueError('GRIPPER_CONFIG_DIRECT_POLICY')
    for key in ('position_tolerance', 'velocity_tolerance', 'settle_time', 'motion_timeout',
                'stop_timeout', 'feedback_max_age', 'command_ttl'):
        value = config[key]
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError('GRIPPER_CONFIG_TIME_OR_TOLERANCE:'+key)
    if config['feedback_max_age'] > .2 or not .05 <= config['command_ttl'] <= .4:
        raise ValueError('GRIPPER_CONFIG_FRESHNESS')
    if config['settle_time'] > config['motion_timeout'] or config['stop_timeout'] > 5 or config['motion_timeout'] > 30:
        raise ValueError('GRIPPER_CONFIG_TIMEOUT')
    direct_tolerance=target_position_tolerance(config,direct=True)
    if type(direct_tolerance) not in (int,float) or not math.isfinite(direct_tolerance) or direct_tolerance<=0:
        raise ValueError('GRIPPER_CONFIG_DIRECT_TOLERANCE')
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
        if 'direct_position_tolerance' in config and 2*direct_tolerance>=abs(item['open']-item['closed']):
            raise ValueError('GRIPPER_CONFIG_DIRECT_TOLERANCE')
        if not 0 < item['kp'] <= 1000 or not 0 <= item['kd'] <= 1000:
            raise ValueError('GRIPPER_CONFIG_GAIN')
        if not .001 <= item['max_speed'] <= 1:
            raise ValueError('GRIPPER_CONFIG_SPEED')
        if 'virtual' in item:
            value = item['virtual']
            if (side != 'right' or type(value) not in (float, int) or not math.isfinite(value) or
                    not min(item['closed'], item['open']) < value < max(item['closed'], item['open']) or
                    min(abs(value-item['closed']), abs(value-item['open'])) <= 2*config['position_tolerance']):
                raise ValueError('GRIPPER_CONFIG_VIRTUAL_POSITION')
    return config


class GripperController:
    def __init__(self, profile, config, send, health, now, arms_idle, *, enabled=False, steady=time.monotonic,
                 require_driver_ack=True):
        self.profile, self.cfg = profile, validate_config(config, profile)
        self.send, self.health, self.now, self.steady, self.arms_idle = send, health, now, steady, arms_idle
        self.enabled = enabled
        self.require_driver_ack = require_driver_ack
        self.lease_id = self.owner = ''
        self.epoch, self.sequence, self.wire_sequence = 0, -1, 0
        self.command_stamp = float('-inf')
        self.groups = ()
        self.expires = self.steady_expires = 0.
        self.authorized = self.supported = False
        self.grasp_authorized = self.direct_authorized = False
        self.load = 'unknown'
        self.feedback_stamp = self.received_at = self.received_steady = 0.
        self.q, self.v, self.current = {}, {}, {}
        self.invalid, self.fault = '', ''
        self.last_tick = None
        self.recovery_since = None
        self.vr = {}
        self.authorized_at = 0.
        self.trigger_opened = dict.fromkeys(SIDES, False)
        self.trigger_source = None
        self.trigger_frames = {}
        self.last_request_rejection = None
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
        previous = self.feedback_stamp
        self.feedback_stamp, self.received_at, self.received_steady = stamp, now, self.steady()
        if any(not self.profile.limits[n][0] <= q[n] <= self.profile.limits[n][1] for n in names):
            self.invalid = 'GRIPPER_FEEDBACK_OUT_OF_LIMIT'
            raise ValueError(self.invalid)
        self.q, self.v, self.current, self.invalid = dict(q), dict(v), dict(current), ''
        if (self.fault and stamp-previous <= self.cfg['feedback_max_age'] and
                all(abs(v[n]) <= self.cfg['velocity_tolerance'] for n in names)):
            if self.recovery_since is None:self.recovery_since = self.steady()
        else:self.recovery_since = None

    def require_feedback(self):
        age = self.cfg['feedback_max_age']
        if (self.invalid or not self.feedback_stamp or not 0 <= self.now()-self.feedback_stamp <= age
                or not 0 <= self.steady()-self.received_steady <= age):
            raise RuntimeError(self.invalid or 'GRIPPER_FEEDBACK_STALE')
        health = self.health()
        if (type(health.get('sample')) is not int or health['sample'] < 1 or
                type(health.get('operation_enabled_mask')) is not int or health['operation_enabled_mask'] < 0):
            raise RuntimeError('GRIPPER_DRIVER_HEALTH_INVALID')
        if self.require_driver_ack:
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
            for side in SIDES:
                if GROUPS[side] in self.groups and health['gripper_latched_mask'] & (1 << SIDES.index(side)):
                    raise RuntimeError('GRIPPER_DRIVER_LATCHED:'+side)
        return health

    def require_write(self):
        if not self.enabled:
            raise RuntimeError('GRIPPER_COMMANDS_DISABLED')
        if not (self.cfg['calibrated'] or self.cfg.get('allow_uncalibrated_direct', False)):
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
            self.grasp_authorized = self.direct_authorized = False
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
        if operation == 'recover':
            if (side or lease_id or self.lease_id or epoch != self.epoch or
                    not math.isfinite(valid_for) or not 0 < valid_for <= .4 or
                    not math.isfinite(stamp) or not 0 <= self.now()-stamp < valid_for):
                raise RuntimeError('GRIPPER_RECOVERY_AUTHORITY')
            self.recover()
            return
        self.check_lease(lease_id, epoch)
        self.require_write()
        if not self.cfg['calibrated'] and (operation not in ('authorize_direct','open','close','hold') or
                operation in ('open','close') and not self.direct_authorized):
            raise RuntimeError('GRIPPER_UNCALIBRATED_DIRECT_ONLY')
        if (type(sequence) is not int or sequence <= self.sequence or not math.isfinite(stamp)
                or stamp < self.command_stamp or not math.isfinite(valid_for) or not 0 < valid_for <= .4 or not 0 <= self.now()-stamp < valid_for):
            raise RuntimeError('GRIPPER_STALE_COMMAND')
        if operation == 'authorize_direct':
            # Explicit trigger operation, independent of a load declaration.
            # Availability is checked by require_write; only configured endpoints.
            if side or load_state not in ('empty','held','unknown') or external_support is not False:
                raise RuntimeError('GRIPPER_DIRECT_CONTEXT')
            self.load,self.supported,self.authorized=load_state,False,True
            self.grasp_authorized,self.direct_authorized=False,True
            self.vr={}
        elif operation == 'authorize_loading':
            if (side or load_state not in ('empty', 'held', 'unknown') or type(external_support) is not bool or
                    load_state=='unknown' and not external_support):
                raise RuntimeError('GRIPPER_LOADING_CONTEXT')
            self.load, self.supported, self.authorized = load_state, external_support, True
            self.grasp_authorized = self.direct_authorized = False
            self.authorized_at = self.now()
            self.trigger_opened = dict.fromkeys(SIDES, False)
            self.vr = {}  # Every new authorization must see a physical trigger release.
        elif operation == 'authorize_grasp':
            if side or self.groups != (GROUPS['right'],) or 'virtual' not in self.cfg['grippers']['right']:
                raise RuntimeError('GRIPPER_AXIAL_POSITION_UNAVAILABLE')
            self.require_left_closed()
            self.authorized, self.grasp_authorized, self.vr = False, True, {}
            self.direct_authorized = False
        elif operation in ('virtual', 'rigid'):
            if side != 'right' or not self.grasp_authorized:
                raise RuntimeError('GRIPPER_GRASP_NOT_AUTHORIZED')
            self.require_left_closed()
            self.move(side, self.cfg['grippers']['right']['virtual' if operation == 'virtual' else 'closed'], grasp=True)
        elif operation == 'hold':
            self.stop('GRIPPER_HOLD_REQUESTED', revoke=False)
        elif operation in ('open', 'close', 'position'):
            if operation == 'position' and self.direct_authorized:
                raise RuntimeError('GRIPPER_DIRECT_ENDPOINTS_ONLY')
            if operation == 'position' and self.load != 'empty' and not self.supported:
                raise RuntimeError('GRIPPER_POSITION_REQUIRES_SUPPORT')
            target = self.cfg['grippers'][side]['open' if operation == 'open' else 'closed'] if side in SIDES else 0.
            self.move(side, position if operation == 'position' else target)
        else:
            raise RuntimeError('GRIPPER_OPERATION')
        self.sequence = sequence
        self.command_stamp = stamp

    def require_left_closed(self):
        self.require_feedback()
        if (abs(self.q[JOINTS['left']]-self.cfg['grippers']['left']['closed']) > self.cfg['position_tolerance'] or
                abs(self.v[JOINTS['left']]) > self.cfg['velocity_tolerance']):
            raise RuntimeError('GRIPPER_LEFT_NOT_CLOSED_STATIONARY')

    def move(self, side, target, *, grasp=False):
        self.require_write()
        if (side not in SIDES or GROUPS[side] not in self.groups or
                not (self.grasp_authorized and side == 'right' if grasp else self.authorized)):
            raise RuntimeError('GRIPPER_LOADING_NOT_AUTHORIZED')
        channels=(self.channels[side],) if self.direct_authorized else self.channels.values()
        if any(c['state']=='STOPPING' and not
                (self.direct_authorized and c['result']=='SUCCEEDED') for c in channels):
            raise RuntimeError('GRIPPER_STOPPING')
        if grasp:
            self.require_left_closed()
            if target not in (self.cfg['grippers']['right']['virtual'], self.cfg['grippers']['right']['closed']):
                raise RuntimeError('GRIPPER_GRASP_TARGET')
        joint = JOINTS[side]
        lo, hi = self.profile.limits[joint]
        if not math.isfinite(target) or not lo <= target <= hi:
            raise RuntimeError('GRIPPER_LIMIT')
        closed = self.cfg['grippers'][side]['closed']
        if self.direct_authorized and target not in (closed,self.cfg['grippers'][side]['open']):
            raise RuntimeError('GRIPPER_DIRECT_ENDPOINTS_ONLY')
        if not grasp and not self.direct_authorized and abs(target-closed) > abs(self.q[joint]-closed)+self.cfg['position_tolerance'] and self.load != 'empty' and not self.supported:
            raise RuntimeError('GRIPPER_OPEN_REQUIRES_SUPPORT')
        c = self.channels[side]
        health = self.require_feedback()
        ack = self._send(side, target, False)
        c.update(state='MOVING', code='TARGET_SENT', touched=True, target=target, ack=ack,
                 direct=self.direct_authorized and not grasp,
                 manual_close=self.direct_authorized and not grasp and target==closed,
                 arrival_tolerance=target_position_tolerance(self.cfg,direct=self.direct_authorized and not grasp),
                 sent_at=self.now(), sample=health['sample'], settled=None, feedback_sequence=self.feedback_stamp,
                 deadline=self.now()+self.cfg['motion_timeout'], steady_deadline=self.steady()+self.cfg['motion_timeout'])

    def _send(self, side, position, hold):
        self.wire_sequence += 1
        self.send(side, position, hold, self.wire_sequence)
        return self.wire_sequence

    def fail(self, code):
        self.fault = 'GRIPPER_STOP_UNCONFIRMED:'+code
        self.recovery_since = None
        self.lease_id, self.owner, self.authorized = '', '', False
        self.grasp_authorized = self.direct_authorized = False
        for c in self.channels.values():
            if c['touched']:
                c.update(state='FAULT', code=self.fault)

    def recover(self):
        """Operator acknowledgment of a stationary manual hold; sends nothing."""
        if (self.require_driver_ack or self.lease_id or not self.arms_idle() or
                self.fault != 'GRIPPER_STOP_UNCONFIRMED:HOLD_TIMEOUT'):
            raise RuntimeError(self.fault or 'GRIPPER_RECOVERY_NOT_REQUIRED')
        health = self.require_feedback()
        if (self.recovery_since is None or self.steady()-self.recovery_since < self.cfg['settle_time'] or
                any(abs(self.v[JOINTS[s]]) > self.cfg['velocity_tolerance'] for s in SIDES)):
            raise RuntimeError('GRIPPER_RECOVERY_REQUIRES_REST')
        for side,c in self.channels.items():
            if c['touched'] and (not c.get('direct') or
                    not health['operation_enabled_mask'] & MASKS[side] or
                    abs(self.q[JOINTS[side]]-c['hold']) > c.get('hold_tolerance',self.cfg['position_tolerance'])):
                raise RuntimeError('GRIPPER_RECOVERY_HOLD_UNCONFIRMED:'+side)
        self.fault = ''
        self.epoch += 1
        for c in self.channels.values():
            c.update(state='CANCELED',code='OPERATOR_RECOVERED',touched=False)

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
                 hold_tolerance=target_position_tolerance(self.cfg,direct=c.get('direct',False)),
                 deadline=self.now()+self.cfg['stop_timeout'], steady_deadline=self.steady()+self.cfg['stop_timeout'],
                 result=result, result_code=code, settled=None, feedback_sequence=self.feedback_stamp)

    def stop(self, code, *, revoke):
        self.authorized = False
        self.grasp_authorized = self.direct_authorized = False
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
        if self.lease_id and self.grasp_authorized:
            try:
                self.require_left_closed()
            except RuntimeError as exc:
                self.stop(str(exc), revoke=True)
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
                    manual=self.cfg.get('independent_trigger_control',False) and c.get('direct')
                    self._hold(side, 'CANCELED' if manual or c.get('manual_close') else 'FAILED',
                               'MANUAL_ENDPOINT_STOPPED' if manual else
                               'CLOSE_STOPPED' if c.get('manual_close') else 'POSITION_TIMEOUT')
                    continue
                # Native commands have no sequence ACK or hold ACK. Confirm only
                # the requested position against advancing measured samples.
                target = c['hold'] if c['state'] == 'STOPPING' else c['target']
                if self.require_driver_ack and c['state'] == 'STOPPING':
                    target = health['gripper_hold_position'][SIDES.index(side)]
                tolerance=c['arrival_tolerance'] if c['state']=='MOVING' else c['hold_tolerance']
                arrived = (health['sample'] > c['sample']
                    and health.get('operation_enabled_mask', 0) & MASKS[side]
                    and self.feedback_stamp > c['sent_at']
                    and abs(self.q[JOINTS[side]]-target) <= tolerance
                    and abs(self.v[JOINTS[side]]) <= self.cfg['velocity_tolerance'])
                if self.require_driver_ack:
                    i = SIDES.index(side)
                    arrived = arrived and health['gripper_sequence'][i] >= c['ack']
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

    def independent_input(self, frame):
        """Fresh coordinator trigger edges own a separate, expiring gripper lease."""
        if frame.side not in SIDES:return
        # Input can run before the timer: never renew an already expired lease.
        if self.lease_id:
            try:self.check_lease(self.lease_id,self.epoch)
            except RuntimeError:
                self.trigger_frames={}
                return
        if (not frame.valid or frame.stop or frame.init or frame.mapping!='pico_snowbot_v1' or
                not 0<=self.now()-frame.stamp<=self.cfg['feedback_max_age'] or
                not math.isfinite(frame.trigger) or not 0<=frame.trigger<=1 or
                len(frame.button_edges)!=8):
            if self.lease_id:self.stop('GRIPPER_VR_STOP_OR_INVALID',revoke=True)
            self.trigger_frames={}
            return
        if self.trigger_source!=frame.source_id:
            if self.lease_id:self.stop('GRIPPER_VR_SOURCE_CHANGED',revoke=True)
            self.trigger_source=frame.source_id;self.trigger_frames={}
        previous=self.trigger_frames.get(frame.side)
        if previous and frame.seq<=previous[0]:return
        counts=tuple(frame.button_edges[6:8])
        if (not frame.source_id or any(type(x) is not int or x<0 for x in counts) or
                counts[0]-counts[1] not in (0,1) or
                frame.trigger>.7 and counts[0]-counts[1]!=1 or
                frame.trigger<.3 and counts[0]-counts[1]!=0 or
                previous and frame.stamp<previous[3]):
            if self.lease_id:self.stop('GRIPPER_VR_COUNTER_INVALID',revoke=True)
            self.trigger_frames={}
            return
        armed=bool(previous and previous[2]) or frame.trigger<.3
        self.trigger_frames[frame.side]=(frame.seq,counts,armed,frame.stamp)
        # A held trigger on startup/reconnect is not a new command.
        if previous is None:return
        delta=tuple(a-b for a,b in zip(counts,previous[1]))
        if any(x<0 or x>1 for x in delta) or all(delta):
            if self.lease_id:self.stop('GRIPPER_VR_EDGE_AMBIGUOUS',revoke=True)
            self.trigger_frames={}
            return
        if self.lease_id and self.owner!=frame.source_id:return
        if self.lease_id:
            # Refresh only with both fresh sides from this coordinator source.
            if all(side in self.trigger_frames and
                   0<=self.now()-self.trigger_frames[side][3]<=.25 for side in SIDES):
                self.expires,self.steady_expires=self.now()+.25,self.steady()+.25
        if not armed or not any(delta):return
        if not all(side in self.trigger_frames and
                   0<=self.now()-self.trigger_frames[side][3]<=.25 for side in SIDES):return
        try:
            if not self.lease_id:
                self.lease('acquire',frame.source_id,tuple(GROUPS.values()))
                self.command('authorize_direct','',self.lease_id,self.epoch,self.sequence+1,
                             frame.stamp,self.cfg['command_ttl'])
            self.command('open' if delta[0] else 'close',frame.side,self.lease_id,self.epoch,
                         self.sequence+1,frame.stamp,self.cfg['command_ttl'])
            self.expires,self.steady_expires=self.now()+.25,self.steady()+.25
        except RuntimeError as exc:
            self.last_request_rejection = dict(stamp=self.now(),side=frame.side,
                source_id=frame.source_id,seq=frame.seq,code=str(exc))
            leased=bool(self.lease_id)
            if leased:self.stop(str(exc),revoke=True)
            if not leased or not self.channels[frame.side]['touched']:
                # A refusal before lease acquisition used to disappear behind
                # NO_COMMAND. Report it without latching a device fault or
                # retrying a held trigger when conditions later improve.
                self.channels[frame.side].update(state='IDLE',code=str(exc))
            self.trigger_frames={}

    def input(self, frame):
        """Optional trigger control, explicitly bound to a leased VR source id."""
        if self.cfg.get('independent_trigger_control',False):
            return self.independent_input(frame)
        if not self.lease_id or self.direct_authorized or self.owner != frame.source_id or frame.side not in SIDES:
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
            control_sequence=max(0, self.sequence), loading_authorized=self.authorized and not self.direct_authorized,
            trigger_armed=bool(self.authorized and side in self.vr and self.vr[side][2]
                and 0 <= self.now()-self.vr[side][3] <= .25),
            measured_position=self.q.get(JOINTS[side], 0.), measured_velocity=self.v.get(JOINTS[side], 0.),
            measured_current=self.current.get(side, 0.), target_position=c['target'],
            position_reached=c['state'] == 'SUCCEEDED', grasp_state='unknown',
            feedback_stamp=self.feedback_stamp, feedback_valid=bool(self.feedback_stamp and not self.invalid and
                0 <= self.now()-self.feedback_stamp <= self.cfg['feedback_max_age'] and
                0 <= self.steady()-self.received_steady <= self.cfg['feedback_max_age']),
            calibrated=self.cfg['calibrated'])
