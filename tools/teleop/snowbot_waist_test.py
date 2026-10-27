#!/usr/bin/env python3
"""Single waist-axis diagnostic. Preview is offline; --execute uses an existing executor.

The operator starts the reviewed driver separately. This program never starts/
stops drivers or changes limits. Session mode owns 16 waist/arm joints.
The waist_topic backend requires standalone servo and only publishes the two
waist positions to /body/motor_cmd; it never writes arm/head commands.
Use without a tool: moving the waist moves both arms in the world frame.
"""
import argparse
from collections import Counter
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import statistics
import time
import uuid

GROUPS = ('waist', 'left_arm', 'right_arm')
NAMES = ('body_joint_1', 'body_joint_2',
         *(f'arm_{side}_joint_{i}' for side in ('l', 'r') for i in range(1, 8)))
SLOTS = (18, 19, *range(7), *range(8, 15))
MASK = sum(1 << i for i in SLOTS)
ENCODER_PROFILES = {'legacy-65536': (65536, 65536), 'outer-262144': (262144, 65536)}


def select_encoder(encoder_profile=None, position_release=None, declarations=()):
    """Select code conversion from explicit provenance, never from the tool version.

    CSP diagnostic wire version 1 exists with both position resolutions. A log
    without scale metadata therefore cannot identify its encoder conversion.
    """
    choices = []

    def add(value, source):
        if not isinstance(value, dict):
            raise ValueError('CSP_ENCODER_METADATA_INVALID')
        pair = tuple(value.get(key) for key in ('position_counts_per_rev', 'speed_counts_per_rev'))
        if any(type(v) is not int for v in pair) or pair not in ENCODER_PROFILES.values():
            raise ValueError('CSP_ENCODER_RESOLUTION_UNSUPPORTED')
        choices.append((pair, source))

    if encoder_profile is not None:
        if not isinstance(encoder_profile, str) or encoder_profile not in ENCODER_PROFILES:
            raise ValueError('CSP_ENCODER_PROFILE_INVALID')
        position, speed = ENCODER_PROFILES[encoder_profile]
        add(dict(position_counts_per_rev=position, speed_counts_per_rev=speed),
            dict(kind='explicit_profile', name=encoder_profile))
    if position_release is not None:
        path = Path(position_release).resolve()
        raw = path.read_bytes()
        release = json.loads(raw)
        if not isinstance(release, dict) or 'csp_encoder' not in release:
            raise ValueError('CSP_ENCODER_METADATA_REQUIRED: old releases need --encoder-profile')
        add(release['csp_encoder'], dict(kind='position_release', path=str(path),
                                       sha256=hashlib.sha256(raw).hexdigest()))
    for value, source in declarations:
        add(value, source)
    if not choices:
        return None
    if len({pair for pair, _ in choices}) != 1:
        raise ValueError('CSP_ENCODER_SCALE_CONFLICT')
    position, speed = choices[0][0]
    return dict(position_counts_per_rev=position, speed_counts_per_rev=speed,
                sources=[source for _, source in choices], physical_units_verified=False,
                running_driver_identity_verified=False)


@dataclass(frozen=True)
class Plan:
    amplitude: float
    move_seconds: float
    dwell: float = 1.

    @property
    def duration(self):
        return self.dwell + 4 * (self.move_seconds + self.dwell)

    def sample(self, elapsed):
        """Offset, speed, acceleration; quintic moves with stationary dwells."""
        if not math.isfinite(elapsed) or elapsed < 0:
            raise ValueError('INVALID_ELAPSED_TIME')
        elapsed -= self.dwell
        previous = 0.
        for target in (self.amplitude, 0., -self.amplitude, 0.):
            if elapsed < 0:
                return previous, 0., 0.
            if elapsed < self.move_seconds:
                u = elapsed / self.move_seconds
                delta = target - previous
                return (previous + delta * (10*u**3 - 15*u**4 + 6*u**5),
                        delta * (30*u**2 - 60*u**3 + 30*u**4) / self.move_seconds,
                        delta * (60*u - 180*u**2 + 120*u**3) / self.move_seconds**2)
            elapsed -= self.move_seconds + self.dwell
            previous = target
        return 0., 0., 0.


def make_plan(degrees=1., speed=.1, acceleration=.5, jerk=10.):
    for value, maximum in ((degrees, 20.), (speed, .375), (acceleration, 2.25), (jerk, 162.)):
        if type(value) not in (float, int) or not math.isfinite(value) or not 0 < value <= maximum:
            raise ValueError('TEST_ENVELOPE_EXCEEDED')
    distance = math.radians(degrees)
    duration = max(1.875*distance/speed, math.sqrt(10/math.sqrt(3)*distance/acceleration),
                   (60*distance/jerk)**(1/3), .2)
    return Plan(distance, math.ceil(duration/.02)*.02)



@dataclass(frozen=True)
class SinePlan:
    amplitude: float
    frequency: float
    cycles: int
    dwell: float = 1.

    @property
    def period(self):
        return 1. / self.frequency

    @property
    def move_seconds(self):
        return self.period / 4.

    @property
    def duration(self):
        return 2*self.dwell + self.cycles*self.period

    @property
    def dynamic_bounds(self):
        omega = 2*math.pi*self.frequency
        dw, ddw, dddw = 1.875/self.period, 10/math.sqrt(3)/self.period**2, 60/self.period**3
        return (self.amplitude*(dw+omega),
                self.amplitude*(ddw+2*dw*omega+omega**2),
                self.amplitude*(dddw+3*ddw*omega+3*dw*omega**2+omega**3))

    def sample(self, elapsed):
        if not math.isfinite(elapsed) or elapsed < 0:
            raise ValueError('INVALID_ELAPSED_TIME')
        t = elapsed-self.dwell
        total = self.cycles*self.period
        if t <= 0 or t >= total:
            return 0., 0., 0.
        u = min(t, total-t, self.period)/self.period
        sign = 1 if t < self.period else -1 if t > total-self.period else 0
        w = 10*u**3-15*u**4+6*u**5
        dw = sign*(30*u**2-60*u**3+30*u**4)/self.period
        ddw = (60*u-180*u**2+120*u**3)/self.period**2 if sign else 0.
        omega = 2*math.pi*self.frequency
        s, c = math.sin(omega*t), math.cos(omega*t)
        return (self.amplitude*w*s, self.amplitude*(dw*s+w*omega*c),
                self.amplitude*(ddw*s+2*dw*omega*c-w*omega**2*s))


def make_sine_plan(degrees=1., frequency=.2, cycles=5, speed=.1, acceleration=.5, jerk=10.):
    make_plan(degrees, speed, acceleration, jerk)
    if (type(frequency) not in (int, float) or not math.isfinite(frequency) or
            not .05 <= frequency <= 1. or type(cycles) is not int or not 3 <= cycles <= 20):
        raise ValueError('INVALID_SINE_PARAMETERS')
    plan = SinePlan(math.radians(degrees), frequency, cycles)
    if any(bound > limit for bound, limit in zip(plan.dynamic_bounds, (speed, acceleration, jerk))):
        raise ValueError('SINE_EXCEEDS_REQUESTED_DYNAMICS')
    return plan


def signal_metadata(plan):
    if isinstance(plan, SinePlan):
        return dict(waveform='sine', frequency_hz=plan.frequency, cycles=plan.cycles,
                    envelope_cycles=1, dwell_s=plan.dwell,
                    dynamic_upper_bounds=dict(zip(('speed', 'acceleration', 'jerk'), plan.dynamic_bounds)))
    return dict(waveform='quintic')


def checked_state(msg, now, received, steady, profile_hash, *, owned=False, epoch=0, stationary=True):
    feedback_age = now-(msg.feedback_stamp.sec+msg.feedback_stamp.nanosec*1e-9)
    if (msg.runtime_mode != 'real' or msg.simulated or msg.device_backend != 'snowbot_vendor' or
            msg.profile_hash != profile_hash or msg.stop_failures or
            not 0 <= steady-received <= .1 or
            not 0 <= feedback_age <= .2 or
            msg.state in ('FAILED', 'STOPPING') or
            msg.code not in ('OK', 'CANCELED', 'FEEDBACK_CONFIRMED', 'STARTUP_READY')):
        raise RuntimeError('EXECUTION_NOT_READY:state='+msg.state+';code='+msg.code+
                           ';feedback_age='+str(round(feedback_age, 4))+
                           ';state_age='+str(round(steady-received, 4))+
                           ';profile_match='+str(msg.profile_hash == profile_hash)+
                           ';stop_failures='+str(list(msg.stop_failures)))
    if (msg.lease_active != owned or (owned and
            (msg.lease_epoch != epoch or set(msg.lease_resources) != set(GROUPS)))):
        raise RuntimeError('EXECUTION_OWNERSHIP_CHANGED')
    if len(msg.joints.name) != len(set(msg.joints.name)) or not (
            len(msg.joints.name) == len(msg.joints.position) == len(msg.joints.velocity)):
        raise RuntimeError('MEASUREMENT_LAYOUT')
    q, v = dict(zip(msg.joints.name, msg.joints.position)), dict(zip(msg.joints.name, msg.joints.velocity))
    if any(n not in q or not all(math.isfinite(x) for x in (q[n], v[n])) for n in NAMES):
        raise RuntimeError('MEASUREMENT_MISSING_OR_NONFINITE')
    if stationary and not owned and any(abs(v[n]) > .01 for n in NAMES):
        raise RuntimeError('MEASUREMENT_NOT_STATIONARY')
    return q, v


def checked_pdo(msg, now, received, steady, boot):
    if (not 0 <= steady-received <= .05 or msg.boot_id != boot or not msg.pdo_sample or
            not 0 <= now-(msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9) <= .05 or
            not math.isfinite(msg.pdo_age) or not 0 <= msg.pdo_age <= .02 or
            msg.expected_wkc <= 0 or msg.wkc != msg.expected_wkc or msg.valid_mask & MASK != MASK or
            len(msg.status) != 20 or any(msg.status[i] & 8 for i in SLOTS) or
            len(msg.position) != 20 or len(msg.speed) != 20 or
            any(not math.isfinite(x) for i in SLOTS for x in (msg.position[i], msg.speed[i]))):
        raise RuntimeError('PDO_NOT_READY')


def checked_pdo_source(publishers, *, discovering=False):
    # DDS can expose the endpoint before its participant/node name arrives.
    if discovering and (not publishers or (len(publishers) == 1 and
            publishers[0].node_name == '_NODE_NAME_UNKNOWN_')):
        return False
    if len(publishers) != 1 or publishers[0].node_name != 'servo_node':
        raise RuntimeError('PDO_SOURCE_NOT_UNIQUE_SERVO:count='+str(len(publishers))+
                           ';nodes='+','.join(p.node_name for p in publishers))
    return True


def json_value(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: json_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_value(item) for item in value]
    return value


def targets(plan, elapsed, initial, limits, axis='body_joint_1'):
    if axis not in NAMES[:2]:
        raise ValueError('TEST_AXIS_INVALID')
    if isinstance(plan, SinePlan):
        lo, hi = limits[axis]
        if not lo+.01 <= initial[axis]-plan.amplitude <= initial[axis]+plan.amplitude <= hi-.01:
            raise ValueError('TEST_POSITION_LIMIT:'+axis)
    offset, _, _ = plan.sample(elapsed)
    q = {n: initial[n] for n in NAMES}
    q[axis] += offset
    for n in NAMES:
        # Extra excursion clearance applies only to the axis being moved.
        margin = .01 if n == axis else 0.
        if not math.isfinite(q[n]) or not limits[n][0]+margin <= q[n] <= limits[n][1]-margin:
            raise ValueError('TEST_POSITION_LIMIT:'+n)
    return q



def checked_waist_topic_pdo(msg, now, received, steady, boot=None):
    if getattr(msg, 'session_selected', False):
        raise RuntimeError('WAIST_TOPIC_SESSION_ACTIVE: standalone servo required')
    if (not 0 <= steady-received <= .05 or not msg.pdo_sample or
            boot is not None and msg.boot_id != boot or
            not 0 <= now-(msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9) <= .05 or
            not math.isfinite(msg.pdo_age) or not 0 <= msg.pdo_age <= .02 or
            msg.expected_wkc <= 0 or msg.wkc != msg.expected_wkc or
            msg.valid_mask & (3 << 18) != (3 << 18) or
            len(msg.position) != 20 or len(msg.speed) != 20 or len(msg.status) != 20 or
            any(msg.status[i] & 8 or not all(math.isfinite(v) for v in
                (msg.position[i], msg.speed[i])) for i in (18, 19)) or
            len(msg.csp_mode_display) != 4 or any(m != 8 for m in msg.csp_mode_display[2:])):
        raise RuntimeError('WAIST_TOPIC_PDO_NOT_READY')
    return tuple(msg.position[18:20])


def waist_topic_targets(plan, elapsed, initial, limits, axis):
    indices = (0, 1) if axis == 'both' else (NAMES[:2].index(axis),)
    if len(initial) != 2 or any(not math.isfinite(q) for q in initial):
        raise ValueError('WAIST_TOPIC_BASELINE_INVALID')
    for i, name in enumerate(NAMES[:2]):
        lo, hi = limits[name]
        excursion = plan.amplitude+.01 if i in indices else 0.
        if not lo+excursion <= initial[i] <= hi-excursion:
            raise ValueError('TEST_POSITION_LIMIT:'+name)
    q = list(initial)
    offset = plan.sample(elapsed)[0]
    for index in indices:
        q[index] += offset
    return q


def execute_waist_topic(args, plan):
    """Standalone legacy waist topic only; no arm/head publishers or leases."""
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import Float64MultiArray
    from snowbot_ctl_msgs.msg import PositionObservation, PositionSessionState
    if not args.robot_config or not args.output:
        raise ValueError('--execute requires --robot-config and --output')
    config_path = args.robot_config.resolve()
    config = json.loads(config_path.read_text())
    profile = json.loads((config_path.parent/config['profile']).read_text())
    execution = profile['execution']
    for axis in NAMES[:2] if args.axis == 'both' else (args.axis,):
        dynamics = execution.get('joint_dynamics', {}).get(axis, {})
        caps = (dynamics.get('velocity', profile['max_speed']),
                dynamics.get('acceleration', execution['max_acceleration']),
                dynamics.get('jerk', execution['max_jerk']))
        if any(v > cap for v, cap in zip((args.speed, args.acceleration, args.jerk), caps)):
            raise ValueError('TEST_EXCEEDS_SELECTED_PROFILE:'+axis)
    encoder = select_encoder(args.encoder_profile, args.position_release)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    log = args.output.open('x')
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('waist_topic_test_'+uuid.uuid4().hex[:8],
                            enable_rosout=False, start_parameter_services=False)
    prefix = '/'+args.vendor_namespace.strip('/') if args.vendor_namespace.strip('/') else ''
    command_topic = prefix+'/body/motor_cmd'
    latest = {}; last_sample = None; progress = time.monotonic(); failure = None; sent = 0

    def record(kind, **data):
        log.write(json.dumps(dict(kind=kind, steady=time.monotonic(), data=json_value(data)),
                             allow_nan=False)+'\n')

    def observe(msg):
        nonlocal last_sample, progress
        now = time.monotonic()
        if msg.pdo_sample != last_sample:
            last_sample, progress = msg.pdo_sample, now
        latest['pdo'] = (msg, now)
        if now-latest.get('logged', 0.) >= .01:
            record('waist_pdo', stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                   boot_id=msg.boot_id, sample=msg.pdo_sample, pdo_age=msg.pdo_age,
                   wkc=msg.wkc, expected_wkc=msg.expected_wkc, valid_mask=msg.valid_mask,
                   position=list(msg.position[18:20]), speed=list(msg.speed[18:20]),
                   status=list(msg.status[18:20]), mode=list(msg.csp_mode_display[2:4]),
                   raw_target=list(msg.csp_target_position_raw[2:4]),
                   raw_actual=list(msg.csp_actual_position_raw[2:4]),
                   session_selected=msg.session_selected)
            latest['logged'] = now

    node.create_subscription(PositionObservation, prefix+'/bindu/position/observation',
                             observe, qos_profile_sensor_data)
    node.create_subscription(PositionSessionState, prefix+'/bindu/position/servo_state',
                             lambda m: latest.update(state=(m, time.monotonic())), 10)
    publisher = node.create_publisher(Float64MultiArray, command_topic, 1)

    def check(discovering=False, boot=None):
        if not checked_pdo_source(node.get_publishers_info_by_topic(
                prefix+'/bindu/position/observation'), discovering=discovering):
            return None
        if 'pdo' not in latest or 'state' not in latest:
            if discovering: return None
            raise RuntimeError('WAIST_TOPIC_FEEDBACK_MISSING')
        state, received = latest['state']
        if state.selected:
            raise RuntimeError('WAIST_TOPIC_SESSION_ACTIVE: standalone servo required')
        if not state.startup_complete:
            if discovering: return None
            raise RuntimeError('WAIST_TOPIC_STARTUP_NOT_COMPLETE')
        if state.control_source != 'position' or not 0 <= time.monotonic()-received <= .2:
            raise RuntimeError('WAIST_TOPIC_SERVO_NOT_READY')
        pubs = node.get_publishers_info_by_topic(command_topic)
        subscriptions = publisher.get_subscription_count()
        if (len(pubs) != 1 or pubs[0].node_name != node.get_name() or subscriptions != 1):
            pending = not pubs or (len(pubs) == 1 and pubs[0].node_name in
                                   (node.get_name(), '_NODE_NAME_UNKNOWN_'))
            if discovering and pending and subscriptions <= 1: return None
            raise RuntimeError('WAIST_TOPIC_COMPETING_OR_MISSING_ENDPOINT')
        msg, received = latest['pdo']
        if time.monotonic()-progress > .05:
            raise RuntimeError('WAIST_TOPIC_PDO_NOT_ADVANCING')
        return checked_waist_topic_pdo(msg, node.get_clock().now().nanoseconds*1e-9,
                                      received, time.monotonic(), boot)

    try:
        deadline = time.monotonic()+5.; baseline = None; rest_since = None
        while True:
            rclpy.spin_once(node, timeout_sec=.005)
            q = check(discovering=True)
            if q is not None:
                msg = latest['pdo'][0]
                if (any(abs(v) > .005 for v in msg.speed[18:20]) or
                        baseline is None or any(abs(a-b) > .001 for a, b in zip(q, baseline))):
                    baseline, rest_since = q, time.monotonic()
                elif time.monotonic()-rest_since >= .2:
                    initial = q; boot = msg.boot_id; break
            if time.monotonic() >= deadline:
                raise RuntimeError('WAIST_TOPIC_DISCOVERY_OR_REST_TIMEOUT')
        waist_topic_targets(plan, 0., initial, profile['limits'], args.axis)
        record('plan', backend='waist_topic', command_topic=command_topic, axis=args.axis,
               initial=list(initial), amplitude_rad=plan.amplitude, duration=plan.duration,
               csp_encoder=encoder, controlled_joints=list(NAMES[:2]), **signal_metadata(plan))
        start = next_tick = time.monotonic(); rest_since = None
        while True:
            rclpy.spin_once(node, timeout_sec=.002)
            now = time.monotonic()
            q = check(boot=boot); elapsed = now-start
            if now-next_tick > .08:
                raise RuntimeError('TEST_STREAM_GAP')
            if now >= next_tick:
                goal = waist_topic_targets(plan, elapsed, initial, profile['limits'], args.axis)
                publisher.publish(Float64MultiArray(data=goal)); sent += 1
                record('waist_target', elapsed=elapsed, position=goal)
                next_tick = now+.02
            if elapsed >= plan.duration:
                resting = (all(abs(a-b) < execution['feedback_tolerances']['joint_position']
                               for a, b in zip(q, initial)) and
                           all(abs(v) <= execution['feedback_tolerances']['joint_velocity']
                               for v in latest['pdo'][0].speed[18:20]))
                rest_since = now if resting and rest_since is None else rest_since if resting else None
                if rest_since is not None and now-rest_since >= .1: break
                if elapsed-plan.duration >= execution['stop_timeout']:
                    raise RuntimeError('WAIST_TOPIC_REST_UNCONFIRMED')
    except (Exception, KeyboardInterrupt) as exc:
        failure = str(exc) or type(exc).__name__
    finally:
        # Legacy topic has no PAUSE/disable API. Last target remains held by servo;
        # after a fault we do not invent a safe return or claim a physical stop.
        record('result', completed=failure is None, error=failure, targets_sent=sent,
               driver_started=False, physical_stop_confirmed=False, hardware_acceptance=False)
        node.destroy_node(); rclpy.shutdown(); log.close()
    if failure:
        raise RuntimeError(failure)


def execute(args, plan):
    encoder = select_encoder(getattr(args, 'encoder_profile', None), getattr(args, 'position_release', None))
    # Imports below this boundary are deliberately absent from preview mode.
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from bindu_interfaces.msg import ExecutionState, MotionCommand, RuntimeEvent
    from bindu_interfaces.srv import Lease, SubmitMotion
    from snowbot_ctl_msgs.msg import PositionObservation
    from bindu_runtime.common import stamp
    from bindu_runtime.robot_setup import RobotSetup
    from tools.teleop.snowbot_pdo import observation_payload

    if not args.robot_config or not args.output:
        raise ValueError('--execute requires --robot-config and --output')
    axis = args.axis
    if axis not in NAMES[:2]:
        raise ValueError('TEST_AXIS_INVALID')
    robot = RobotSetup.load(args.robot_config)
    if tuple(n for g in GROUPS for n in robot.profile.groups.get(g, ())) != NAMES:
        raise ValueError('SNOWBOT_GROUP_LAYOUT')
    if (args.speed > robot.profile.speed(axis) or
            args.acceleration > robot.profile.acceleration(axis) or
            args.jerk > robot.profile.jerk(axis)):
        raise ValueError('TEST_EXCEEDS_SELECTED_PROFILE')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    log = args.output.open('x')
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('snowbot_waist_test_'+uuid.uuid4().hex[:8])
    ns = '/'+args.namespace.strip('/')
    topic = '/'+args.vendor_namespace.strip('/') if args.vendor_namespace.strip('/') else ''
    latest = {}; lease = None; failure = None; last_pdo = None; progress = time.monotonic()
    task = 'waist-test-'+uuid.uuid4().hex
    submitted = set()

    def record(kind, data):
        log.write(json.dumps(json_value(dict(kind=kind, steady=time.monotonic(), data=data)), allow_nan=False)+'\n')

    def state(msg):
        latest['state'] = (msg, time.monotonic())

    def pdo(msg):
        nonlocal last_pdo, progress
        now = time.monotonic()
        if last_pdo is not None and msg.pdo_sample < last_pdo:
            latest['error'] = 'PDO_SEQUENCE_REVERSED'
        if msg.pdo_sample != last_pdo:
            last_pdo, progress = msg.pdo_sample, now
        latest['pdo'] = (msg, now)
        if now-latest.get('logged', 0.) >= .01:
            payload = observation_payload(msg)
            payload.update(source_stamp_ns=msg.source_stamp_ns, expires_at_ns=msg.expires_at_ns)
            record('pdo', payload)
            latest['logged'] = now

    def event(msg):
        if msg.task_id == task or msg.command_id in submitted:
            record('event', dict(state=msg.state, code=msg.code, command_id=msg.command_id))
            if msg.state in ('TARGET_REJECTED', 'DEVICE_FEEDBACK_DETAIL', 'REJECTED', 'FAILED'):
                latest.setdefault('error', msg.code)

    node.create_subscription(ExecutionState, ns+'/execution/state', state, 10)
    node.create_subscription(PositionObservation, topic+'/bindu/position/observation', pdo, qos_profile_sensor_data)
    node.create_subscription(RuntimeEvent, ns+'/events', event, 100)
    leases = node.create_client(Lease, ns+'/execution/lease')
    submit = node.create_client(SubmitMotion, ns+'/execution/submit')

    def spin_until(predicate, timeout):
        end = time.monotonic()+timeout
        while not predicate():
            if time.monotonic() >= end:
                raise RuntimeError('TEST_WAIT_TIMEOUT')
            rclpy.spin_once(node, timeout_sec=.005)

    def call(client, request):
        if not client.service_is_ready():
            raise RuntimeError('EXECUTOR_SERVICE_UNAVAILABLE')
        future = client.call_async(request)
        spin_until(future.done, .25)
        reply = future.result()
        if not (reply.ok if hasattr(reply, 'ok') else reply.accepted):
            raise RuntimeError(latest.get('error', reply.code))
        return reply

    try:
        spin_until(lambda: 'state' in latest and 'pdo' in latest and leases.service_is_ready() and
                   submit.service_is_ready() and checked_pdo_source(node.get_publishers_info_by_topic(
                       topic+'/bindu/position/observation'), discovering=True), 5.)
        def check(owned=False, *, settling=False):
            now, steady = node.get_clock().now().nanoseconds*1e-9, time.monotonic()
            msg, received = latest['state']
            q, v = checked_state(msg, now, received, steady, robot.profile.digest,
                                 owned=owned, epoch=lease.epoch if lease else 0, stationary=not settling)
            obs, received = latest['pdo']
            checked_pdo(obs, now, received, steady, boot)
            if steady-progress > .05 or latest.get('error'):
                raise RuntimeError(latest.get('error', 'PDO_NOT_ADVANCING'))
            publishers = node.get_publishers_info_by_topic(topic+'/bindu/position/observation')
            checked_pdo_source(publishers)
            return q, v

        boot = latest['pdo'][0].boot_id
        initial, _ = check(settling=True)
        # Require advancing samples before acquiring any ownership.
        sample = latest['pdo'][0].pdo_sample
        spin_until(lambda: latest['pdo'][0].pdo_sample > sample, .1)
        # A READY/instantaneous low-speed sample can precede another settling
        # swing. Observe all owned joints before acquiring any lease or sending.
        deadline = time.monotonic()+5.
        rest_since = None
        rest_q = None
        while True:
            initial, velocity = check(settling=True)
            obs = latest['pdo'][0]
            worst = max(((abs(velocity[n]), n) for n in NAMES),
                        default=(0., ''))
            worst = max(worst, max((abs(obs.speed[slot]), n) for n, slot in zip(NAMES, SLOTS)))
            now = time.monotonic()
            if worst[0] > .005:
                rest_since = rest_q = None
            elif rest_since is None or any(abs(initial[n]-rest_q[n]) > .001 for n in NAMES):
                rest_since, rest_q = now, dict(initial)
            elif now-rest_since >= .2:
                record('settled', dict(window_s=now-rest_since, speed_limit=.005,
                                       position_range_limit=.001, max_speed=worst[0]))
                break
            if now >= deadline:
                raise RuntimeError('TEST_NOT_SETTLED:axis='+worst[1]+';speed='+str(worst[0]))
            rclpy.spin_once(node, timeout_sec=.005)
        for t in (0., plan.dwell+plan.move_seconds, plan.dwell+3*plan.move_seconds+2*plan.dwell):
            targets(plan, t, initial, robot.profile.limits, axis)
        record('plan', dict(task=task, robot_config=str(robot.path), profile_hash=robot.profile.digest,
                           joint_names=NAMES, axis=axis, initial=initial, amplitude_rad=plan.amplitude,
                           move_seconds=plan.move_seconds, duration=plan.duration,
                           csp_encoder=encoder, **signal_metadata(plan), **vars_limits(args)))
        lease = call(leases, Lease.Request(operation='acquire', owner=task, resources=list(GROUPS)))
        if lease.profile_hash != robot.profile.digest:
            raise RuntimeError('LEASE_PROFILE_MISMATCH')
        spin_until(lambda: latest['state'][0].lease_active, .25)
        start = next_tick = renewed = time.monotonic()
        while True:
            elapsed = time.monotonic()-start
            if elapsed >= plan.duration:
                break
            measured, _ = check(True)
            if any(abs(measured[n]-initial[n]) > .02 for n in NAMES if n != axis):
                raise RuntimeError('NON_TEST_JOINT_DRIFT')
            if abs(measured[axis]-initial[axis]) > plan.amplitude+.02:
                raise RuntimeError('WAIST_TEST_ENVELOPE')
            if elapsed > 1. and not (latest['pdo'][0].reference_valid and latest['pdo'][0].output_valid):
                raise RuntimeError('SERVO_REFERENCE_UNAVAILABLE')
            if time.monotonic()-renewed >= .5:
                call(leases, Lease.Request(operation='renew', lease_id=lease.lease_id, epoch=lease.epoch))
                renewed = time.monotonic()
            goal = targets(plan, elapsed, initial, robot.profile.limits, axis)
            msg = MotionCommand(schema_version=2, runtime_mode='real', command_id=uuid.uuid4().hex,
                task_id=task, observation_id=str(latest['pdo'][0].pdo_sample), profile_hash=robot.profile.digest,
                lease_id=lease.lease_id, epoch=lease.epoch, mode='joint_target',
                resource_groups=list(GROUPS), stamp=stamp(node.get_clock().now().nanoseconds*1e-9),
                valid_for=.2, joint_names=list(NAMES), positions=[goal[n] for n in NAMES])
            submitted.add(msg.command_id)
            call(submit, SubmitMotion.Request(command=msg))
            record('target', dict(elapsed=elapsed, joint=axis, position=goal[axis],
                                  **({'yaw': goal[axis]} if axis == 'body_joint_1' else {})))
            next_tick += .02
            if time.monotonic()-next_tick > .08:
                raise RuntimeError('TEST_STREAM_GAP')
            spin_until(lambda: time.monotonic() >= next_tick, .1)
    except (Exception, KeyboardInterrupt) as exc:
        failure = str(exc) or type(exc).__name__
    finally:
        # Release uses the existing PAUSE/stop confirmation. Never stop the driver
        # or fall back to an unguarded waist topic after a failure.
        if lease:
            try:
                call(leases, Lease.Request(operation='release', lease_id=lease.lease_id, epoch=lease.epoch))
                released = time.monotonic()
                spin_until(lambda: latest['state'][1] > released and
                    not latest['state'][0].lease_active and latest['state'][0].state != 'STOPPING', 5.)
                if latest['state'][0].stop_failures or latest['state'][0].state == 'FAILED':
                    raise RuntimeError('STOP_UNCONFIRMED')
                q, v = dict(zip(latest['state'][0].joints.name, latest['state'][0].joints.position)), dict(zip(
                    latest['state'][0].joints.name, latest['state'][0].joints.velocity))
                if any(n not in q or n not in v or not math.isfinite(v[n]) or abs(v[n]) > .01 for n in NAMES):
                    raise RuntimeError('STOP_MEASUREMENT_UNAVAILABLE_OR_MOVING')
            except (Exception, KeyboardInterrupt) as exc:
                failure = (failure+'; ' if failure else '')+'STOP_UNCONFIRMED: '+str(exc)
        record('result', dict(completed=failure is None, error=failure,
                             physical_acceptance=False, driver_started=False))
        node.destroy_node(); rclpy.shutdown(); log.close()
    if failure:
        raise RuntimeError(failure)


def vars_limits(args):
    return dict(requested_speed=args.speed, requested_acceleration=args.acceleration, requested_jerk=args.jerk)


def summarize_csp(rows, slot, encoder=None):
    """Raw attempt evidence, including failed receives; never infer physical cause."""
    index = slot-16
    missing = malformed = unsupported = repeats = command_invalid = feedback_invalid = 0
    seen = set(); modes = Counter(); pairs = Counter(); errors = []; encoding = []
    sent = []; measured = []; mismatch = not_csp = 0
    arrays = {'csp_controlword': (0, 65535), 'csp_mode_command': (-128, 127),
              'csp_mode_display': (-128, 127), 'csp_target_position_raw': (-2**31, 2**31-1),
              'csp_target_velocity_raw': (-2**31, 2**31-1), 'csp_target_torque_raw': (-32768, 32767),
              'csp_actual_position_raw': (-2**31, 2**31-1), 'csp_actual_velocity_raw': (-2**31, 2**31-1)}
    valid_frames = 0
    for data in rows:
        if 'csp_diagnostics_version' not in data:
            missing += 1
            continue
        if type(data['csp_diagnostics_version']) is not int or data['csp_diagnostics_version'] != 1:
            unsupported += 1
            continue
        if (any(not isinstance(data.get(k), (list, tuple)) or len(data[k]) != 4 or
                any(type(v) is not int or not lo <= v <= hi for v in data[k])
                for k, (lo, hi) in arrays.items()) or
                any(type(data.get(k)) is not int or not 0 <= data[k] <= 0xfffff
                    for k in ('csp_command_mask', 'csp_feedback_mask')) or
                any(type(data.get(k)) is not int or data[k] <= 0
                    for k in ('boot_id', 'receive_attempt', 'csp_send_attempt_ns'))):
            malformed += 1
            continue
        identity = data['boot_id'], data['receive_attempt']
        if identity in seen:
            repeats += 1
            continue
        seen.add(identity); valid_frames += 1
        command_valid = (bool(data['csp_command_mask'] & (1 << slot)) and
                         type(data.get('attempt_age')) in (int, float) and 0 <= data['attempt_age'] <= .02)
        feedback_valid = (bool(data['csp_feedback_mask'] & data.get('valid_mask', 0) & (1 << slot)) and
                          data.get('expected_wkc', 0) > 0 and data.get('wkc') == data['expected_wkc'] and
                          type(data.get('pdo_age')) in (int, float) and 0 <= data['pdo_age'] <= .02)
        if command_valid:
            sent.append(data['csp_target_position_raw'][index])
            not_csp += data['csp_mode_command'][index] != 8
            output = data.get('output_position', [])
            if (data.get('output_valid') and isinstance(output, (list, tuple)) and len(output) == 16 and
                    type(output[slot-4]) in (int, float) and math.isfinite(output[slot-4])):
                if encoder is not None:
                    encoding.append(data['csp_target_position_raw'][index]-
                                    output[slot-4]*encoder['position_counts_per_rev']/(2*math.pi))
        else:
            command_invalid += 1
        if feedback_valid:
            measured.append(data['csp_actual_position_raw'][index])
            modes[str(data['csp_mode_display'][index])] += 1
        else:
            feedback_invalid += 1
        if command_valid and feedback_valid:
            pair = data['csp_mode_command'][index], data['csp_mode_display'][index]
            pairs[f'{pair[0]}/{pair[1]}'] += 1
            mismatch += pair[0] != pair[1]
            errors.append(data['csp_target_position_raw'][index]-data['csp_actual_position_raw'][index])
    return dict(available=valid_frames > 0, slot=slot, array_index=index, unique_attempts=valid_frames,
        missing_extension_frames=missing, unsupported_version_frames=unsupported, malformed_frames=malformed,
        repeated_attempts_ignored=repeats, invalid_command_attempts=command_invalid,
        invalid_feedback_attempts=feedback_invalid, actual_mode_counts=dict(modes),
        command_actual_mode_counts=dict(pairs), mode_mismatch_attempts=mismatch, command_not_csp_attempts=not_csp,
        sent_position_raw_range=[min(sent), max(sent)] if sent else None,
        actual_position_raw_range=[min(measured), max(measured)] if measured else None,
        following_error_raw_peak=max(map(abs, errors), default=None),
        following_error_raw_rms=math.sqrt(sum(x*x for x in errors)/len(errors)) if errors else None,
        sent_vs_output_peak_counts=max(map(abs, encoding), default=None),
        output_conversion_counts_per_code_rad=(encoder['position_counts_per_rev']/(2*math.pi)
                                               if encoder is not None else None),
        speed_conversion_counts_per_code_rad=(encoder['speed_counts_per_rev']/(2*math.pi)
                                              if encoder is not None else None),
        encoder_scale_available=encoder is not None, encoder_selection=encoder,
        encoding_comparison_reason=('CSP_ENCODER_SCALE_UNSPECIFIED' if encoder is None else
                                    'OK' if encoding else 'NO_VALID_SENT_OUTPUT_PAIRS'),
        command_is_drive_acknowledgement=False, physical_units_verified=False, cause_identified=False)


def analyze(path, session=None, *, encoder_profile=None, position_release=None):
    """Compare same-PDO reference/output/measurement; estimate speed from position."""
    samples = []; result = None; repeats = 0; axis = 'body_joint_1'
    pdo_rows = []; sessions = set(); observed_sessions = set(); invalid = Counter(); other_sessions = 0
    selection = 'explicit' if session is not None else 'unavailable'
    declarations = []
    with Path(path).open() as source:
        # PDO callbacks can write before plan metadata; select the axis first.
        for line in source:
            row = json.loads(line)
            if row['kind'] in ('plan', 'metadata') and row['data'].get('csp_encoder') is not None:
                declared_session = row['data'].get('session')
                if declared_session is not None and (type(declared_session) is not int or declared_session <= 0):
                    raise ValueError('CAPTURE_ENCODER_SESSION_INVALID')
                declarations.append((declared_session, row['data']['csp_encoder'],
                                     dict(kind='capture_metadata', record_kind=row['kind'],
                                          provenance=row['data']['csp_encoder'].get('sources', [])
                                          if isinstance(row['data']['csp_encoder'], dict) else [])))
            if row['kind'] == 'plan':
                axis = row['data'].get('axis', 'body_joint_1')
                if axis not in NAMES[:2]:
                    raise ValueError('TEST_AXIS_INVALID')
            if row['kind'] == 'event' and row['data'].get('state') == 'TARGET_SENT':
                code = row['data'].get('code', '')
                if code.startswith('SNOWBOT_TARGET:'):
                    sessions.add(json.loads(code.split(':', 1)[1])['session'])
            if row['kind'] == 'pdo':
                observed = row['data'].get('session')
                if type(observed) is int and observed > 0:
                    observed_sessions.add(observed)
        if session is None:
            candidates = sessions or observed_sessions
            if len(candidates) > 1:
                raise ValueError('CAPTURE_SESSION_AMBIGUOUS: select --session')
            session = next(iter(candidates), None)
            selection = 'target_sent' if sessions else 'observed_only_unmapped' if candidates else 'unavailable'
        encoder = select_encoder(encoder_profile, position_release,
                                 [(value, source) for declared_session, value, source in declarations
                                  if declared_session is None or declared_session == session])
        source.seek(0)
        for line in source:
            row = json.loads(line)
            if row['kind'] == 'result':
                result = row['data']
            if row['kind'] != 'pdo':
                continue
            data = row['data']
            if session is not None and data.get('session') != session:
                other_sessions += 1
                continue
            pdo_rows.append(data)
            reasons = []
            if not data.get('reference_valid'): reasons.append('reference_invalid')
            if not data.get('output_valid'): reasons.append('output_invalid')
            if data.get('expected_wkc', 0) <= 0 or data.get('wkc') != data['expected_wkc']: reasons.append('bad_wkc')
            if type(data.get('pdo_age')) not in (int, float) or not 0 <= data['pdo_age'] <= .02: reasons.append('pdo_stale')
            if data.get('valid_mask', 0) & MASK != MASK: reasons.append('incomplete_feedback')
            if any(data['status'][slot] & 8 for slot in SLOTS): reasons.append('drive_fault')
            invalid.update(reasons)
            if not (data.get('reference_valid') and data.get('output_valid') and
                    data.get('expected_wkc', 0) > 0 and data.get('wkc') == data['expected_wkc'] and
                    type(data.get('pdo_age')) in (int, float) and 0 <= data['pdo_age'] <= .02 and
                    data.get('valid_mask', 0) & MASK == MASK and
                    not any(data['status'][slot] & 8 for slot in SLOTS)):
                continue
            slot, ref_slot = (18, 14) if axis == 'body_joint_1' else (19, 15)
            values = (data['stamp']-data['pdo_age'], data['position'][slot], data['speed'][slot],
                      data['reference_position'][ref_slot], data['reference_speed'][ref_slot], data['output_position'][ref_slot])
            if not all(type(v) in (int, float) and math.isfinite(v) for v in values):
                invalid['nonfinite_reference_or_measurement'] += 1
                continue
            if samples and data['boot_id'] == samples[-1][0] and data['pdo_sample'] == samples[-1][1]:
                repeats += 1
                continue
            if samples and (data['boot_id'] != samples[-1][0] or data['pdo_sample'] < samples[-1][1] or
                            values[0] <= samples[-1][2]):
                raise ValueError('CAPTURE_BOOT_OR_SAMPLE_ORDER_CHANGED')
            samples.append((data['boot_id'], data['pdo_sample'], *values))
    csp = summarize_csp(pdo_rows, 18 if axis == 'body_joint_1' else 19, encoder)
    diagnostics = dict(session=session, session_selection=selection, pdo_frames=len(pdo_rows), other_session_frames_ignored=other_sessions,
                       invalid_frame_reasons=dict(invalid), csp_diagnostics=csp)
    if not samples:
        return dict(axis=axis, samples=0, reference_analysis_available=False,
                    reason='NO_VALID_SERVO_REFERENCE_SAMPLES', run_result=result,
                    physical_units_verified=False, hardware_acceptance=False, **diagnostics)
    speeds = []; ratios = []; previous = 0
    for index, sample in enumerate(samples):
        while previous+1 < index and sample[2]-samples[previous+1][2] >= .05:
            previous += 1
        dt = sample[2]-samples[previous][2]
        if .05 <= dt <= .1:
            estimated = (sample[3]-samples[previous][3])/dt
            speeds.append(estimated)
            if abs(estimated) >= .005:
                reported = (sample[4]+samples[previous][4])/2
                ratios.append(reported/estimated)
    return dict(axis=axis, physical_units_verified=False, reference_analysis_available=True, **diagnostics,
        samples=len(samples), repeated_samples_ignored=repeats, run_result=result,
        reference_speed_peak_rad_s=max(abs(s[6]) for s in samples),
        reported_speed_peak_rad_s=max(abs(s[4]) for s in samples),
        position_difference_speed_peak_rad_s=max(map(abs, speeds), default=None),
        reported_to_position_difference_ratio_median=statistics.median(ratios) if ratios else None,
        reference_measurement_rms_rad=math.sqrt(sum((s[5]-s[3])**2 for s in samples)/len(samples)),
        output_measurement_rms_rad=math.sqrt(sum((s[7]-s[3])**2 for s in samples)/len(samples)),
        speed_time_basis='header.stamp - pdo_age; 50-100 ms position difference; diagnostic only',
        hardware_acceptance=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--axis', choices=(*NAMES[:2], 'both'), default='body_joint_1',
                        help='both is available only with --backend waist_topic')
    parser.add_argument('--degrees', type=float, default=1., help='code-scale excursion each way; default 1 deg, <=20')
    parser.add_argument('--speed', type=float, default=.1, help='code-scale rad/s; default .1, <=0.375 and selected profile')
    parser.add_argument('--acceleration', type=float, default=.5, help='code-scale rad/s^2; default .5, <=2.25 and selected profile')
    parser.add_argument('--jerk', type=float, default=10., help='code-scale rad/s^3; default 10, <=162')
    parser.add_argument('--backend', choices=('session', 'waist_topic'), default='session')
    parser.add_argument('--waveform', choices=('quintic', 'sine'), default='quintic')
    parser.add_argument('--frequency', type=float, default=.2, help='sine frequency in Hz')
    parser.add_argument('--cycles', type=int, default=5, help='sine cycles including first/last envelope cycles')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--execute', action='store_true', help='operator only: acquire 16-axis lease and send targets')
    mode.add_argument('--analyze', type=Path, help='offline analysis of an existing capture')
    parser.add_argument('--session', type=int, help='select one servo session for offline analysis')
    parser.add_argument('--encoder-profile', choices=tuple(ENCODER_PROFILES),
                        help='explicit log/code encoder scale; old logs without metadata remain unspecified')
    parser.add_argument('--position-release', type=Path,
                        help='selected position_release.json with csp_encoder metadata; never selects it from current HEAD')
    parser.add_argument('--robot-config', type=Path)
    parser.add_argument('--namespace', default='bindu_snowbot')
    parser.add_argument('--vendor-namespace', default='/')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.analyze:
        print(json.dumps(analyze(args.analyze, args.session, encoder_profile=args.encoder_profile,
                                 position_release=args.position_release), indent=2, allow_nan=False))
        return
    if args.session is not None:
        parser.error('--session requires --analyze')
    if args.axis == 'both' and args.backend != 'waist_topic':
        parser.error('--axis both requires --backend waist_topic')
    plan = (make_sine_plan(args.degrees, args.frequency, args.cycles, args.speed, args.acceleration, args.jerk)
            if args.waveform == 'sine' else make_plan(args.degrees, args.speed, args.acceleration, args.jerk))
    encoder = select_encoder(args.encoder_profile, args.position_release)
    print(json.dumps(dict(preview=not args.execute, axis=args.axis, physical_units_verified=False, degrees=args.degrees,
        duration_s=plan.duration, move_seconds=plan.move_seconds,
        target_peak_speed=None if isinstance(plan, SinePlan) else 1.875*plan.amplitude/plan.move_seconds,
        target_peak_acceleration=None if isinstance(plan, SinePlan) else 10/math.sqrt(3)*plan.amplitude/plan.move_seconds**2,
        target_peak_jerk=None if isinstance(plan, SinePlan) else 60*plan.amplitude/plan.move_seconds**3,
        backend=args.backend, controlled_groups=('waist',) if args.backend == 'waist_topic' else GROUPS,
        holds_other_joint_targets=args.backend == 'session',
        starts_driver=False, csp_encoder=encoder, **signal_metadata(plan), **vars_limits(args)), indent=2))
    if args.execute:
        (execute_waist_topic if args.backend == 'waist_topic' else execute)(args, plan)


if __name__ == '__main__':
    main()
