#!/usr/bin/env python3
"""Single waist-axis diagnostic. Preview is offline; --execute uses an existing executor.

The operator starts the reviewed position stack separately. This program never
starts/stops drivers, changes limits, resets faults or writes the legacy body
topic. The current Snowbot session owns all 16 waist/arm joints: arm joint
targets and the other waist axis stay at the captured measurement, not Cartesian poses.
Use without a tool: moving the waist moves both arms in the world frame.
"""
import argparse
from dataclasses import dataclass
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


def make_plan(degrees=20., speed=.25, acceleration=.5, jerk=10.):
    for value, maximum in ((degrees, 20.), (speed, .375), (acceleration, 2.25), (jerk, 162.)):
        if type(value) not in (float, int) or not math.isfinite(value) or not 0 < value <= maximum:
            raise ValueError('TEST_ENVELOPE_EXCEEDED')
    distance = math.radians(degrees)
    duration = max(1.875*distance/speed, math.sqrt(10/math.sqrt(3)*distance/acceleration),
                   (60*distance/jerk)**(1/3), .2)
    return Plan(distance, math.ceil(duration/.02)*.02)


def checked_state(msg, now, received, steady, profile_hash, *, owned=False, epoch=0):
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
    if not owned and any(abs(v[n]) > .01 for n in NAMES):
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
    if not publishers and discovering:
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
    offset, _, _ = plan.sample(elapsed)
    q = {n: initial[n] for n in NAMES}
    q[axis] += offset
    for n in NAMES:
        # Extra excursion clearance applies only to the axis being moved.
        margin = .01 if n == axis else 0.
        if not math.isfinite(q[n]) or not limits[n][0]+margin <= q[n] <= limits[n][1]-margin:
            raise ValueError('TEST_POSITION_LIMIT:'+n)
    return q


def execute(args, plan):
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
        if msg.task_id == task and msg.state in ('REJECTED', 'FAILED'):
            latest['error'] = msg.code

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
            raise RuntimeError(reply.code)
        return reply

    try:
        spin_until(lambda: 'state' in latest and 'pdo' in latest and leases.service_is_ready() and
                   submit.service_is_ready() and checked_pdo_source(node.get_publishers_info_by_topic(
                       topic+'/bindu/position/observation'), discovering=True), 5.)
        def check(owned=False):
            now, steady = node.get_clock().now().nanoseconds*1e-9, time.monotonic()
            msg, received = latest['state']
            q, v = checked_state(msg, now, received, steady, robot.profile.digest,
                                 owned=owned, epoch=lease.epoch if lease else 0)
            obs, received = latest['pdo']
            checked_pdo(obs, now, received, steady, boot)
            if steady-progress > .05 or latest.get('error'):
                raise RuntimeError(latest.get('error', 'PDO_NOT_ADVANCING'))
            publishers = node.get_publishers_info_by_topic(topic+'/bindu/position/observation')
            checked_pdo_source(publishers)
            return q, v

        boot = latest['pdo'][0].boot_id
        initial, _ = check()
        # Require advancing samples before acquiring any ownership.
        sample = latest['pdo'][0].pdo_sample
        spin_until(lambda: latest['pdo'][0].pdo_sample > sample, .1)
        initial, _ = check()
        for t in (0., plan.dwell+plan.move_seconds, plan.dwell+3*plan.move_seconds+2*plan.dwell):
            targets(plan, t, initial, robot.profile.limits, axis)
        record('plan', dict(task=task, robot_config=str(robot.path), profile_hash=robot.profile.digest,
                           joint_names=NAMES, axis=axis, initial=initial, amplitude_rad=plan.amplitude,
                           move_seconds=plan.move_seconds, duration=plan.duration, **vars_limits(args)))
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


def analyze(path):
    """Compare same-PDO reference/output/measurement; estimate speed from position."""
    samples = []; result = None; repeats = 0; axis = 'body_joint_1'
    with Path(path).open() as source:
        # PDO callbacks can write before plan metadata; select the axis first.
        for line in source:
            row = json.loads(line)
            if row['kind'] == 'plan':
                axis = row['data'].get('axis', 'body_joint_1')
                if axis not in NAMES[:2]:
                    raise ValueError('TEST_AXIS_INVALID')
                break
        source.seek(0)
        for line in source:
            row = json.loads(line)
            if row['kind'] == 'result':
                result = row['data']
            if row['kind'] != 'pdo':
                continue
            data = row['data']
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
                continue
            if samples and data['boot_id'] == samples[-1][0] and data['pdo_sample'] == samples[-1][1]:
                repeats += 1
                continue
            if samples and (data['boot_id'] != samples[-1][0] or data['pdo_sample'] < samples[-1][1] or
                            values[0] <= samples[-1][2]):
                raise ValueError('CAPTURE_BOOT_OR_SAMPLE_ORDER_CHANGED')
            samples.append((data['boot_id'], data['pdo_sample'], *values))
    if not samples:
        raise ValueError('NO_VALID_SERVO_REFERENCE_SAMPLES')
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
    return dict(axis=axis, physical_units_verified=False,
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
    parser.add_argument('--axis', choices=NAMES[:2], default='body_joint_1', help='only this waist target changes')
    parser.add_argument('--degrees', type=float, default=20., help='excursion each way, <=20 deg')
    parser.add_argument('--speed', type=float, default=.25, help='rad/s, <=0.375 and selected profile')
    parser.add_argument('--acceleration', type=float, default=.5, help='rad/s^2, <=2.25 and selected profile')
    parser.add_argument('--jerk', type=float, default=10., help='rad/s^3, <=162')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--execute', action='store_true', help='operator only: acquire 16-axis lease and send targets')
    mode.add_argument('--analyze', type=Path, help='offline analysis of an existing capture')
    parser.add_argument('--robot-config', type=Path)
    parser.add_argument('--namespace', default='bindu_snowbot')
    parser.add_argument('--vendor-namespace', default='/')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.analyze:
        print(json.dumps(analyze(args.analyze), indent=2))
        return
    plan = make_plan(args.degrees, args.speed, args.acceleration, args.jerk)
    print(json.dumps(dict(preview=not args.execute, axis=args.axis, physical_units_verified=False, degrees=args.degrees,
        duration_s=plan.duration, move_seconds=plan.move_seconds,
        target_peak_speed=1.875*plan.amplitude/plan.move_seconds,
        target_peak_acceleration=10/math.sqrt(3)*plan.amplitude/plan.move_seconds**2,
        target_peak_jerk=60*plan.amplitude/plan.move_seconds**3,
        controlled_groups=GROUPS, holds_other_joint_targets=True,
        starts_driver=False, **vars_limits(args)), indent=2))
    if args.execute:
        execute(args, plan)


if __name__ == '__main__':
    main()
