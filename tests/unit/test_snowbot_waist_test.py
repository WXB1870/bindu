"""Offline signal/envelope/feedback tests; never import ROS or open device IO."""
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
from types import ModuleType, SimpleNamespace as S
import unittest
from unittest.mock import patch

from tools.teleop import snowbot_waist_test as waist
from tools.teleop.snowbot_waist_test import (
    MASK, NAMES, analyze, checked_pdo, checked_state, json_value, make_plan, targets,
)


def state():
    return S(runtime_mode='real', simulated=False, device_backend='snowbot_vendor',
        profile_hash='hash', stop_failures=[], feedback_stamp=S(sec=10, nanosec=0),
        state='IDLE', code='OK', lease_active=False, lease_epoch=0, lease_resources=[],
        joints=S(name=list(NAMES), position=[0.]*16, velocity=[0.]*16))


def pdo():
    return S(boot_id=1, pdo_sample=2, header=S(stamp=S(sec=10, nanosec=0)), pdo_age=.001,
        expected_wkc=67, wkc=67, valid_mask=MASK, status=[0x27]*20,
        position=[0.]*20, speed=[0.]*20)


class WaistTests(unittest.TestCase):
    def test_signal_limits_and_rest_at_every_boundary(self):
        for degrees, speed in ((3., .05), (3., .1), (3., .25), (20., .25)):
            plan = make_plan(degrees, speed, .5, 162.)
            previous = plan.sample(0.)
            step = .001
            for index in range(1, math.ceil(plan.duration/step)+1):
                q, v, a = current = plan.sample(index*step)
                self.assertLessEqual(abs(q), plan.amplitude+1e-10)
                self.assertLessEqual(abs(v), speed+1e-10)
                self.assertLessEqual(abs(a), .5+1e-10)
                self.assertLessEqual(abs(a-previous[2])/step, 162.+1e-8)
                self.assertLessEqual(abs(q-previous[0])/step, speed+1e-6)
                previous = current
            self.assertEqual(plan.sample(plan.duration), (0., 0., 0.))
            for k, expected in enumerate((plan.amplitude, 0., -plan.amplitude, 0.)):
                t = plan.dwell+k*(plan.move_seconds+plan.dwell)+plan.move_seconds
                q, v, a = plan.sample(t)
                self.assertAlmostEqual(q, expected)
                self.assertAlmostEqual(v, 0.)
                self.assertAlmostEqual(a, 0.)

    def test_invalid_or_unreviewed_envelope_refused(self):
        for values in ((0.,), (float('nan'),), (21.,), (3., .376), (3., .05, 2.251), (3., .05, .25, 163.)):
            with self.subTest(values=values), self.assertRaises(ValueError):
                make_plan(*values)

    def test_default_twenty_degree_excursion_reaches_requested_speed(self):
        plan = make_plan()
        self.assertAlmostEqual(math.degrees(plan.amplitude), 20.)
        peak = plan.sample(plan.dwell+plan.move_seconds/2)
        self.assertGreater(peak[1], .249)
        self.assertLessEqual(peak[1], .25)
        self.assertAlmostEqual(plan.duration, 15.48)

    def test_only_yaw_target_changes_and_limits_keep_margin(self):
        plan = make_plan()
        initial = {n: .1+index*.01 for index, n in enumerate(NAMES)}
        limits = {n: (-2., 2.) for n in NAMES}
        output = targets(plan, plan.dwell+plan.move_seconds, initial, limits)
        self.assertAlmostEqual(output['body_joint_1'], initial['body_joint_1']+plan.amplitude)
        self.assertTrue(all(output[n] == initial[n] for n in NAMES if n != 'body_joint_1'))
        limits['body_joint_1'] = (-2., output['body_joint_1']+.005)
        with self.assertRaises(ValueError):
            targets(plan, plan.dwell+plan.move_seconds, initial, limits)

    def test_faster_signal_and_pitch_selection_keep_other_targets(self):
        plan = make_plan(20., .375, 1., 10.)
        self.assertLess(plan.duration, make_plan().duration)
        self.assertLessEqual(1.875*plan.amplitude/plan.move_seconds, .375)
        self.assertGreater(1.875*plan.amplitude/plan.move_seconds, .37)
        initial = {n: 0. for n in NAMES}
        limits = {n: (-2., 2.) for n in NAMES}
        output = targets(plan, plan.dwell+plan.move_seconds, initial, limits, 'body_joint_2')
        self.assertAlmostEqual(output['body_joint_2'], plan.amplitude)
        self.assertTrue(all(output[n] == initial[n] for n in NAMES if n != 'body_joint_2'))
        limits['body_joint_2'] = (-2., output['body_joint_2']+.005)
        with self.assertRaisesRegex(ValueError, 'TEST_POSITION_LIMIT:body_joint_2'):
            targets(plan, plan.dwell+plan.move_seconds, initial, limits, 'body_joint_2')
        with self.assertRaisesRegex(ValueError, 'TEST_AXIS_INVALID'):
            targets(plan, 0., initial, limits, 'arm_l_joint_1')

    def test_held_arms_near_limits_do_not_require_excursion_margin(self):
        plan = make_plan()
        initial = {n: 0. for n in NAMES}
        limits = {n: (-2., 2.) for n in NAMES}
        initial['arm_l_joint_2'] = -.00026365293888375163
        initial['arm_r_joint_2'] = .0003834951785393059
        limits['arm_l_joint_2'] = (-.0017453292519943296, math.pi)
        limits['arm_r_joint_2'] = (-math.pi, .0017453292519943296)
        for offset in (-plan.amplitude, 0., plan.amplitude):
            t = (plan.dwell+3*plan.move_seconds+2*plan.dwell if offset < 0 else
                 plan.dwell+plan.move_seconds if offset > 0 else 0.)
            output = targets(plan, t, initial, limits)
            self.assertAlmostEqual(output['body_joint_1'], offset)
            self.assertTrue(all(output[n] == initial[n] for n in NAMES if n != 'body_joint_1'))
        initial['arm_r_joint_2'] = limits['arm_r_joint_2'][1]+.0001
        with self.assertRaisesRegex(ValueError, 'TEST_POSITION_LIMIT:arm_r_joint_2'):
            targets(plan, 0., initial, limits)

    def test_stale_simulated_busy_or_missing_measurements_refused(self):
        checked_state(state(), 10.01, 2., 2.01, 'hash')
        stopped = state(); stopped.state = stopped.code = 'CANCELED'
        checked_state(stopped, 10.01, 2., 2.01, 'hash')
        for change in (dict(simulated=True), dict(lease_active=True), dict(profile_hash='other'),
                       dict(state='FAILED'), dict(stop_failures=['waist'])):
            msg = state()
            for name, value in change.items():
                setattr(msg, name, value)
            with self.subTest(change=change), self.assertRaises(RuntimeError):
                checked_state(msg, 10.01, 2., 2.01, 'hash')
        with self.assertRaises(RuntimeError):
            checked_state(state(), 10.3, 2., 2.01, 'hash')
        msg = state(); msg.joints.velocity = []
        with self.assertRaises(RuntimeError):
            checked_state(msg, 10.01, 2., 2.01, 'hash')

    def test_fault_bad_wkc_missing_waist_and_restart_refused(self):
        checked_pdo(pdo(), 10.01, 2., 2.01, 1)
        for change in (dict(boot_id=2), dict(pdo_age=.03), dict(wkc=66),
                       dict(valid_mask=MASK & ~(1<<18)), dict(status=[8]*20)):
            msg = pdo()
            for name, value in change.items():
                setattr(msg, name, value)
            with self.subTest(change=change), self.assertRaises(RuntimeError):
                checked_pdo(msg, 10.01, 2., 2.01, 1)

    def test_offline_analysis_exposes_speed_scale_disagreement(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'capture.jsonl'; rows = []
            for index in range(30):
                t = 10.+index*.01
                position = [0.]*20; position[18] = .05*(t-10.)
                speed = [0.]*20; speed[18] = 1.
                reference = [0.]*16; reference[14] = position[18]+.001
                velocity = [0.]*16; velocity[14] = .05
                rows.append(dict(kind='pdo', data=dict(boot_id=1, pdo_sample=index+1, stamp=t,
                    pdo_age=0., reference_valid=True, output_valid=True, wkc=67, expected_wkc=67,
                    valid_mask=MASK, status=[0x27]*20, position=position, speed=speed,
                    reference_position=reference, reference_speed=velocity, output_position=reference)))
            path.write_text(''.join(json.dumps(row)+'\n' for row in rows))
            result = analyze(path)
            self.assertAlmostEqual(result['position_difference_speed_peak_rad_s'], .05)
            self.assertAlmostEqual(result['reported_to_position_difference_ratio_median'], 20.)
            self.assertAlmostEqual(result['reference_measurement_rms_rad'], .001)
            self.assertFalse(result['hardware_acceptance'])
            rows[-1]['data']['pdo_sample'] = 1
            path.write_text(''.join(json.dumps(row)+'\n' for row in rows))
            with self.assertRaises(ValueError):
                analyze(path)

    def test_preview_has_no_ros_dependency_or_output_file(self):
        result = subprocess.run([sys.executable, '-m', 'tools.teleop.snowbot_waist_test'],
                                capture_output=True, text=True, check=True)
        result = json.loads(result.stdout)
        self.assertTrue(result['preview'])
        self.assertFalse(result['physical_units_verified'])
        self.assertFalse(result['starts_driver'])
        self.assertEqual(result['axis'], 'body_joint_1')
        self.assertEqual(result['degrees'], 20.)
        self.assertEqual(result['requested_speed'], .25)
        self.assertEqual(result['requested_acceleration'], .5)
        self.assertGreater(result['target_peak_speed'], .249)
        self.assertEqual(json_value([float('nan'), float('-inf')]), [None, None])

    def test_pitch_analysis_selects_pitch_even_when_pdo_precedes_plan(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'pitch.jsonl'
            rows = []
            for i in range(20):
                t = 10.+i*.01
                q = [0.]*20; q[19] = .05*(t-10.)
                v = [0.]*20; v[19] = 1.
                ref = [0.]*16; ref[15] = q[19]+.001
                rv = [0.]*16; rv[15] = .05
                rows.append(dict(kind='pdo', data=dict(boot_id=1, pdo_sample=i+1,
                    stamp=t, pdo_age=0., reference_valid=True, output_valid=True,
                    wkc=67, expected_wkc=67, valid_mask=MASK, status=[0x27]*20,
                    position=q, speed=v, reference_position=ref,
                    reference_speed=rv, output_position=ref)))
            rows.insert(1, dict(kind='plan', data=dict(axis='body_joint_2')))
            path.write_text(''.join(json.dumps(r)+'\n' for r in rows))
            result = analyze(path)
            self.assertEqual(result['axis'], 'body_joint_2')
            self.assertAlmostEqual(result['position_difference_speed_peak_rad_s'], .05)
            self.assertAlmostEqual(result['reference_measurement_rms_rad'], .001)

    def test_actual_runner_with_memory_ros_holds_arms_and_releases_on_fault(self):
        # Execute the real runner with in-memory ROS objects and a virtual clock.
        # No launch, socket, ROS runtime or hardware process exists in this test.
        for axis, inject_fault in ((n, f) for n in NAMES[:2] for f in (False, True)):
            clock = S(value=10.)
            requests = []; commands = []; subscriptions = {}
            current = {n: .1 for n in NAMES}; owned = S(value=False)

            def msg(**kwargs):
                return S(**kwargs)

            class Client:
                def __init__(self, service):
                    self.service = service

                def service_is_ready(self):
                    return True

                def call_async(self, request):
                    if self.service.endswith('/lease'):
                        requests.append(request.operation)
                        if request.operation == 'acquire':
                            owned.value = True
                        if request.operation == 'release':
                            owned.value = False
                        response = S(ok=True, lease_id='lease', epoch=1, profile_hash='hash')
                    else:
                        commands.append(request.command)
                        current.update(zip(request.command.joint_names, request.command.positions))
                        response = S(accepted=True)
                    return S(done=lambda: True, result=lambda: response)

            class Node:
                def create_subscription(self, kind, topic, callback, qos):
                    subscriptions[topic] = callback

                def create_client(self, kind, topic):
                    return Client(topic)

                def get_clock(self):
                    return S(now=lambda: S(nanoseconds=int(clock.value*1e9)))

                def get_publishers_info_by_topic(self, topic):
                    return [S(node_name='servo_node')]

                def destroy_node(self):
                    pass

            def stamp(value):
                ns = int(value*1e9)
                return S(sec=ns//10**9, nanosec=ns % 10**9)

            def spin_once(node, timeout_sec):
                clock.value += .005
                s = state(); s.feedback_stamp = s.stamp = stamp(clock.value)
                s.lease_active = owned.value; s.lease_epoch = 1; s.lease_resources = list(waist.GROUPS)
                s.joints.position = [current[n] for n in NAMES]
                p = pdo(); p.header.stamp = stamp(clock.value); p.pdo_sample = round(clock.value*1000)
                p.reference_valid = p.output_valid = True
                p.source_stamp_ns = p.expires_at_ns = 0
                p.reference_position = p.output_position = [current[n] for n in (*NAMES[2:], *NAMES[:2])]
                p.reference_speed = [0.]*16
                for n, slot in zip(NAMES, waist.SLOTS):
                    p.position[slot] = current[n]
                if inject_fault and len(commands) >= 3 and owned.value:
                    p.status[18 if axis == 'body_joint_1' else 19] |= 8
                subscriptions['/bindu_snowbot/execution/state'](s)
                subscriptions['/bindu/position/observation'](p)

            modules = {}
            def module(name, **attrs):
                value = ModuleType(name)
                value.__dict__.update(attrs); modules[name] = value
            module('rclpy', init=lambda **kwargs: None, shutdown=lambda: None,
                   create_node=lambda name: Node(), spin_once=spin_once)
            module('rclpy.qos', qos_profile_sensor_data=object())
            module('rclpy.signals', SignalHandlerOptions=S(NO=0))
            module('bindu_interfaces.msg', ExecutionState=S, MotionCommand=msg, RuntimeEvent=S)
            module('bindu_interfaces.srv', Lease=S(Request=msg), SubmitMotion=S(Request=msg))
            module('snowbot_ctl_msgs.msg', PositionObservation=S)
            module('bindu_runtime.common', stamp=stamp)
            profile = S(groups={'waist': NAMES[:2], 'left_arm': NAMES[2:9], 'right_arm': NAMES[9:]},
                digest='hash', limits={n: (-2., 2.) for n in NAMES},
                speed=lambda n: .25, acceleration=lambda n: .5, jerk=lambda n: 162.)
            module('bindu_runtime.robot_setup', RobotSetup=S(load=lambda path: S(profile=profile, path=path)))
            with tempfile.TemporaryDirectory() as directory, patch.dict(sys.modules, modules), patch.object(
                    waist.time, 'monotonic', lambda: clock.value):
                args = S(axis=axis, robot_config=Path('/memory/robot.json'), output=Path(directory)/'capture.jsonl',
                         namespace='bindu_snowbot', vendor_namespace='/', speed=.25, acceleration=.5, jerk=10.)
                if inject_fault:
                    with self.assertRaisesRegex(RuntimeError, 'PDO_NOT_READY'):
                        waist.execute(args, make_plan())
                else:
                    waist.execute(args, make_plan())
                rows = [json.loads(line) for line in args.output.read_text().splitlines()]
                self.assertEqual(rows[-1]['data']['completed'], not inject_fault)
                self.assertEqual(next(r['data']['axis'] for r in rows if r['kind'] == 'plan'), axis)
            self.assertTrue(commands)
            self.assertEqual(requests[0], 'acquire')
            self.assertEqual(requests[-1], 'release')
            self.assertFalse(owned.value)
            for command in commands:
                goal = dict(zip(command.joint_names, command.positions))
                self.assertTrue(all(goal[n] == .1 for n in NAMES if n != axis))
                self.assertEqual(command.resource_groups, list(waist.GROUPS))


if __name__ == '__main__':
    unittest.main()
