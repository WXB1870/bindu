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

    def test_sine_has_exact_steady_cycles_and_smooth_finite_envelope(self):
        plan = waist.make_sine_plan()
        self.assertEqual(plan.duration, 27.)
        for t in (0., 1., 26., 27., 28.):
            self.assertEqual(plan.sample(t), (0., 0., 0.))
        for t, sign in ((7.25, 1), (9.75, -1), (12.25, 1)):
            self.assertAlmostEqual(plan.sample(t)[0], sign*math.pi/180)
        previous = plan.sample(0.)
        for index in range(1, 27001):
            q, v, a = current = plan.sample(index*.001)
            self.assertLessEqual(abs(q), math.pi/180+1e-12)
            self.assertLessEqual(abs(v), .1)
            self.assertLessEqual(abs(a), .5)
            self.assertLessEqual(abs(a-previous[2])/.001, 10.)
            self.assertAlmostEqual((q-previous[0])/.001, (v+previous[1])/2, delta=1e-6)
            previous = current

    def test_sine_refuses_invalid_or_excessive_dynamics(self):
        for values in (dict(frequency=0), dict(frequency=float('nan')),
                       dict(cycles=True), dict(cycles=2), dict(cycles=21),
                       dict(degrees=20, frequency=1), dict(speed=.001)):
            with self.subTest(values=values), self.assertRaises(ValueError):
                waist.make_sine_plan(**values)

    def test_sine_full_excursion_checked_before_zero_offset_start(self):
        plan = waist.make_sine_plan()
        initial = dict.fromkeys(NAMES, 0.)
        limits = dict.fromkeys(NAMES, (-2., 2.))
        limits['body_joint_1'] = (-2., plan.amplitude+.005)
        with self.assertRaisesRegex(ValueError, 'TEST_POSITION_LIMIT'):
            targets(plan, 0., initial, limits, 'body_joint_1')
        limits['body_joint_1'] = (-2., 2.)
        goal = targets(plan, 7.25, initial, limits, 'body_joint_2')
        self.assertAlmostEqual(goal['body_joint_2'], math.pi/180)
        self.assertTrue(all(goal[n] == 0. for n in NAMES if n != 'body_joint_2'))

    def test_sine_cli_preview_is_offline_and_records_signal_parameters(self):
        result = subprocess.run([sys.executable, '-m', 'tools.teleop.snowbot_waist_test',
                                 '--waveform', 'sine', '--frequency', '.2', '--cycles', '5'],
                                cwd=Path(__file__).resolve().parents[2], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        data = json.loads(result.stdout)
        self.assertTrue(data['preview'])
        self.assertEqual(data['waveform'], 'sine')
        self.assertEqual(data['duration_s'], 27.)
        self.assertEqual(data['frequency_hz'], .2)
        self.assertEqual(data['cycles'], 5)
        self.assertFalse(data['starts_driver'])

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

    def test_default_small_faster_excursion(self):
        plan = make_plan()
        self.assertAlmostEqual(math.degrees(plan.amplitude), 1.)
        peak = plan.sample(plan.dwell+plan.move_seconds/2)
        self.assertGreater(peak[1], .068)
        self.assertLessEqual(peak[1], .1)
        self.assertAlmostEqual(plan.duration, 6.92)

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
        self.assertLess(plan.duration, make_plan(20., .25, .5, 10.).duration)
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

    def test_source_discovery_waits_only_for_empty_graph_before_ownership(self):
        self.assertFalse(waist.checked_pdo_source([], discovering=True))
        self.assertTrue(waist.checked_pdo_source([S(node_name='servo_node')]))
        for publishers in ([], [S(node_name='other')],
                           [S(node_name='servo_node'), S(node_name='servo_node')]):
            with self.assertRaisesRegex(RuntimeError, 'PDO_SOURCE_NOT_UNIQUE_SERVO'):
                waist.checked_pdo_source(publishers)
        with self.assertRaisesRegex(RuntimeError, 'PDO_SOURCE_NOT_UNIQUE_SERVO'):
            waist.checked_pdo_source([S(node_name='other')], discovering=True)

    def test_unknown_dds_name_waits_only_during_bounded_source_discovery(self):
        unknown = S(node_name='_NODE_NAME_UNKNOWN_')
        self.assertFalse(waist.checked_pdo_source([unknown], discovering=True))
        self.assertTrue(waist.checked_pdo_source([S(node_name='servo_node')], discovering=True))
        with self.assertRaisesRegex(RuntimeError, 'PDO_SOURCE_NOT_UNIQUE_SERVO'):
            waist.checked_pdo_source([unknown])
        for publishers in ([unknown, S(node_name='servo_node')],
                           [unknown, unknown], [S(node_name='other')]):
            with self.assertRaisesRegex(RuntimeError, 'PDO_SOURCE_NOT_UNIQUE_SERVO'):
                waist.checked_pdo_source(publishers, discovering=True)

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
        self.assertEqual(result['degrees'], 1.)
        self.assertEqual(result['requested_speed'], .1)
        self.assertEqual(result['requested_acceleration'], .5)
        self.assertLessEqual(result['target_peak_speed'], .1)
        self.assertLessEqual(result['target_peak_acceleration'], .5)
        self.assertLessEqual(result['target_peak_jerk'], 10.)
        self.assertEqual(result['requested_jerk'], 10.)
        self.assertEqual(json_value([float('nan'), float('-inf')]), [None, None])

    def diagnostic_row(self, attempt=1, **changes):
        data = dict(boot_id=1, session=77, receive_attempt=attempt, pdo_sample=attempt,
                    stamp=10.+attempt*.01, pdo_age=0., attempt_age=0., reference_valid=True,
                    output_valid=True, wkc=67, expected_wkc=67, valid_mask=MASK,
                    status=[0x27]*20, position=[0.]*20, speed=[0.]*20,
                    reference_position=[0.]*16, reference_speed=[0.]*16, output_position=[0.]*16,
                    csp_diagnostics_version=1, csp_send_attempt_ns=1000+attempt,
                    csp_command_mask=0xf0000, csp_feedback_mask=0xf0000,
                    csp_controlword=[0x1f]*4, csp_mode_command=[8]*4, csp_mode_display=[8]*4,
                    csp_target_position_raw=[0, 0, 100, 200], csp_target_velocity_raw=[0]*4,
                    csp_target_torque_raw=[0]*4, csp_actual_position_raw=[0, 0, 90, 170],
                    csp_actual_velocity_raw=[0]*4)
        return dict(kind='pdo', data=dict(data, **changes))

    def analyze_rows(self, rows, **options):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'capture.jsonl'
            path.write_text(''.join(json.dumps(r)+'\n' for r in rows))
            return analyze(path, **options)

    def test_unversioned_csp_does_not_guess_position_resolution_from_wire_version(self):
        result = self.analyze_rows([self.diagnostic_row()])['csp_diagnostics']
        self.assertTrue(result['available'])
        self.assertEqual(result['following_error_raw_peak'], 10)
        self.assertFalse(result['encoder_scale_available'])
        self.assertIsNone(result['sent_vs_output_peak_counts'])
        self.assertIsNone(result['output_conversion_counts_per_code_rad'])
        self.assertEqual(result['encoding_comparison_reason'], 'CSP_ENCODER_SCALE_UNSPECIFIED')

    def test_old_and_new_position_scales_preserve_speed_resolution_and_raw_evidence(self):
        for profile, position_counts in (('legacy-65536', 65536), ('outer-262144', 262144)):
            with self.subTest(profile=profile):
                output = [0.]*16; output[14] = .1
                target = round(.1*position_counts/(2*math.pi))
                row = self.diagnostic_row(output_position=output,
                    csp_target_position_raw=[0, 0, target, 0],
                    csp_actual_position_raw=[0, 0, target-5, 0])
                result = self.analyze_rows([row], encoder_profile=profile)['csp_diagnostics']
                self.assertLessEqual(result['sent_vs_output_peak_counts'], .5)
                self.assertEqual(result['following_error_raw_peak'], 5)
                self.assertAlmostEqual(result['output_conversion_counts_per_code_rad'], position_counts/(2*math.pi))
                self.assertAlmostEqual(result['speed_conversion_counts_per_code_rad'], 65536/(2*math.pi))
                self.assertFalse(result['encoder_selection']['physical_units_verified'])
                self.assertFalse(result['encoder_selection']['running_driver_identity_verified'])

    def test_capture_and_release_select_scale_and_conflicting_sources_are_rejected(self):
        metadata = dict(position_counts_per_rev=262144, speed_counts_per_rev=65536)
        rows = [self.diagnostic_row(), dict(kind='plan', data=dict(csp_encoder=metadata))]
        self.assertEqual(self.analyze_rows(rows)['csp_diagnostics']['encoder_selection']['position_counts_per_rev'], 262144)
        with self.assertRaisesRegex(ValueError, 'CSP_ENCODER_SCALE_CONFLICT'):
            self.analyze_rows(rows, encoder_profile='legacy-65536')
        with tempfile.TemporaryDirectory() as tmp:
            release = Path(tmp)/'position_release.json'
            release.write_text(json.dumps(dict(csp_encoder=metadata)))
            result = self.analyze_rows([self.diagnostic_row()], position_release=release)['csp_diagnostics']
            self.assertEqual(result['encoder_selection']['position_counts_per_rev'], 262144)
            self.assertEqual(len(result['encoder_selection']['sources'][0]['sha256']), 64)
            with self.assertRaisesRegex(ValueError, 'CSP_ENCODER_SCALE_CONFLICT'):
                self.analyze_rows([self.diagnostic_row()], position_release=release, encoder_profile='legacy-65536')
            release.write_text('{}')
            with self.assertRaisesRegex(ValueError, 'CSP_ENCODER_METADATA_REQUIRED'):
                self.analyze_rows([self.diagnostic_row()], position_release=release)
        for malformed in (True, {}, dict(position_counts_per_rev=True, speed_counts_per_rev=65536),
                          dict(position_counts_per_rev=262144, speed_counts_per_rev=262144)):
            with self.subTest(malformed=malformed), self.assertRaises(ValueError):
                self.analyze_rows([dict(kind='plan', data=dict(csp_encoder=malformed)), self.diagnostic_row()])

    def test_encoder_metadata_is_selected_for_the_requested_session(self):
        rows = [dict(kind='metadata', data=dict(session=66, csp_encoder=dict(
                    position_counts_per_rev=65536, speed_counts_per_rev=65536))),
                dict(kind='metadata', data=dict(session=77, csp_encoder=dict(
                    position_counts_per_rev=262144, speed_counts_per_rev=65536))),
                self.diagnostic_row(session=66), self.diagnostic_row(2, session=77)]
        result = self.analyze_rows(rows, session=77)
        self.assertEqual(result['other_session_frames_ignored'], 1)
        self.assertEqual(result['csp_diagnostics']['encoder_selection']['position_counts_per_rev'], 262144)
        with self.assertRaisesRegex(ValueError, 'CAPTURE_SESSION_AMBIGUOUS'):
            self.analyze_rows(rows)

    def test_csp_analysis_keeps_bad_receives_and_rejects_stale_or_malformed_evidence(self):
        rows = [self.diagnostic_row(), self.diagnostic_row(),
                self.diagnostic_row(2, pdo_sample=1, wkc=66, reference_valid=False,
                                    output_valid=False, csp_feedback_mask=0),
                self.diagnostic_row(3, csp_mode_display=[8, 8, 9, 8]),
                self.diagnostic_row(4, pdo_age=.03, attempt_age=.03),
                self.diagnostic_row(5, csp_actual_position_raw=[123]),
                self.diagnostic_row(6, csp_diagnostics_version=99)]
        result = self.analyze_rows(rows)
        raw = result['csp_diagnostics']
        self.assertEqual(raw['unique_attempts'], 4)
        self.assertEqual(raw['repeated_attempts_ignored'], 1)
        self.assertEqual(raw['invalid_feedback_attempts'], 2)
        self.assertEqual(raw['invalid_command_attempts'], 1)
        self.assertEqual((raw['malformed_frames'], raw['unsupported_version_frames']), (1, 1))
        self.assertEqual(raw['command_actual_mode_counts'], {'8/8': 1, '8/9': 1})
        self.assertEqual(raw['mode_mismatch_attempts'], 1)
        self.assertEqual(raw['following_error_raw_peak'], 10)
        self.assertEqual(result['invalid_frame_reasons']['bad_wkc'], 1)
        self.assertEqual(result['invalid_frame_reasons']['pdo_stale'], 1)
        self.assertFalse(raw['command_is_drive_acknowledgement'])
        self.assertFalse(raw['cause_identified'])

    def test_csp_pitch_selection_session_filter_and_empty_reference_are_explicit(self):
        event = dict(kind='event', data=dict(state='TARGET_SENT', code='SNOWBOT_TARGET:{"session":77}'))
        rows = [self.diagnostic_row(session=66), event,
                dict(kind='plan', data=dict(axis='body_joint_2')),
                self.diagnostic_row(2, reference_valid=False, output_valid=False)]
        result = self.analyze_rows(rows)
        self.assertEqual(result['other_session_frames_ignored'], 1)
        self.assertEqual(result['samples'], 0)
        self.assertFalse(result['reference_analysis_available'])
        self.assertEqual(result['csp_diagnostics']['following_error_raw_peak'], 30)
        self.assertEqual(result['csp_diagnostics']['array_index'], 3)
        self.assertFalse(result['hardware_acceptance'])
        json.dumps(result, allow_nan=False)
        rows.append(dict(kind='event', data=dict(state='TARGET_SENT', code='SNOWBOT_TARGET:{"session":88}')))
        with self.assertRaisesRegex(ValueError, 'CAPTURE_SESSION_AMBIGUOUS'):
            self.analyze_rows(rows)
        self.assertEqual(self.analyze_rows(rows, session=77)['session'], 77)

    def test_old_capture_does_not_fabricate_raw_csp_evidence(self):
        row = self.diagnostic_row()
        row['data'] = {k: v for k, v in row['data'].items() if not k.startswith('csp_')}
        result = self.analyze_rows([row])['csp_diagnostics']
        self.assertFalse(result['available'])
        self.assertEqual(result['missing_extension_frames'], 1)
        self.assertIsNone(result['following_error_raw_peak'])
        self.assertEqual(result['actual_mode_counts'], {})

    def test_old_capture_without_source_mapping_never_silently_mixes_sessions(self):
        rows = [self.diagnostic_row(session=66), self.diagnostic_row(2, session=77)]
        with self.assertRaisesRegex(ValueError, 'CAPTURE_SESSION_AMBIGUOUS'):
            self.analyze_rows(rows)
        result = self.analyze_rows(rows, session=77)
        self.assertEqual(result['samples'], 1)
        self.assertEqual(result['session_selection'], 'explicit')
        self.assertEqual(self.analyze_rows(rows[:1])['session_selection'], 'observed_only_unmapped')

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
        cases = [(n, f, 'rest') for n in NAMES[:2] for f in (False, True)]
        cases += [('body_joint_1', False, scenario) for scenario in
                  ('transient', 'moving', 'moving_pdo', 'drifting', 'reject',
                   'source_pending', 'source_unknown', 'sine')]
        for axis, inject_fault, scenario in cases:
            clock = S(value=10.)
            requests = []; commands = []; subscriptions = {}
            current = {n: .1 for n in NAMES}; owned = S(value=False)

            def msg(**kwargs):
                return S(**kwargs)

            test_case = self

            class Client:
                def __init__(self, service):
                    self.service = service

                def service_is_ready(self):
                    return True

                def call_async(self, request):
                    if self.service.endswith('/lease'):
                        requests.append(request.operation)
                        if request.operation == 'acquire':
                            if scenario == 'transient':
                                test_case.assertGreaterEqual(clock.value, 10.65-1e-8)
                            if scenario == 'source_pending':
                                test_case.assertGreaterEqual(clock.value, 10.35-1e-8)
                            owned.value = True
                        if request.operation == 'release':
                            owned.value = False
                        response = S(ok=True, lease_id='lease', epoch=1, profile_hash='hash')
                    else:
                        commands.append(request.command)
                        if scenario == 'reject':
                            subscriptions['/bindu_snowbot/events'](S(task_id='',
                                command_id=request.command.command_id, state='TARGET_REJECTED',
                                code='SNOWBOT_TARGET:trajectory_takeover_seed'))
                            return S(done=lambda: True, result=lambda: S(accepted=False, code='INVALID_LEASE'))
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
                    if scenario == 'source_unknown' or scenario == 'source_pending' and clock.value < 10.15:
                        return [S(node_name='_NODE_NAME_UNKNOWN_')]
                    if axis == 'body_joint_2' and clock.value < 10.15:
                        return []
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
                if scenario == 'drifting' and not owned.value:
                    current['body_joint_2'] = .1+.01*(clock.value-10.)
                s.joints.position = [current[n] for n in NAMES]
                if not owned.value and (scenario == 'moving' or scenario == 'transient' and clock.value < 10.45):
                    s.joints.velocity[1] = -.00995
                p = pdo();
                if not owned.value and scenario == 'moving_pdo':
                    p.speed[19] = -.00995
                p.header.stamp = stamp(clock.value); p.pdo_sample = round(clock.value*1000)
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
                         namespace='bindu_snowbot', vendor_namespace='/', speed=.25, acceleration=.5, jerk=10.,
                         encoder_profile='outer-262144')
                test_plan = waist.make_sine_plan() if scenario == 'sine' else make_plan()
                blocked = scenario in ('moving', 'moving_pdo', 'drifting', 'source_unknown')
                if blocked or scenario == 'reject':
                    error = ('TEST_WAIT_TIMEOUT' if scenario == 'source_unknown' else
                             'TEST_NOT_SETTLED' if blocked else 'trajectory_takeover_seed')
                    with self.assertRaisesRegex(RuntimeError, error):
                        waist.execute(args, test_plan)
                elif inject_fault:
                    with self.assertRaisesRegex(RuntimeError, 'PDO_NOT_READY'):
                        waist.execute(args, test_plan)
                else:
                    waist.execute(args, test_plan)
                rows = [json.loads(line) for line in args.output.read_text().splitlines()]
                self.assertEqual(rows[-1]['data']['completed'], not inject_fault and not blocked and scenario != 'reject')
                if blocked:
                    self.assertEqual(requests, [])
                    self.assertEqual(commands, [])
                    continue
                if scenario == 'reject':
                    self.assertIn('trajectory_takeover_seed', rows[-1]['data']['error'])
                    self.assertNotIn('INVALID_LEASE', rows[-1]['data']['error'])
                self.assertTrue(any(row['kind'] == 'settled' for row in rows))
                self.assertEqual(next(r['data']['axis'] for r in rows if r['kind'] == 'plan'), axis)
                if scenario == 'sine':
                    self.assertEqual(next(r['data']['waveform'] for r in rows if r['kind'] == 'plan'), 'sine')
                self.assertEqual(next(r['data']['csp_encoder']['position_counts_per_rev']
                                      for r in rows if r['kind'] == 'plan'), 262144)
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
