"""Offline VR/base boundary checks; no ROS graph or device communication."""
from dataclasses import replace
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace as S, ModuleType
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch
import numpy as np
from tests.support.paths import ROOT
from bindu_contracts.teleoperation import VRFrame
from bindu_teleoperation.base import BaseTeleop, ZERO, validate_base_config
from bindu_teleoperation.vr.receiver import decode_controller
from bindu_runtime.teleop_deployment import load_deployment


SHARE = ROOT/'src/integration/bindu_runtime'
CONFIG = SHARE/'config/robots/snowbot/snowbot_base.json'


class BaseTests(unittest.TestCase):
    def setUp(self):
        self.cfg = json.loads(CONFIG.read_text())
        self.cfg.pop('linear_axis_sign',None)  # Existing generic WebXR convention.
        self.base = BaseTeleop(self.cfg)
        self.now = 10.
        self.seq = 0
        self.source = 'headset'
        self.step()
        for _ in range(8):
            self.step()
        self.assertTrue(self.base.armed)

    def pair(self, left=(0., 0.), right=(0., 0.)):
        self.seq += 1
        return tuple(VRFrame(self.source, self.seq, self.now, side, (),
            mapping='pico_snowbot_v1', stick=stick) for side, stick in (('left', left), ('right', right)))

    def step(self, left=(0., 0.), right=(0., 0.), *, dt=.02, allowed=True, velocity=ZERO):
        self.now += dt
        self.base.ingest(*self.pair(left, right), self.now, self.now)
        self.base.observe(self.now, velocity, self.now, self.now)
        return self.base.tick(self.now, self.now, allowed=allowed)

    def test_only_left_y_and_right_x_drive_two_axis_command(self):
        for _ in range(35):
            result = self.step(left=(1., -1.), right=(-1., 1.))
        self.assertEqual(result, (.1, .5))
        previous = result
        result = self.step(left=(0., 1.), right=(1., 0.))
        self.assertAlmostEqual(previous[0]-result[0], .2*.02)
        self.assertAlmostEqual(previous[1]-result[1], 1.*.02)
        for _ in range(65):
            result = self.step(left=(0., 1.), right=(1., 0.))
        self.assertEqual(result, (-.1, -.5))

    def test_snowbot_forward_reverse_calibration_keeps_yaw_and_rejects_bad_signs(self):
        configured=json.loads(CONFIG.read_text())
        self.assertEqual(configured['linear_axis_sign'],1)
        self.base.cfg=validate_base_config(dict(self.cfg,linear_axis_sign=configured['linear_axis_sign']))
        for _ in range(35):
            result=self.step(left=(0.,-1.),right=(-1.,0.))
        self.assertEqual(result,(-.1,.5))
        for _ in range(65):
            result=self.step(left=(0.,1.),right=(1.,0.))
        self.assertEqual(result,(.1,-.5))
        for sign in (0,True,float('nan'),2):
            with self.assertRaisesRegex(ValueError,'linear_axis_sign'):
                validate_base_config(dict(self.cfg,linear_axis_sign=sign))

    def test_unused_axes_and_all_buttons_have_no_velocity_mapping(self):
        self.assertEqual(self.step(left=(1., 0.), right=(0., 1.)), ZERO)
        for bits in range(8):
            pair = tuple(replace(f, buttons=bits) for f in self.pair())
            self.base.ingest(*pair, self.now, self.now)
            self.assertEqual(self.base.target, ZERO)

    def test_deadzone_matches_old_script_without_rescaling(self):
        self.step(left=(0., -.049), right=(.049, 0.))
        self.assertEqual(self.base.target, ZERO)
        self.step(left=(0., -.05), right=(.05, 0.))
        self.assertAlmostEqual(self.base.target[0], .005)
        self.assertAlmostEqual(self.base.target[1], -.025)

    def test_stop_is_immediate_and_held_stick_cannot_resume(self):
        self.assertGreater(self.step(left=(0., -1.))[0], 0.)
        self.assertEqual(self.step(left=(0., -1.), allowed=False), ZERO)
        for _ in range(10):
            self.assertEqual(self.step(left=(0., -1.)), ZERO)
        for _ in range(8):
            self.step()
        self.assertGreater(self.step(left=(0., -1.))[0], 0.)

    def test_missing_invalid_axes_and_bad_pair_stop_without_fabricated_neutral(self):
        original = self.pair(left=(0., -1.))
        bad = ((), (float('nan'), 0.), (float('inf'), 0.), (1.01, 0.), (True, 0.), (10**400, 0.))
        pairs = [(replace(original[0], stick=axes), original[1]) for axes in bad]
        pairs += [(replace(original[0], valid=False), original[1]),
                  (original[0], replace(original[1], seq=999)),
                  (original[0], replace(original[1], source_id='other')),
                  (replace(original[0], stop=True), original[1])]
        for pair in pairs:
            with self.subTest(pair=pair):
                self.base.ingest(*pair, self.now, self.now)
                self.assertEqual(self.base.command, ZERO)
                self.assertFalse(self.base.armed)
                self.assertFalse(self.base.neutral)

    def test_replayed_input_does_not_renew_expiry_or_rearm(self):
        pair = self.pair()
        self.base.ingest(*pair, self.now, self.now)
        received = self.base.received
        self.base.ingest(*pair, self.now+.1, self.now+.1)
        self.assertEqual(self.base.received, received)
        self.assertFalse(self.base.armed)
        self.assertFalse(self.base.neutral)

    def test_reconnect_and_retired_source_require_neutral(self):
        old = self.pair()
        self.source = 'new'
        self.assertEqual(self.step(left=(0., -1.)), ZERO)
        for _ in range(8):
            self.step()
        self.assertTrue(self.base.armed)
        self.base.ingest(*old, self.now, self.now)
        self.assertEqual(self.base.reason, 'BASE_RETIRED_SOURCE')
        self.assertFalse(self.base.armed)

    def test_source_expiry_and_monotonic_receipt_timeout_are_independent(self):
        self.step(left=(0., -1.))
        self.now += .26
        self.base.observe(self.now, ZERO, self.now, self.now)
        self.base.last_tick = self.now-.02
        self.assertEqual(self.base.tick(self.now, self.now, allowed=True), ZERO)
        self.assertEqual(self.base.reason, 'BASE_INPUT_TIMEOUT')
        self.base.received = self.now-.3
        self.base.input_stamp = self.now
        self.base.last_tick = self.now-.02
        self.assertEqual(self.base.tick(self.now, self.now, allowed=True), ZERO)
        self.assertEqual(self.base.reason, 'BASE_INPUT_TIMEOUT')

    def test_tick_gap_and_feedback_loss_stop_without_ramp(self):
        for kind in ('tick', 'feedback'):
            self.step(left=(0., -1.))
            if kind == 'tick':
                self.base.last_tick = self.now-.2
            else:
                self.base.feedback_velocity = None
                self.base.last_tick = self.now-.02
            self.assertEqual(self.base.tick(self.now, self.now, allowed=True), ZERO)
            self.assertIn(self.base.reason, ('BASE_TICK_GAP', 'BASE_FEEDBACK_UNAVAILABLE'))

    def test_zero_publish_requires_later_sustained_measured_stop(self):
        self.step(left=(0., -1.))
        self.step(allowed=False, velocity=(.1, 0.))
        self.assertFalse(self.base.stopped(self.now, self.now))
        for _ in range(4):
            self.step(allowed=False)
        self.assertFalse(self.base.stopped(self.now, self.now))
        self.step(allowed=False, velocity=(0., .02))
        self.assertFalse(self.base.stopped(self.now, self.now))
        for _ in range(8):
            self.step(allowed=False)
        self.assertTrue(self.base.stopped(self.now, self.now))
        self.assertFalse(self.base.stopped(self.now+.3, self.now+.3))

    def test_duplicate_feedback_does_not_refresh_or_confirm_stop(self):
        stamp = self.base.feedback_stamp
        received = self.base.feedback_received
        self.base.observe(stamp, ZERO, stamp+.1, stamp+.1)
        self.assertEqual(self.base.feedback_received, received)
        self.assertFalse(self.base.healthy(stamp+.21, stamp+.21))
        self.base.observe(stamp+.2, (float('nan'), 0.), stamp+.2, stamp+.2)
        self.assertIsNone(self.base.feedback_velocity)

    def test_feedback_gap_cannot_count_towards_continuous_stop_window(self):
        self.assertTrue(self.base.stopped(self.now, self.now))
        self.now += .3
        self.base.observe(self.now, ZERO, self.now, self.now)
        self.assertFalse(self.base.stopped(self.now, self.now))

    def test_only_driven_axes_need_neutral_to_rearm(self):
        self.step(allowed=False)
        for _ in range(8):
            self.step(left=(1., 0.), right=(0., 1.))
        self.assertTrue(self.base.armed)

    def test_config_rejects_invalid_limits_and_unconfirmed_watchdog_is_explicit(self):
        self.assertIsNone(self.cfg['driver_watchdog_s'])
        for key in ('max_linear_m_s', 'max_angular_rad_s', 'deadzone', 'driver_watchdog_s'):
            for value in (True, 0., -1., float('nan'), float('inf'), 10**400):
                with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                    validate_base_config(dict(self.cfg, **{key: value}))


class WithoutFeedbackTests(unittest.TestCase):
    pair = BaseTests.pair
    def setUp(self):
        self.cfg = dict(json.loads(CONFIG.read_text()), require_feedback=False)
        self.base = BaseTeleop(self.cfg)
        self.now, self.seq, self.source = 10., 0, 'headset'

    def step(self, *args, **kwargs):
        # Exercise the same input path with no Odometry observations at all.
        with patch.object(self.base, 'observe'):
            return BaseTests.step(self, *args, **kwargs)

    def test_neutral_rearms_without_claiming_measured_stop(self):
        self.assertEqual(self.step(left=(0., -1.)), ZERO)
        self.assertEqual(self.step(left=(0., -1.)), ZERO)
        self.assertFalse(self.base.armed)
        self.step()
        self.assertTrue(self.base.armed)
        self.assertFalse(self.base.healthy(self.now, self.now))
        self.assertFalse(self.base.stopped(self.now, self.now))
        self.assertGreater(self.step(left=(0., -1.))[0], 0.)
        self.step(allowed=False)
        self.assertEqual(self.step(left=(0., -1.)), ZERO)
        self.step()
        self.assertGreater(self.step(left=(0., -1.))[0], 0.)
        self.now += .3
        self.assertEqual(self.base.tick(self.now, self.now, allowed=True), ZERO)
        self.assertFalse(self.base.armed)

    def test_feedback_exception_preserves_input_timeout_and_source_rearm(self):
        self.step()
        self.step()
        self.assertGreater(self.step(left=(0., -1.))[0], 0.)
        self.now += .26
        self.base.last_tick = self.now-.02
        self.assertEqual(self.base.tick(self.now, self.now, allowed=True), ZERO)
        self.assertEqual(self.base.reason, 'BASE_INPUT_TIMEOUT')
        self.source = 'new-headset'
        self.assertEqual(self.step(left=(0., -1.)), ZERO)
        self.step()
        self.assertGreater(self.step(left=(0., -1.))[0], 0.)
        pair = self.pair(left=(0., -1.))
        self.base.ingest(replace(pair[0], stick=()), pair[1], self.now, self.now)
        self.assertEqual(self.base.command, ZERO)
        self.assertFalse(self.base.armed)

class DecodeTests(unittest.TestCase):
    def test_real_controller_fields_pass_through_without_changing_buttons(self):
        pose = np.eye(4).ravel(order='F').tolist()
        state = dict(squeezeValue=0., triggerValue=0., aButton=True, bButton=True,
                     thumbstick=True, thumbstickValue=[.2, -.7])
        event = dict(left=pose, right=pose, leftState=dict(state), rightState=dict(state))
        frame = decode_controller(event, 'left', 'id', 1, 10., 'pico_snowbot_v1')
        self.assertEqual(frame.stick, (.2, -.7))
        self.assertEqual(frame.buttons, 7)
        for axes in (None, [], [0.], [0., 0., 0.], [float('nan'), 0.], [True, 0.], ['0', 0.], [10**400, 0.]):
            event['leftState']['thumbstickValue'] = axes
            frame = decode_controller(event, 'left', 'id', 1, 10., 'pico_snowbot_v1')
            self.assertTrue(frame.valid)
            self.assertEqual(frame.buttons, 7)
            self.assertEqual(frame.stick, ())


class DeploymentTests(unittest.TestCase):
    def test_existing_recipes_resolve_the_optional_base_config(self):
        for mode in ('real', 'sim'):
            _, args = load_deployment(SHARE/f'config/deployments/snowbot_vr_{mode}.json', SHARE)
            self.assertEqual(Path(args['base_config']), CONFIG)
            validate_base_config(json.loads(Path(args['base_config']).read_text()))

    def test_base_recipe_requires_actual_vr_and_pico_position_assembly(self):
        recipe = json.loads((SHARE/'config/deployments/snowbot_vr_real.json').read_text())
        for key in ('vr_enabled', 'pico_enabled', 'position_contract'):
            bad = json.loads(json.dumps(recipe))
            bad['arguments'][key] = False
            with tempfile.TemporaryDirectory() as tmp:
                path = Path(tmp)/'bad.json'
                path.write_text(json.dumps(bad))
                with self.subTest(key=key), self.assertRaises(ValueError):
                    load_deployment(path, SHARE)


class AdapterTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Load the real adapter with in-memory ROS message/transport substitutes.
        # This verifies wiring without importing rclpy or contacting a robot.
        def module(**attrs):
            result = ModuleType('substitute')
            result.__dict__.update(attrs)
            return result
        def twist():
            return S(linear=S(x=0., y=0., z=0.), angular=S(x=0., y=0., z=0.))
        substitutes = {'geometry_msgs.msg': module(Twist=twist),
            'nav_msgs.msg': module(Odometry=object),
            'rclpy.qos': module(QoSProfile=lambda **kw: kw,
                ReliabilityPolicy=S(RELIABLE=1, BEST_EFFORT=2), DurabilityPolicy=S(VOLATILE=1)),
            'bindu_runtime.common': module(seconds=lambda x: x)}
        spec = importlib.util.spec_from_file_location('bindu_runtime._base_offline_test',
            SHARE/'bindu_runtime/teleop_base.py')
        cls.adapter = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, substitutes):
            spec.loader.exec_module(cls.adapter)

    def node(self, simulation=False):
        return S(simulation=simulation, now=lambda: 10.,
            create_publisher=Mock(return_value=Mock()), create_subscription=Mock(),
            get_name=lambda: 'pico_control', get_namespace=lambda: '/bindu_snowbot')

    def test_simulation_readonly_and_unknown_watchdog_never_create_command_publisher(self):
        for sim, enabled in ((False, False), (False, True), (True, True)):
            node = self.node(sim)
            bridge = self.adapter.TeleopBase(node, CONFIG, enabled)
            self.assertFalse(bridge.writable)
            node.create_publisher.assert_not_called()
            bridge.close()

    def test_omitted_optional_watchdog_defaults_to_readonly(self):
        node = self.node()
        cfg = json.loads(CONFIG.read_text())
        del cfg['driver_watchdog_s']
        with patch.object(self.adapter.json, 'loads', return_value=cfg):
            bridge = self.adapter.TeleopBase(node, CONFIG, True)
        self.assertFalse(bridge.writable)
        node.create_publisher.assert_not_called()

    def test_watchdog_deferral_is_explicit_and_still_requires_command_enable(self):
        original = json.loads(CONFIG.read_text())
        for enabled, simulation, expected in ((False, False, False), (True, False, True), (True, True, False)):
            node = self.node(simulation)
            cfg = dict(original, require_driver_watchdog=False)
            with patch.object(self.adapter.json, 'loads', return_value=cfg):
                bridge = self.adapter.TeleopBase(node, CONFIG, enabled)
            self.assertEqual(bridge.writable, expected)
            self.assertIsNone(bridge.snapshot()['driver_watchdog_s'])
            self.assertFalse(bridge.snapshot()['require_driver_watchdog'])
            node.create_publisher.return_value.publish.assert_not_called()
        for invalid in ('false', 0, None):
            with self.subTest(invalid=invalid), self.assertRaisesRegex(ValueError, 'require_driver_watchdog'):
                validate_base_config(dict(original, require_driver_watchdog=invalid))

    def test_no_feedback_policy_keeps_output_exclusivity_without_odom_or_stop_claim(self):
        node = self.node()
        cfg = dict(json.loads(CONFIG.read_text()), require_driver_watchdog=False,
                   require_feedback=False, receiver_node='/car_control')
        with patch.object(self.adapter.json, 'loads', return_value=cfg):
            bridge = self.adapter.TeleopBase(node, CONFIG, True)
        node.create_subscription.assert_not_called()
        own = S(node_name='pico_control', node_namespace='/bindu_snowbot')
        node.get_publishers_info_by_topic = Mock(return_value=[own])
        node.get_subscriptions_info_by_topic = Mock(return_value=[S(
            node_name='car_control', node_namespace='/', topic_type='geometry_msgs/msg/Twist')])
        bridge.check_graph(10.)
        self.assertTrue(bridge.graph_ok)
        node.get_publishers_info_by_topic.assert_called_once_with('/cmd_vel')
        view = bridge.snapshot()
        self.assertIsNone(view['stop_confirmed'])
        self.assertIsNone(view['measured'])
        self.assertFalse(view['require_feedback'])
        bridge.on_odom(object())  # Disabled feedback must never be consumed.
        node.get_publishers_info_by_topic.return_value = [own, own]
        bridge.check_graph(10.2)
        self.assertFalse(bridge.graph_ok)
        for invalid in ('false', 0, None):
            with self.subTest(invalid=invalid), self.assertRaisesRegex(ValueError, 'require_feedback'):
                validate_base_config(dict(cfg, require_feedback=invalid))

    def test_pico_stop_condition_ignores_only_explicitly_disabled_base_feedback(self):
        import ast
        from bindu_teleoperation.pico import PicoFeedback
        path = SHARE/'bindu_runtime/pico_node.py'
        cls = next(n for n in ast.parse(path.read_text()).body if isinstance(n, ast.ClassDef) and n.name == 'PicoNode')
        method = next(n for n in cls.body if getattr(n, 'name', '') == 'feedback')
        namespace = dict(PicoFeedback=PicoFeedback, seconds=lambda stamp: stamp,
                         joints_at_rest=lambda *args: True, time=S(monotonic=lambda: 10.))
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), namespace)
        state = S(lease_active=False, state='IDLE', reference=S(name=[]), joints=object(), feedback_stamp=10.)
        owner = S(reading=lambda: (state, {}), position_ops=S(modes=lambda **kw: ('closed', 'closed'), stopped=lambda: True),
            control=S(phase='standby'), handoff_complete=False, names=[], stop_request=None, now=lambda: 10.)
        for required in (True, False):
            core = S(healthy=Mock(return_value=False), stopped=Mock(return_value=False))
            owner.base = S(writable=True, cfg=dict(require_feedback=required), core=core)
            result = namespace['feedback'](owner)
            self.assertEqual(result.healthy, not required)
            self.assertEqual(result.stopped, not required)
            if not required:
                core.healthy.assert_not_called()
                owner.stop_request = dict(failed=True, deadline=20.)
                self.assertFalse(namespace['feedback'](owner).healthy)
                owner.stop_request = None
                owner.reading = Mock(side_effect=RuntimeError('PICO_STOP_UNCONFIRMED:ARM_FAULT'))
                self.assertFalse(namespace['feedback'](owner).healthy)

    def test_site_pins_both_car_control_endpoints(self):
        node, bridge, _, feedback = self.writable()
        bridge.cfg.update(receiver_node='/car_control', feedback_node='/car_control')
        feedback.node_name, feedback.node_namespace = 'wrong', '/'
        receiver = S(node_name='car_control', node_namespace='/', topic_type='geometry_msgs/msg/Twist')
        node.get_subscriptions_info_by_topic.return_value = [receiver]
        bridge.check_graph(10.)
        self.assertFalse(bridge.graph_ok)
        feedback.node_name = 'car_control'
        bridge.check_graph(10.2)
        self.assertTrue(bridge.graph_ok)
        receiver.node_name = 'wrong'
        bridge.check_graph(10.4)
        self.assertFalse(bridge.graph_ok)

    def writable(self):
        node = self.node()
        cfg = dict(json.loads(CONFIG.read_text()), driver_watchdog_s=.2)
        with patch.object(self.adapter.json, 'loads', return_value=cfg):
            bridge = self.adapter.TeleopBase(node, CONFIG, True)
        own = S(node_name='pico_control', node_namespace='/bindu_snowbot')
        feedback = S(topic_type='nav_msgs/msg/Odometry', endpoint_gid=[1]*16)
        node.get_publishers_info_by_topic = Mock(side_effect=lambda topic: [own] if topic == '/cmd_vel' else [feedback])
        node.get_subscriptions_info_by_topic = Mock(return_value=[S(topic_type='geometry_msgs/msg/Twist')])
        return node, bridge, own, feedback

    def test_exclusive_output_and_pinned_feedback_instance(self):
        node, bridge, own, feedback = self.writable()
        bridge.check_graph(10.)
        self.assertTrue(bridge.graph_ok)
        node.get_publishers_info_by_topic.side_effect = lambda topic: [own, own] if topic == '/cmd_vel' else [feedback]
        bridge.check_graph(10.2)
        self.assertFalse(bridge.graph_ok)
        feedback.endpoint_gid = [2]*16
        node.get_publishers_info_by_topic.side_effect = lambda topic: [own] if topic == '/cmd_vel' else [feedback]
        bridge.check_graph(10.4)
        self.assertFalse(bridge.graph_ok)

    def test_changed_feedback_source_cannot_recover_when_original_gid_returns(self):
        _, bridge, _, feedback = self.writable()
        bridge.check_graph(10.)
        self.assertTrue(bridge.graph_ok)
        feedback.endpoint_gid = [2]*16
        bridge.check_graph(10.2)
        self.assertFalse(bridge.graph_ok)
        feedback.endpoint_gid = [1]*16
        bridge.check_graph(10.4)
        self.assertFalse(bridge.graph_ok)
        self.assertIsNone(bridge.core.feedback_velocity)

    def test_unknown_feedback_gid_cannot_establish_identity(self):
        for gid in (None, [], [0]*16):
            _, bridge, _, feedback = self.writable()
            feedback.endpoint_gid = gid
            with self.subTest(gid=gid):
                bridge.check_graph(10.)
                self.assertFalse(bridge.graph_ok)

    def test_adapter_only_sets_forward_yaw_and_close_sends_zero(self):
        _, bridge, _, _ = self.writable()
        control = S(pending=None, phase='following', source='headset', seq=1)
        bridge.core.source, bridge.core.seq = control.source, control.seq
        bridge.core.tick = Mock(return_value=(.1, -.5))
        bridge.tick(control, healthy=True)
        msg = bridge.publisher.publish.call_args.args[0]
        self.assertEqual((msg.linear.x, msg.angular.z), (.1, -.5))
        self.assertEqual((msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y), (0.,)*4)
        self.assertTrue(bridge.core.tick.call_args.kwargs['allowed'])
        for phase, pending, healthy in (('locked', None, True), ('preparing', None, True),
                ('following', object(), True), ('following', None, False)):
            control.phase, control.pending = phase, pending
            bridge.tick(control, healthy)
            self.assertFalse(bridge.core.tick.call_args.kwargs['allowed'])
        control.phase, control.pending, control.seq = 'following', None, 2
        bridge.tick(control, True)
        self.assertFalse(bridge.core.tick.call_args.kwargs['allowed'])
        bridge.close()
        msg = bridge.publisher.publish.call_args.args[0]
        self.assertEqual((msg.linear.x, msg.angular.z), ZERO)

    def test_feedback_frame_is_checked_and_no_target_is_used_as_measurement(self):
        _, bridge, _, _ = self.writable()
        bridge.check_graph(10.)
        msg = S(header=S(stamp=10.), child_frame_id='wrong',
                twist=S(twist=S(linear=S(x=.1), angular=S(z=.2))))
        bridge.on_odom(msg)
        self.assertIsNone(bridge.core.feedback_velocity)
        msg.child_frame_id = bridge.cfg['feedback_frame']
        bridge.on_odom(msg)
        self.assertEqual(bridge.core.feedback_velocity, (.1, .2))

    def test_decoder_pico_and_adapter_chain_with_independent_input_and_timer_rates(self):
        from bindu_teleoperation.pico import PicoControl, PicoFeedback
        from bindu_teleoperation.vr.receiver import ButtonCounters
        node, bridge, _, _ = self.writable()
        clock = [10.]
        node.now = lambda: clock[0]
        control = PicoControl(initial_load='unknown', guided_loading=True, dual_modes=True,
                              direct_grippers=True)
        counters = [ButtonCounters(), ButtonCounters()]
        pose = np.eye(4).ravel(order='F').tolist()
        sequence = 0
        next_input = next_feedback = 10.
        max_linear = max_angular = 0.
        with patch.object(self.adapter.time, 'monotonic', side_effect=lambda: clock[0]):
            # Synthetic event ordering: 200 Hz timer, 60 Hz input, 50 Hz odom.
            # Feedback is explicitly injected; this is not a physical model.
            for tick in range(241):
                clock[0] = now = 10.+tick*.005
                bridge.check_graph(now)
                if now >= next_feedback:
                    moving = .3 <= now-10. < 1.
                    bridge.on_odom(S(header=S(stamp=now), child_frame_id=bridge.cfg['feedback_frame'],
                        twist=S(twist=S(linear=S(x=.02 if moving else 0.), angular=S(z=0.)))))
                    next_feedback += .02
                healthy = bridge.core.healthy(now, now)
                control.update_feedback(PicoFeedback(now, bridge.core.stopped(now, now), healthy=healthy), now)
                if now >= next_input:
                    sequence += 1
                    stop = now-10. >= 1.
                    state = dict(squeezeValue=0., triggerValue=0., aButton=False,
                                 bButton=False, thumbstick=False)
                    event = dict(left=pose, right=pose,
                        leftState=dict(state, bButton=stop,
                            thumbstickValue=[0., -1. if now-10. >= .3 else 0.]),
                        rightState=dict(state, thumbstickValue=[-.5 if now-10. >= .3 else 0., 0.]))
                    pair = tuple(counters[i].update(decode_controller(event, side, 'headset', sequence,
                        now, 'pico_snowbot_v1')) for i, side in enumerate(('left', 'right')))
                    bridge.ingest(pair, now)
                    control.ingest(*pair, now)
                    next_input += 1./60.
                control.tick(now)
                bridge.tick(control, healthy)
                msg = bridge.publisher.publish.call_args.args[0]
                max_linear = max(max_linear, msg.linear.x)
                max_angular = max(max_angular, msg.angular.z)
                if now-10. >= 1.02:
                    self.assertEqual((msg.linear.x, msg.angular.z), ZERO)
            self.assertAlmostEqual(max_linear, .1)
            self.assertAlmostEqual(max_angular, .25)
            self.assertEqual((control.phase, control.reason), ('locked', 'PICO_Y_STOP'))
            self.assertEqual(control.teleop_mode, 'independent_arms')


if __name__ == '__main__':
    unittest.main()
