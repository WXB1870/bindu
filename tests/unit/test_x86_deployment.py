"""Portable deployment checks; never start a controller or touch a device."""
import json
import os
from pathlib import Path
import subprocess
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

from tools.deployment import runtime_check, x86_real


class X86DeploymentTests(unittest.TestCase):
    def test_supported_os_python_pairs_and_abi_rejection(self):
        for release, version, distro in [('22.04', (3, 10), 'humble'), ('24.04', (3, 12), 'jazzy')]:
            info = dict(ID='ubuntu', VERSION_ID=release)
            self.assertEqual(x86_real.target_distro(info, 'x86_64', version), distro)
            for machine, python in [('aarch64', version), ('x86_64', (3, 11))]:
                with self.assertRaises(ValueError):
                    x86_real.target_distro(info, machine, python)
        with self.assertRaises(ValueError):
            x86_real.target_distro(dict(ID='debian', VERSION_ID='12'), 'x86_64', (3, 12))

    def test_clean_environment_keeps_dds_and_drops_old_overlays(self):
        values = dict(PYTHONPATH='/old', AMENT_PREFIX_PATH='/old', CONDA_PREFIX='/old',
                      BINDU_VR_CONFIG='simulation.json', BINDU_VIEWER_DEPS='/old',
                      ROS_DISTRO='humble', ROS_DOMAIN_ID='193', RMW_IMPLEMENTATION='rmw_cyclonedds_cpp')
        with patch.dict(os.environ, values, clear=True):
            env = x86_real.clean_environment()
        for key in values:
            if key not in ('ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION'):
                self.assertNotIn(key, env)
        self.assertEqual(env['ROS_DOMAIN_ID'], '193')
        self.assertEqual(env['RMW_IMPLEMENTATION'], 'rmw_cyclonedds_cpp')
        self.assertEqual(env['PATH'], x86_real.SYSTEM_PATH)

    def test_generated_environment_quotes_paths_and_removes_overlay_chain(self):
        with tempfile.TemporaryDirectory(prefix="bindu ' $ ") as tmp:
            workspace = Path(tmp)
            vendor = workspace/'vendor local_setup.bash'
            vendor.write_text('export VENDOR_LOADED=1\n')
            (workspace/'venv/bin').mkdir(parents=True)
            (workspace/'venv/bin/activate').write_text('export VENV_LOADED=1\n')
            profile = dict(ros='jazzy', python=[3, 12], vendor_setup=str(vendor))
            script = x86_real.environment_script(workspace, profile)
            # Stand-in for ROS setup; exercise the generated shell, including quoting.
            script = script.replace('source /opt/ros/jazzy/setup.bash || return', ':')
            env_file = workspace/'env.sh'
            env_file.write_text(script)
            command = ['bash', '--noprofile', '--norc', '-ec',
                       'source "$1"; /usr/bin/python3 -c '\
                       "'import os,json; print(json.dumps(dict(os.environ)))'", 'test', str(env_file)]
            env = dict(os.environ, PYTHONPATH='/old', AMENT_PREFIX_PATH='/old')
            env.pop('FASTDDS_BUILTIN_TRANSPORTS', None)
            result = json.loads(subprocess.check_output(command, env=env, text=True))
            self.assertNotIn('AMENT_PREFIX_PATH', result)
            self.assertNotIn('/old', result['PYTHONPATH'])
            self.assertEqual(result['BINDU_VR_CONFIG'], str(workspace/'site.json'))
            self.assertEqual(result['VENDOR_LOADED'], '1')
            self.assertEqual(result['VENV_LOADED'], '1')
            self.assertEqual(result['FASTDDS_BUILTIN_TRANSPORTS'], 'UDPv4')
            env['FASTDDS_BUILTIN_TRANSPORTS'] = 'DEFAULT'
            overridden = json.loads(subprocess.check_output(command, env=env, text=True))
            self.assertEqual(overridden['FASTDDS_BUILTIN_TRANSPORTS'], 'DEFAULT')

    def test_existing_deployment_inherits_udp_without_regenerating_environment(self):
        with tempfile.TemporaryDirectory() as tmp:
            workspace = Path(tmp)
            # An environment generated before the cross-user feedback fix.
            old_script = 'export PREPARED_BEFORE_DDS_FIX=1\n'
            (workspace/'env.sh').write_text(old_script)
            output = workspace/'child.json'
            code = ('import json,os,sys; from pathlib import Path; '
                    'Path(sys.argv[1]).write_text(json.dumps(dict(os.environ)))')
            for selected, expected in ((None, 'UDPv4'), ('', 'UDPv4'), ('DEFAULT', 'DEFAULT')):
                with self.subTest(selected=selected):
                    env = dict(ROS_DOMAIN_ID='193', RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
                    if selected is not None:
                        env['FASTDDS_BUILTIN_TRANSPORTS'] = selected
                    with patch.dict(os.environ, env, clear=True):
                        x86_real.in_environment(workspace, ['/usr/bin/python3', '-c', code, str(output)])
                    child = json.loads(output.read_text())
                    self.assertEqual(child['FASTDDS_BUILTIN_TRANSPORTS'], expected)
                    self.assertEqual(child['ROS_DOMAIN_ID'], '193')
                    self.assertEqual(child['RMW_IMPLEMENTATION'], 'rmw_fastrtps_cpp')
                    self.assertEqual(child['PREPARED_BEFORE_DDS_FIX'], '1')
                    self.assertEqual((workspace/'env.sh').read_text(), old_script)

    def test_vr_wrapper_keeps_stdlib_before_viewer_backports(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root/'argparse.py').write_text('raise RuntimeError("obsolete argparse imported")\n')
            env_file = root/'env.sh'
            import shlex
            env_file.write_text('export BINDU_VIEWER_DEPS='+shlex.quote(str(root))+'\n')
            env = dict(os.environ, BINDU_VR_ENV=str(env_file), PYTHONPATH='', PYTHONDONTWRITEBYTECODE='1')
            result = subprocess.run(['bash', str(x86_real.ROOT/'tools/run_vr.sh'), '--unknown-flag'],
                                    cwd=root, env=env, text=True, capture_output=True)
            self.assertEqual(result.returncode, 2)
            self.assertIn('unrecognized arguments', result.stderr)
            self.assertNotIn('obsolete argparse imported', result.stderr)

    def test_workspace_identity_rejects_moved_repo_and_os(self):
        with tempfile.TemporaryDirectory() as tmp:
            workspace = Path(tmp)
            data = dict(schema=1, root=str(x86_real.ROOT), workspace=str(workspace),
                        ros='jazzy', python=[3, 12], ready=True)
            with patch.object(x86_real, 'system_target', return_value=('jazzy', [3, 12])):
                for key, value in [('root', '/another/checkout'), ('ros', 'humble'), ('python', [3, 10])]:
                    (workspace/'deployment.json').write_text(json.dumps(dict(data, **{key: value})))
                    with self.assertRaises(ValueError):
                        x86_real.read_profile(workspace)
                (workspace/'deployment.json').write_text(json.dumps(data))
                self.assertTrue(x86_real.read_profile(workspace)['ready'])

    def test_incomplete_prepare_and_simulation_cannot_start(self):
        for command in ('check', 'start'):
            with patch.object(x86_real, 'read_profile', return_value=dict(ready=False)), \
                    patch.object(x86_real, 'in_environment') as execute:
                self.assertEqual(x86_real.main([command]), 2)
                execute.assert_not_called()
        with tempfile.TemporaryDirectory() as tmp:
            workspace = Path(tmp)
            (workspace/'install').mkdir()
            (workspace/'install/local_setup.bash').touch()
            (workspace/'site.json').write_text(json.dumps(dict(arguments=dict(runtime_mode='simulation'))))
            with patch.object(x86_real, 'read_profile', return_value=dict(ready=True)), \
                    patch.object(x86_real, 'require_current_build'), \
                    patch.object(x86_real, 'in_environment') as execute:
                self.assertEqual(x86_real.main(['start', '--workspace', tmp]), 2)
                execute.assert_not_called()

    def test_check_uses_site_and_passes_check_flag_without_start(self):
        with tempfile.TemporaryDirectory() as tmp:
            workspace = Path(tmp)
            (workspace/'install').mkdir()
            (workspace/'install/local_setup.bash').touch()
            (workspace/'site.json').write_text(x86_real.TEMPLATE.read_text())
            with patch.object(x86_real, 'read_profile', return_value=dict(ready=True)), \
                    patch.object(x86_real, 'require_current_build'), \
                    patch.object(x86_real, 'in_environment') as execute:
                self.assertEqual(x86_real.main(['check', '--workspace', tmp, '--domain-id', '193']), 0)
                command = execute.call_args.args[1]
                self.assertIn('--check', command)
                self.assertIn((workspace/'site.json').resolve(), command)
                self.assertEqual(json.loads((workspace/'site.json').read_text())['arguments']['command_enabled'], False)
                execute.reset_mock()
                self.assertEqual(x86_real.main(['check-robot', '--workspace', tmp]), 0)
                self.assertIn('--check', execute.call_args.args[1])
                self.assertIn('--check-robot', execute.call_args.args[1])
                execute.reset_mock()
                self.assertEqual(x86_real.main(['start', '--workspace', tmp, '--physics']), 2)
                execute.assert_not_called()

    def test_changed_source_and_failed_build_require_rebuild(self):
        with tempfile.TemporaryDirectory() as tmp:
            workspace = Path(tmp)
            with patch.object(x86_real, 'source_digest', return_value='current'):
                for info in (dict(ready=False, source_sha256='current'),
                             dict(ready=True, source_sha256='old')):
                    (workspace/'build.json').write_text(json.dumps(info))
                    with self.assertRaisesRegex(ValueError, '重新 build'):
                        x86_real.require_current_build(workspace)
                (workspace/'build.json').write_text(json.dumps(dict(ready=True, source_sha256='current')))
                x86_real.require_current_build(workspace)

    def test_prepare_retry_preserves_site_and_marks_failed_dependency_update(self):
        with tempfile.TemporaryDirectory() as tmp:
            workspace = Path(tmp)/'deploy'
            vendor = Path(tmp)/'local_setup.bash'
            vendor.touch()
            options = NS(workspace=workspace, vendor_setup=vendor, ros_distro=None)
            original_is_file = Path.is_file
            def is_file(path):
                return str(path) == '/opt/ros/jazzy/setup.bash' or original_is_file(path)
            with patch.object(x86_real, 'system_target', return_value=('jazzy', [3, 12])), \
                    patch.object(Path, 'is_file', is_file), \
                    patch.object(x86_real, 'in_environment'), \
                    patch.object(x86_real.subprocess, 'run') as run:
                x86_real.prepare(options)
                self.assertTrue(json.loads((workspace/'deployment.json').read_text())['ready'])
                site = workspace/'site.json'
                site.write_text('reviewed site config\n')
                run.side_effect = subprocess.CalledProcessError(1, 'pip')
                with self.assertRaises(subprocess.CalledProcessError):
                    x86_real.prepare(options)
                self.assertFalse(json.loads((workspace/'deployment.json').read_text())['ready'])
                self.assertEqual(site.read_text(), 'reviewed site config\n')
                run.side_effect = None
                x86_real.prepare(options)
                self.assertTrue(json.loads((workspace/'deployment.json').read_text())['ready'])
                self.assertEqual(site.read_text(), 'reviewed site config\n')


class RuntimeCheckTests(unittest.TestCase):
    def test_detects_stale_pico_and_gripper_but_allows_site_overrides(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            source = root/'src/integration/bindu_runtime'
            installed = root/'install/share/bindu_runtime'
            source.mkdir(parents=True)
            installed.mkdir(parents=True)
            for filename in ('pico.json', 'gripper.json'):
                (source/filename).write_text('new')
                (installed/filename).write_text('old')
                with patch.object(runtime_check, 'ROOT', root):
                    with self.assertRaisesRegex(ValueError, '请重建'):
                        runtime_check.check_installed_configs([installed/filename], installed)
                    runtime_check.check_installed_configs([source/filename], installed)

    def native_fixture(self, root):
        tool = root/'tool.json'
        tool.write_text('{}')
        robot = NS(tool_config=tool, urdf=root/'robot.urdf', profile=object(), profile_path=root/'profile.json')
        package = NS(**{name: object() for name in (
            'ExecutionState', 'VRInput', 'TeleopModeState', 'GripperState', 'SelectTeleopMode',
            'TeleopSession', 'GripperControl', 'JointPositionCommand', 'RetInformation')})
        package.JointPositionCommand = NS(get_fields_and_field_types=lambda:
            dict(l_arm_position='double[7]', r_arm_position='double[7]'))
        package.RetInformation = NS(get_fields_and_field_types=lambda:
            {'header': 'std_msgs/Header', **{p+'_'+k: 'double[8]'
             for p in ('l', 'r') for k in ('position', 'speed', 'iq')}})
        solver = Mock()
        support = Mock()
        modules = {'ament_index_python.packages': NS(get_package_share_directory=lambda _: str(root)),
                   'rclpy.type_support': NS(check_for_type_support=support),
                   'bindu_kinematics.bimanual': NS(BimanualQP=solver)}
        return robot, package, solver, support, modules

    def test_loads_native_execution_vendor_types_and_qp_then_closes(self):
        with tempfile.TemporaryDirectory() as tmp:
            robot, package, solver, support, modules = self.native_fixture(Path(tmp))
            with patch.dict('sys.modules', modules), patch.object(runtime_check, 'check_installed_configs'), \
                    patch.object(runtime_check.importlib, 'import_module', return_value=package) as load:
                report = runtime_check.check_runtime(dict(runtime_mode='real', robot_config='robot.json'), robot)
            self.assertEqual(report['software_dependencies'], 'passed')
            self.assertEqual(report['robot_interface'], 'not_checked')
            self.assertFalse(report['vendor_messages']['full_body_feedback'])
            self.assertIn(('bindu_execution.interpolation',), [call.args for call in load.call_args_list])
            self.assertIn(('snowbot_ctl_msgs.msg',), [call.args for call in load.call_args_list])
            self.assertEqual(support.call_count, 9)
            solver.return_value.close.assert_called_once()

    def test_missing_execution_native_or_qp_symbol_fails_preflight(self):
        with tempfile.TemporaryDirectory() as tmp:
            robot, package, solver, _, modules = self.native_fixture(Path(tmp))
            for broken in ('execution', 'qp'):
                def load(name):
                    if broken == 'execution' and name == 'bindu_execution.interpolation':
                        raise ImportError('missing _native')
                    return package
                solver.side_effect = ValueError('missing bq_shared_motion_weight') if broken == 'qp' else None
                with patch.dict('sys.modules', modules), patch.object(runtime_check, 'check_installed_configs'), \
                        patch.object(runtime_check.importlib, 'import_module', side_effect=load):
                    with self.assertRaises((ImportError, ValueError)):
                        runtime_check.check_runtime(dict(runtime_mode='real', robot_config='robot.json'), robot)


class VendorInterfaceTests(unittest.TestCase):
    def setUp(self):
        self.args = dict(namespace='bindu', vendor_namespace='/')
        self.node = Mock()
        self.writers, self.readers = {}, {}
        for expected in runtime_check.vendor_endpoints(self.args):
            space, name = expected['node'].rsplit('/', 1)
            info = NS(node_namespace=space or '/', node_name=name, topic_type=expected['type'])
            target = self.writers if expected['role']=='publisher' else self.readers
            target.setdefault(expected['topic'], []).append(info)
        self.node.get_publishers_info_by_topic.side_effect = lambda topic: self.writers.get(topic, [])
        self.node.get_subscriptions_info_by_topic.side_effect = lambda topic: self.readers.get(topic, [])
        self.health = Mock(messages={'servo': {'native_position_guard_version': 1}})
        self.feedback = NS(header=NS(stamp=NS(sec=10, nanosec=0)),
            l_position=[.1]*8, r_position=[-.1]*8, l_speed=[0.]*8, r_speed=[0.]*8)

    def test_16_axis_messages_are_compatible_but_not_full_body(self):
        command = NS(get_fields_and_field_types=lambda: dict(l_arm_position='double[7]', r_arm_position='double[7]'))
        fields = {'header':'std_msgs/Header', **{p+'_'+k:'double[8]'
            for p in ('l','r') for k in ('position','speed','iq')}}
        feedback = NS(get_fields_and_field_types=lambda:fields)
        report = runtime_check.inspect_vendor_messages(command, feedback)
        self.assertEqual(report['missing_groups'], ['waist', 'head'])
        self.assertTrue(report['compatible']); self.assertFalse(report['full_body_feedback'])
        for name in ('l_speed', 'r_iq'):
            broken = dict(fields); broken[name]='double[7]'
            with self.assertRaisesRegex(ValueError, 'MESSAGE_SCHEMA_MISMATCH'):
                runtime_check.inspect_vendor_messages(command, NS(get_fields_and_field_types=lambda:broken))

    def test_observer_or_wrong_topic_type_cannot_replace_motor_receiver(self):
        self.assertEqual(runtime_check.inspect_vendor_endpoints(self.args, self.node), [])
        self.readers['/joint_position_cmd'] = [NS(node_namespace='/', node_name='observer',
            topic_type='snowbot_ctl_msgs/msg/JointPositionCommand')]
        self.assertTrue(any('ENDPOINT_IDENTITY:/joint_position_cmd' in e
            for e in runtime_check.inspect_vendor_endpoints(self.args, self.node)))
        self.readers['/joint_position_cmd'][0].node_name='robot_upperMonitor'
        self.readers['/joint_position_cmd'][0].topic_type='std_msgs/msg/Float64MultiArray'
        self.assertTrue(any('ENDPOINT_TYPE:/joint_position_cmd' in e
            for e in runtime_check.inspect_vendor_endpoints(self.args, self.node)))

    def test_native_grippers_use_command_subscribers_and_no_new_feedback_topic(self):
        endpoints = runtime_check.vendor_endpoints(dict(self.args, gripper_config='site.json'))
        grips = [e for e in endpoints if '/gripper/' in e['topic']]
        self.assertEqual({e['topic'] for e in grips}, {'/gripper/left_cmd', '/gripper/right_cmd'})
        self.assertTrue(all(e['role']=='subscription' and e['node']=='/servo_node' for e in grips))
        self.assertFalse(any('cmd_sensor' in e['topic'] or 'bindu/gripper_command' in e['topic'] for e in endpoints))

    def test_partial_or_stale_feedback_and_missing_safety_fail_robot_check(self):
        report = runtime_check.assess_robot_interface(self.args, self.node, {'feedback':self.feedback}, 10., self.health)
        self.assertTrue(report['feedback_usable'])
        self.assertEqual(report['missing_feedback_groups'], ['waist', 'head'])
        self.assertEqual(report['robot_interface'], 'unavailable')
        self.feedback.torso_position=[.2,.3];self.feedback.torso_speed=[0.,0.]
        self.feedback.head_position=[.4,.5];self.feedback.head_speed=[0.,0.]
        good=runtime_check.assess_robot_interface(self.args, self.node, {'feedback':self.feedback}, 10., self.health)
        self.assertEqual(good['robot_interface'], 'available')
        self.health.require.side_effect=RuntimeError('SNOWBOT_HEALTH_UNAVAILABLE:servo')
        bad=runtime_check.assess_robot_interface(self.args, self.node, {'feedback':self.feedback}, 10.3, self.health)
        self.assertFalse(bad['feedback_usable'])
        self.assertIn('SNOWBOT_HEALTH_UNAVAILABLE:servo', bad['errors'])
        self.node.create_publisher.assert_not_called()
        self.node.create_client.assert_not_called()

    def test_robot_probe_creates_only_subscriptions_and_always_cleans_up(self):
        rclpy = Mock()
        rclpy.create_node.return_value = self.node
        self.node.get_clock.return_value.now.return_value.nanoseconds = 10000000000
        modules = {'rclpy': rclpy, 'rclpy.qos': NS(qos_profile_sensor_data=object()),
            'snowbot_ctl_msgs.msg': NS(RetInformation=object()), 'std_msgs.msg': NS(String=object())}
        for failure in (None, RuntimeError('probe failed')):
            self.node.reset_mock();rclpy.reset_mock()
            with patch.dict('sys.modules', modules), \
                    patch.object(runtime_check.time, 'monotonic', side_effect=[0., 2.]), \
                    patch.object(runtime_check, 'assess_robot_interface', return_value={}, side_effect=failure):
                if failure:
                    with self.assertRaisesRegex(RuntimeError, 'probe failed'):
                        runtime_check.check_robot_interface(self.args, 1.)
                else:
                    self.assertEqual(runtime_check.check_robot_interface(self.args, 1.), {})
            self.assertEqual(self.node.create_subscription.call_count, 3)
            self.node.create_publisher.assert_not_called()
            self.node.create_client.assert_not_called()
            self.node.destroy_node.assert_called_once()
            rclpy.shutdown.assert_called_once()


if __name__ == '__main__':
    unittest.main()
