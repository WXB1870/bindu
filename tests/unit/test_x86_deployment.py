"""Portable deployment checks; never start a controller or touch a device."""
from bindu_contracts.resources import package_share
import json
import io
from contextlib import redirect_stdout, redirect_stderr
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

from tools.deployment import process_guard, runtime_check, x86_real


class X86DeploymentTests(unittest.TestCase):
    def test_build_fingerprint_covers_application_changes_and_deletions(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            for name in ('framework','application'):
                folder=root/name
                (folder/'src').mkdir(parents=True)
                subprocess.run(['git','init','-q',str(folder)],check=True)
            for repository, name, dependencies in (
                    ('framework','bindu_bimanual_qp','<depend>bindu_contracts</depend>'),
                    ('framework','bindu_contracts',''),('framework','bindu_lio',''),
                    ('application','snowbot_bringup','<exec_depend>bindu_bimanual_qp</exec_depend>')):
                package=root/repository/'src'/name;package.mkdir()
                (package/'package.xml').write_text('<package><name>'+name+'</name>'+dependencies+'</package>')
                (package/'value.py').write_text('original')
            with patch.object(x86_real,'ROOT',root/'framework'):
                first=x86_real.source_digest(root/'application')
                (root/'framework/src/bindu_lio/value.py').write_text('unrelated')
                (root/'application/src/snowbot_bringup/README.md').write_text('documentation')
                self.assertEqual(first,x86_real.source_digest(root/'application'))
                dependency=root/'framework/src/bindu_contracts/value.py'
                dependency.write_text('changed dependency')
                self.assertNotEqual(first,x86_real.source_digest(root/'application'))
                dependency.write_text('original')
                source=root/'application/src/snowbot_bringup/value.py'
                source.write_text('changed')
                self.assertNotEqual(first,x86_real.source_digest(root/'application'))
                source.unlink()
                self.assertNotEqual(first,x86_real.source_digest(root/'application'))

    def setUp(self):
        self.guard = patch.object(x86_real, 'require_stopped', return_value={})
        self.guard.start()
        self.addCleanup(self.guard.stop)

    def test_real_profile_guard_runs_before_native_or_ros_checks(self):
        from bindu_contracts.profile import Profile
        from dataclasses import replace
        profile=Profile.load(package_share("snowbot_bringup")/'config/robots/snowbot/snowbot_real_position.json')
        for invalid in (replace(profile,name='snowbot_real_position_first_run'),
                        replace(profile,capabilities=('joint_position','velocity'))):
            with self.subTest(name=invalid.name),self.assertRaisesRegex(ValueError,'SNOWBOT_PROFILE_REQUIRED'):
                runtime_check.check_runtime(dict(runtime_mode='real'),NS(profile=invalid))

    def test_supported_os_python_pairs_and_abi_rejection(self):
        for release, version, distro in [('22.04', (3, 10), 'humble'), ('24.04', (3, 12), 'jazzy')]:
            info = dict(ID='ubuntu', VERSION_ID=release)
            self.assertEqual(x86_real.target_distro(info, 'x86_64', version), distro)
            if release == '22.04':
                self.assertEqual(x86_real.target_distro(info, 'aarch64', version), distro)
            else:
                with self.assertRaises(ValueError):
                    x86_real.target_distro(info, 'aarch64', version)
            for machine, python in [('armv7l', version), ('x86_64', (3, 11)), ('aarch64', (3, 11))]:
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

    def test_real_vr_wrapper_delegates_before_loading_another_environment(self):
        with tempfile.TemporaryDirectory(prefix="bindu real ' $ ") as tmp:
            root=Path(tmp)
            tools=root/'tools'
            (tools/'deployment').mkdir(parents=True)
            (tools/'__init__.py').touch()
            (tools/'deployment/__init__.py').touch()
            wrapper=tools/'run_vr.sh'
            wrapper.write_text((x86_real.ROOT/'tools/run_vr.sh').read_text())
            # A tiny Python entry records arguments; no ROS or controller runs.
            (tools/'deployment/x86_real.py').write_text(
                'import json,sys; print(json.dumps(sys.argv[1:]))\n')
            wrong_env=root/'wrong-env.sh'
            wrong_env.write_text('exit 73\n')
            env=dict(os.environ,BINDU_VR_ENV=str(wrong_env),PYTHONPATH='',
                     PYTHONDONTWRITEBYTECODE='1')
            workspace=root/'selected workspace'
            for extra in ([],['--enable-commands','--input-timing']):
                with self.subTest(extra=extra):
                    arguments=['--workspace',str(workspace),*extra]
                    result=subprocess.run(['bash',str(wrapper),'real',*arguments],
                        cwd=root.parent,env=env,text=True,capture_output=True)
                    self.assertEqual(result.returncode,0,result.stderr)
                    self.assertEqual(json.loads(result.stdout),['start',*arguments])

    def test_real_vr_help_does_not_require_a_prepared_or_ros_environment(self):
        env=dict(os.environ,BINDU_VR_ENV='/missing/env.sh',PYTHONPATH='',
                 PYTHONDONTWRITEBYTECODE='1')
        result=subprocess.run(['bash',str(x86_real.ROOT/'tools/run_vr.sh'),'real','--help'],
                              env=env,text=True,capture_output=True)
        self.assertEqual(result.returncode,0,result.stderr)
        self.assertIn('--workspace',result.stdout)

    def test_vr_wrapper_keeps_stdlib_before_viewer_backports(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root/'argparse.py').write_text('raise RuntimeError("obsolete argparse imported")\n')
            env_file = root/'env.sh'
            import shlex
            from importlib.util import find_spec
            packages = os.pathsep.join(str(Path(find_spec(name).origin).parents[1])
                                       for name in ('snowbot_bringup', 'bindu_contracts'))
            env_file.write_text('export PYTHONPATH='+shlex.quote(packages)+'\n'
                               'export BINDU_VIEWER_DEPS='+shlex.quote(str(root))+'\n')
            env = dict(os.environ, BINDU_VR_ENV=str(env_file), PYTHONPATH='', PYTHONDONTWRITEBYTECODE='1')
            result = subprocess.run(['bash', str(x86_real.ROOT/'tools/run_vr.sh'), '--unknown-flag'],
                                    cwd=root, env=env, text=True, capture_output=True)
            self.assertEqual(result.returncode, 2)
            self.assertIn('unrecognized arguments', result.stderr)
            self.assertNotIn('obsolete argparse imported', result.stderr)

    def test_vr_wrapper_uses_selected_viewer_deps_without_searching_old_outputs(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);tools=root/'tools'
            (tools/'teleop').mkdir(parents=True)
            (tools/'__init__.py').touch();(tools/'teleop/__init__.py').touch()
            wrapper=tools/'run_vr.sh'
            wrapper.write_text((x86_real.ROOT/'tools/run_vr.sh').read_text())
            (tools/'teleop/run_vr.py').write_text(
                'import json,os; print(json.dumps(os.environ.get("PYTHONPATH","").split(os.pathsep)))\n')
            old=[root/'artifacts/snowbot-viewer-deps',
                 root/'artifacts/vr-live-2026-10-18/viewer-deps']
            for folder in old:(folder/'meshcat').mkdir(parents=True)
            selected=root/'selected deps';selected.mkdir()
            environment=root/'env.sh';environment.write_text(':\n')
            for deps in (None,selected):
                with self.subTest(deps=deps):
                    env=dict(os.environ,BINDU_VR_ENV=str(environment),PYTHONPATH='',
                             PYTHONDONTWRITEBYTECODE='1')
                    env.pop('BINDU_VIEWER_DEPS',None)
                    if deps:env['BINDU_VIEWER_DEPS']=str(deps)
                    result=subprocess.run(['bash',str(wrapper)],cwd=root.parent,env=env,
                                          text=True,capture_output=True)
                    self.assertEqual(result.returncode,0,result.stderr)
                    paths=json.loads(result.stdout)
                    for folder in old:self.assertNotIn(str(folder),paths)
                    self.assertEqual(str(selected) in paths,deps is not None)

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
            (workspace/'site.json').write_text((package_share('snowbot_bringup')/x86_real.TEMPLATE_NAME).read_text())
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
                self.assertEqual(x86_real.main(['start', '--workspace', tmp, '--enable-commands']), 0)
                self.assertIn('--enable-commands', execute.call_args.args[1])
                self.assertNotIn('--check', execute.call_args.args[1])
                execute.reset_mock()
                self.assertEqual(x86_real.main(['start', '--workspace', tmp, '--physics']), 2)
                execute.assert_not_called()

    def test_start_uses_completed_install_without_scanning_source(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root/'build.json').write_text(json.dumps(dict(ready=True, native_process_exit_success=True)))
            (root/'deployment.json').write_text('{}')
            with patch.object(x86_real, 'source_digest', side_effect=AssertionError('startup source scan')):
                x86_real.require_current_build(root, check_sources=False)
                (root/'build.json').write_text(json.dumps(dict(ready=False)))
                with self.assertRaises(ValueError): x86_real.require_current_build(root, check_sources=False)

    def test_changed_source_and_failed_build_require_rebuild(self):
        with tempfile.TemporaryDirectory() as tmp:
            workspace = Path(tmp)
            (workspace/'deployment.json').write_text(json.dumps({
                'application_source': str(package_share('snowbot_bringup').parents[1])}))
            with patch.object(x86_real, 'source_digest', return_value='current'):
                for info in (dict(ready=False, source_sha256='current'),
                             dict(ready=True, source_sha256='old'),
                             dict(ready=True, source_sha256='current')):
                    (workspace/'build.json').write_text(json.dumps(info))
                    with self.assertRaisesRegex(ValueError, '重新 build'):
                        x86_real.require_current_build(workspace)
                (workspace/'build.json').write_text(json.dumps(dict(
                    ready=True, native_process_exit_success=True, source_sha256='current')))
                x86_real.require_current_build(workspace)

    def test_build_selects_one_numeric_prefix_and_requires_clean_child_exit(self):
        with tempfile.TemporaryDirectory(prefix="bindu ' $ ") as tmp:
            workspace = Path(tmp)
            (workspace/'deployment.json').write_text(json.dumps(dict(python=[3, 12],
                application_source=str(package_share('snowbot_bringup').parents[1]))))
            prefix = workspace/'venv/lib/python3.12/site-packages/cmeel.prefix'
            for name in ('pinocchio', 'coal'):
                directory = prefix/'lib/cmake'/name
                directory.mkdir(parents=True)
                (directory/(name+'Config.cmake')).touch()
            with patch.object(x86_real, 'source_digest', return_value='same'), \
                    patch.object(x86_real, 'in_environment') as execute:
                x86_real.build(workspace)
                command = execute.call_args_list[0].args[1]
                self.assertIn('--cmake-clean-cache', command)
                for name in ('pinocchio', 'coal'):
                    self.assertIn('-D'+name+'_DIR='+str(prefix/'lib/cmake'/name), command)
                self.assertTrue(json.loads((workspace/'build.json').read_text())['native_process_exit_success'])
                execute.reset_mock()
                x86_real.build(workspace)
                self.assertNotIn('--cmake-clean-cache',execute.call_args_list[0].args[1])
                (prefix/'lib/cmake/coal/coalConfig.cmake').write_text('# changed numeric dependency')
                execute.reset_mock()
                x86_real.build(workspace)
                self.assertIn('--cmake-clean-cache',execute.call_args_list[0].args[1])
                # A child may finish its test body and still fail in native exit handlers.
                execute.reset_mock()
                execute.side_effect = [None, subprocess.CalledProcessError(-6, 'native-lifecycle')]
                with self.assertRaises(subprocess.CalledProcessError):
                    x86_real.build(workspace)
                info = json.loads((workspace/'build.json').read_text())
                self.assertFalse(info['ready'])
                self.assertFalse(info.get('native_process_exit_success', False))

    def test_missing_numeric_config_cannot_fall_back_to_ros(self):
        with tempfile.TemporaryDirectory() as tmp:
            workspace = Path(tmp)
            (workspace/'deployment.json').write_text(json.dumps(dict(python=[3, 10],
                application_source=str(package_share('snowbot_bringup').parents[1]))))
            directory = workspace/'venv/lib/python3.10/site-packages/cmeel.prefix/lib/cmake/pinocchio'
            directory.mkdir(parents=True)
            (directory/'pinocchioConfig.cmake').touch()
            with patch.object(x86_real, 'source_digest', return_value='same'), \
                    patch.object(x86_real, 'in_environment') as execute:
                with self.assertRaisesRegex(ValueError, '数值依赖缺失'):
                    x86_real.build(workspace)
                execute.assert_not_called()
                self.assertFalse(json.loads((workspace/'build.json').read_text())['ready'])

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
    def test_legacy_simulation_preflight_preserves_its_tool_timing(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            robot,package,solver,_,modules=self.native_fixture(root)
            source=json.loads((package_share('snowbot_bringup')/'config/examples/snowbot/bimanual_snowbot_shovel.json').read_text())
            robot.tool_config.write_text(json.dumps(source))
            parameters=root/'parameters.json'
            parameters.write_text(json.dumps({'teleop':{'input_max_age_s':.3,'command_ttl_s':.35}}))
            with patch.dict('sys.modules',modules),patch.object(runtime_check.importlib,'import_module',return_value=package):
                runtime_check.check_runtime(dict(runtime_mode='simulation',parameters_config=str(parameters)),robot)
            self.assertEqual(solver.call_args.args[0]['input_max_age'],source['input_max_age'])
            self.assertEqual(solver.call_args.args[0]['command_max_age'],source['command_max_age'])

    def native_fixture(self, root):
        from bindu_contracts.profile import Profile
        tool = root/'tool.json'
        source=package_share('snowbot_bringup')/'config/robots/snowbot/bimanual_snowbot_real.json'
        tool.write_bytes(source.read_bytes())
        (root/'runtime_parameters.json').write_bytes(source.with_name('runtime_parameters.json').read_bytes())
        profile = Profile.load(package_share("snowbot_bringup")/'config/robots/snowbot/snowbot_real_position.json')
        robot = NS(tool_config=tool, urdf=root/'robot.urdf', profile=profile, profile_path=root/'profile.json')
        package = NS(**{name: object() for name in (
            'ExecutionState', 'VRInput', 'TeleopModeState', 'GripperState', 'SelectTeleopMode',
            'TeleopSession', 'GripperControl', 'PositionObservation', 'PositionSessionMeta',
            'PositionSessionCommand', 'PositionSessionUpper', 'PositionSessionServo', 'PositionSessionState', 'PositionTrajectory')})
        package.VRInput = NS(get_fields_and_field_types=lambda: dict.fromkeys(
            ('stick', 'head_pose', 'head_stamp', 'head_age_s', 'head_source_id', 'head_seq'), 'fixture'))
        solver = Mock()
        support = Mock()
        modules = {'ament_index_python.packages': NS(get_package_share_directory=lambda _: str(root)),
                   'rclpy.type_support': NS(check_for_type_support=support),
                   'bindu_interfaces.msg': package,
                   'bindu_kinematics.bimanual': NS(BimanualQP=solver)}
        return robot, package, solver, support, modules

    def test_loads_native_execution_vendor_types_and_qp_then_closes(self):
        with tempfile.TemporaryDirectory() as tmp:
            robot, package, solver, support, modules = self.native_fixture(Path(tmp))
            with patch.dict('sys.modules', modules), \
                    patch.object(runtime_check.importlib, 'import_module', return_value=package) as load:
                report = runtime_check.check_runtime(dict(runtime_mode='real', robot_config='robot.json'), robot)
            self.assertEqual(report['software_dependencies'], 'passed')
            self.assertEqual(report['robot_interface'], 'not_checked')
            self.assertEqual(report['vendor_messages']['protocol'],'session_v3')
            self.assertIn(('bindu_execution.interpolation',), [call.args for call in load.call_args_list])
            self.assertIn(('snowbot_ctl_msgs.msg',), [call.args for call in load.call_args_list])
            self.assertEqual(support.call_count, 14)
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
                with patch.dict('sys.modules', modules), \
                        patch.object(runtime_check.importlib, 'import_module', side_effect=load):
                    with self.assertRaisesRegex(ImportError if broken=='execution' else ValueError,
                            'missing _native' if broken=='execution' else 'missing bq_shared_motion_weight'):
                        runtime_check.check_runtime(dict(runtime_mode='real', robot_config='robot.json'), robot)

    def test_target_session_requires_wire_type_without_upper_curve_generator(self):
        with tempfile.TemporaryDirectory() as tmp:
            robot, package, _, support, modules = self.native_fixture(Path(tmp))
            for name in ('PositionObservation','PositionSessionMeta','PositionSessionCommand',
                         'PositionSessionUpper','PositionSessionServo','PositionSessionState'):
                setattr(package, name, object())
            args = dict(runtime_mode='real', robot_config='robot.json')
            del package.PositionTrajectory
            with patch.dict('sys.modules', modules), \
                    patch.object(runtime_check.importlib, 'import_module', return_value=package):
                with self.assertRaises(AttributeError): runtime_check.check_runtime(args, robot)
                package.PositionTrajectory = object()
                self.assertEqual(runtime_check.check_runtime(args, robot)['software_dependencies'], 'passed')
            self.assertIn(package.PositionTrajectory, [call.args[0] for call in support.call_args_list])


class CspInstallTests(unittest.TestCase):
    def test_legacy_module_cli_preserves_help_and_argument_errors(self):
        env = dict(os.environ, PYTHONPATH=os.pathsep.join(sys.path))
        for arguments, code in ((['--help'], 0), ([], 2)):
            with self.subTest(arguments=arguments):
                result = subprocess.run(
                    [sys.executable, '-m', 'tools.deployment.runtime_check', *arguments],
                    cwd=x86_real.ROOT, env=env, capture_output=True, text=True)
                self.assertEqual(result.returncode, code)
                self.assertIn('--vendor-workspace', result.stdout+result.stderr)

    def fixture(self, root):
        fields = {k: v[0] for k, v in runtime_check.CSP_DIAGNOSTIC_FIELDS.items()}
        prefixes = {k: root/'install'/k for k in
                    ('snowbot_ctl_msgs', 'snowbot_servo_control', 'snowbot_dual_arm_control', 'robot_control')}
        for path in prefixes.values(): path.mkdir(parents=True)
        for rel in ('ctl_msgs/msg/PositionObservation.msg',
                    'servo_control/include/servo_control/position_observation.h',
                    'servo_control/src/servo_control/ethercat_driver.cpp',
                    'servo_control/src/ros_controller/rosLoop.cpp'):
            path = root/'src'/rel; path.parent.mkdir(parents=True, exist_ok=True); path.write_text(rel)
        (root/'src/ctl_msgs/msg/PositionObservation.msg').write_text(
            ''.join(kind+' '+name+'\n' for name, kind in fields.items()))
        for source, target in (
                ('ctl_msgs/msg/PositionObservation.msg', prefixes['snowbot_ctl_msgs']/'share/snowbot_ctl_msgs/msg/PositionObservation.msg'),
                ('servo_control/include/servo_control/position_observation.h', prefixes['snowbot_servo_control']/'include/servo_control/position_observation.h')):
            target.parent.mkdir(parents=True, exist_ok=True); target.write_bytes((root/'src'/source).read_bytes())
        binary = prefixes['snowbot_servo_control']/'lib/snowbot_servo_control/servo_node'
        binary.parent.mkdir(parents=True); binary.write_bytes(b'test servo binary')
        generated = prefixes['snowbot_ctl_msgs']/'_position_observation.py'; generated.write_text('# generated\n')
        return fields, prefixes, generated, binary

    def test_schema_requires_all_fields_exact_lengths_and_signedness(self):
        fields = {k: v[0] for k, v in runtime_check.CSP_DIAGNOSTIC_FIELDS.items()}
        self.assertTrue(runtime_check.inspect_csp_schema(fields)['compatible'])
        for changes in ({'csp_mode_display': 'uint8[4]'}, {'csp_target_position_raw': 'int32[20]'},
                        {'csp_send_attempt_ns': 'double'}):
            self.assertFalse(runtime_check.inspect_csp_schema(dict(fields, **changes))['compatible'])
        del fields['csp_feedback_mask']
        self.assertFalse(runtime_check.inspect_csp_schema(fields)['compatible'])

    def test_local_audit_and_optional_frozen_receipt_do_not_overclaim_version(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); fields, prefixes, generated, binary = self.fixture(root)
            report = runtime_check.verify_vendor_install(root, fields, prefixes, generated)
            self.assertTrue(report['installation_checks_passed'])
            self.assertFalse(report['exact_servo_version_verified'])
            self.assertFalse(report['consumer_binary_versions_verified'])
            receipt = dict(files={'src/'+k: dict(after=v) for k, v in report['source_sha256'].items()},
                           binary_sha256=report['binary_sha256'])
            self.assertTrue(runtime_check.verify_vendor_install(root, fields, prefixes, generated, receipt)
                            ['exact_servo_version_verified'])
            broken = dict(receipt, files=dict(receipt['files']))
            broken['files']['src/ctl_msgs/msg/PositionObservation.msg'] = '0'*64
            with self.assertRaisesRegex(ValueError, 'EXPECTED_SOURCE_SHA256_MISMATCH'):
                runtime_check.verify_vendor_install(root, fields, prefixes, generated, broken)
            binary.write_bytes(binary.read_bytes()+b'other version')
            with self.assertRaisesRegex(ValueError, 'EXPECTED_BINARY_SHA256_MISMATCH'):
                runtime_check.verify_vendor_install(root, fields, prefixes, generated, receipt)
            # Without a frozen receipt, the digest does not identify a selected build.
            self.assertFalse(runtime_check.verify_vendor_install(root, fields, prefixes, generated)
                             ['exact_servo_version_verified'])

    def test_generated_python_symlink_can_resolve_to_this_workspaces_rosidl_build(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); fields, prefixes, generated, _ = self.fixture(root)
            target = root/'build/snowbot_ctl_msgs/rosidl_generator_py/_position_observation.py'
            target.parent.mkdir(parents=True); target.write_bytes(generated.read_bytes())
            generated.unlink(); generated.symlink_to(target)
            report = runtime_check.verify_vendor_install(root, fields, prefixes, generated)
            self.assertEqual(report['generated_module'], str(target.resolve()))

    def test_wrong_overlay_module_and_old_install_are_rejected(self):
        for failure in ('overlay', 'module', 'message'):
            with self.subTest(failure=failure), tempfile.TemporaryDirectory() as tmp:
                root = Path(tmp); fields, prefixes, generated, binary = self.fixture(root)
                if failure == 'overlay': prefixes['robot_control'] = root/'other-install'
                elif failure == 'module': generated = root/'other-overlay/_position_observation.py'
                elif failure == 'message':
                    (prefixes['snowbot_ctl_msgs']/'share/snowbot_ctl_msgs/msg/PositionObservation.msg').write_text('old')
                with self.assertRaises(ValueError):
                    runtime_check.verify_vendor_install(root, fields, prefixes, generated)

    def test_cli_only_loads_local_types_and_returns_failure_for_a_bad_schema(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); fields, prefixes, generated, _ = self.fixture(root)
            kind = NS(__module__='csp_generated_fixture', get_fields_and_field_types=lambda: fields)
            support = Mock()
            modules = {'ament_index_python.packages': NS(get_package_prefix=lambda name: str(prefixes[name])),
                       'rclpy.type_support': NS(check_for_type_support=support),
                       'snowbot_ctl_msgs.msg': NS(PositionObservation=kind),
                       'csp_generated_fixture': NS(__file__=str(generated))}
            with patch.dict('sys.modules', modules), redirect_stdout(io.StringIO()) as out:
                self.assertEqual(runtime_check.main(['--vendor-workspace', str(root)]), 0)
                self.assertFalse(json.loads(out.getvalue())['hardware_accessed'])
            fields.pop('csp_mode_display')
            with patch.dict('sys.modules', modules), redirect_stderr(io.StringIO()) as err:
                self.assertEqual(runtime_check.main(['--vendor-workspace', str(root)]), 2)
                self.assertIn('SCHEMA_REBUILD_REQUIRED', json.loads(err.getvalue())['error'])


class PositionSessionPreflightTests(unittest.TestCase):
    def setUp(self):
        self.args = dict(namespace='bindu', vendor_namespace='/')
        self.node = Mock()
        self.node.get_namespace.return_value = '/'
        self.node.get_name.return_value = 'position_probe'
        self.node.count_publishers.return_value = 0
        self.writers, self.readers = {}, {}
        self.qos = NS(QoSCompatibility=NS(OK=0, WARNING=1, ERROR=2),
                      qos_check_compatible=lambda offered, requested: (0 if offered == requested else 2, ''))
        for endpoint in runtime_check.vendor_endpoints(self.args):
            self.add(endpoint['topic'], endpoint['role'], endpoint['node'], endpoint['type'])
        self.node.get_publishers_info_by_topic.side_effect = lambda topic: self.writers.get(topic, [])
        self.node.get_subscriptions_info_by_topic.side_effect = lambda topic: self.readers.get(topic, [])

    def add(self, topic, role, owner, kind):
        space, name = owner.rsplit('/', 1)
        info = NS(node_namespace=space or '/', node_name=name, topic_type=kind, qos_profile='reliable')
        target = self.writers if role == 'publisher' else self.readers
        target.setdefault(topic, []).append(info)
        return info

    def local_readers(self):
        for endpoint in runtime_check.vendor_endpoints(self.args):
            topic = endpoint['topic']
            if not any(i.node_name == 'position_probe' for i in self.readers.get(topic, [])):
                self.add(topic, 'subscription', '/position_probe', endpoint['type'])

    def inspect(self):
        with patch.dict('sys.modules', {'rclpy.qos': self.qos}):
            return runtime_check.inspect_position_routes(self.args, self.node)

    def test_local_endpoint_and_actual_relay_qos_match(self):
        self.local_readers()
        self.assertEqual(self.inspect(), [])
        self.node.create_publisher.assert_not_called()
        self.node.create_client.assert_not_called()

    def test_trajectory_session_keeps_typed_readonly_routes(self):
        self.local_readers()
        self.assertEqual(self.inspect(), [])
        with patch.object(runtime_check, 'check_position_session', return_value='typed') as probe:
            self.assertEqual(runtime_check.check_robot_interface(self.args, 1.), 'typed')
        probe.assert_called_once_with(self.args, 1.)
        self.node.create_publisher.assert_not_called(); self.node.create_client.assert_not_called()

    def test_message_type_name_mismatch_rejects_every_route(self):
        self.local_readers()
        for role, graph in (('publisher', self.writers), ('subscription', self.readers)):
            for topic, values in graph.items():
                for peer in values:
                    if peer.node_name == 'position_probe':continue
                    with self.subTest(topic=topic, role=role):
                        kind = peer.topic_type
                        peer.topic_type = 'std_msgs/msg/String'
                        self.assertIn('SNOWBOT_ENDPOINT_TYPE:'+topic+':'+kind,
                                      runtime_check.inspect_vendor_endpoints(self.args, self.node))
                        peer.topic_type = kind

    def test_relay_qos_uses_its_receiver_and_not_the_passive_reader(self):
        self.local_readers()
        topic = '/bindu/position/upper'
        self.writers[topic][0].qos_profile = 'best_effort'
        self.readers[topic][0].qos_profile = 'best_effort'
        self.assertEqual(self.inspect(), [])
        self.readers[topic][0].qos_profile = 'reliable'
        self.assertIn('SNOWBOT_ENDPOINT_QOS_INCOMPATIBLE:'+topic, self.inspect())

    def test_qos_unknown_and_missing_local_endpoint_are_explicit(self):
        self.local_readers()
        self.qos.qos_check_compatible = lambda *args: (1, 'unknown policy')
        self.assertIn('SNOWBOT_ENDPOINT_QOS_UNVERIFIED:/bindu/position/command', self.inspect())
        self.readers['/bindu/position/command'].pop()
        self.assertIn('SNOWBOT_SESSION_ROUTE_UNVERIFIED:/bindu/position/command', self.inspect())

    def test_readonly_probe_detects_wrong_command_type_and_cleans_up(self):
        import snowbot_hardware.session as session
        self.node.get_clock.return_value.now.return_value.nanoseconds = 10_000_000_000
        self.readers['/bindu/position/command'][0].topic_type = 'std_msgs/msg/String'
        def subscribe(kind, topic, callback, qos):
            name = next(e['type'] for e in runtime_check.vendor_endpoints(self.args) if e['topic'] == topic)
            self.add(topic, 'subscription', '/position_probe', name)
        self.node.create_subscription.side_effect = subscribe
        rclpy = Mock();rclpy.create_node.return_value = self.node
        types = NS(**{name: object() for name in ('PositionObservation', 'PositionSessionState',
                     'PositionSessionCommand', 'PositionSessionUpper', 'PositionSessionServo')})
        modules = {'rclpy':rclpy, 'rclpy.qos':NS(**vars(self.qos), qos_profile_sensor_data='sensor'),
                   'snowbot_ctl_msgs.msg':types}
        health = Mock(observation=object())
        with patch.dict('sys.modules', modules), \
                patch.object(session, 'SessionHealth', return_value=health), \
                patch.object(session, 'decode_observation', return_value=({}, {}, 10.)), \
                patch.object(runtime_check.time, 'monotonic', side_effect=[0.,2.]):
            result = runtime_check.check_robot_interface(self.args, 1.)
        self.assertEqual(result['robot_interface'], 'unavailable')
        self.assertIn('SNOWBOT_ENDPOINT_TYPE:/bindu/position/command:snowbot_ctl_msgs/msg/PositionSessionCommand', result['errors'])
        self.assertEqual(self.node.create_subscription.call_count, 7)
        self.node.create_publisher.assert_not_called();self.node.create_client.assert_not_called()
        self.node.destroy_node.assert_called_once();rclpy.shutdown.assert_called_once()

class DeploymentProcessGuardTests(unittest.TestCase):
    def test_legacy_and_application_modules_share_the_same_stop_guard(self):
        for module, expected in (
                ('bindu_runtime.snowbot_adapter', 'snowbot_adapter'),
                ('snowbot_hardware.adapter', 'snowbot_adapter'),
                ('bindu_runtime.pico_node', 'pico_control'),
                ('snowbot_teleop.node', 'pico_control')):
            with self.subTest(module=module):
                self.assertEqual(process_guard.controller_entries(['python3','-m',module]),[expected])

    def process(self, root, pid, *argv):
        folder = root/str(pid)
        folder.mkdir()
        (folder/'cmdline').write_bytes(b'\0'.join(os.fsencode(arg) for arg in argv)+b'\0')

    def test_native_python_module_and_ros_wrappers_are_all_blocked(self):
        forms = [('/prefix/servo_node',),
                 ('/prefix/venv/bin/python3', '/prefix/lib/bindu_runtime/snowbot_adapter'),
                 ('/usr/bin/python3', '/prefix/lib/bindu_runtime/pico_control'),
                 ('python3', '-m', 'bindu_runtime.bimanual_tool_node'),
                 ('python3', '-m', 'tools.teleop.run_vr'),
                 ('/usr/bin/ros2', 'run', 'bindu_runtime', 'teleop_tool')]
        for argv in forms:
            with self.subTest(argv=argv), tempfile.TemporaryDirectory() as tmp:
                root = Path(tmp)
                self.process(root, 42, *argv)
                with self.assertRaisesRegex(ValueError, 'DEPLOYMENT_CONTROLLERS_RUNNING'):
                    process_guard.require_stopped(root)

    def test_unrelated_python_and_exited_process_do_not_block(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.process(root, 42, 'python3', '-c', 'print("servo_node")')
            (root/'43').mkdir()  # Exited before reading its cmdline.
            self.assertEqual(process_guard.require_stopped(root),
                             dict(controllers=[], unreadable_pids=[]))

    def test_missing_proc_blocks_but_unreadable_process_is_reported(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            with self.assertRaisesRegex(ValueError, 'PROCESS_SCAN_UNAVAILABLE'):
                process_guard.require_stopped(root/'missing')
            self.process(root, 42, 'python3', '/unknown/script.py')
            with patch.object(Path, 'read_bytes', side_effect=PermissionError):
                self.assertEqual(process_guard.require_stopped(root),
                                 dict(controllers=[], unreadable_pids=[42]))

    def test_build_guard_precedes_build_markers_and_subprocesses(self):
        with tempfile.TemporaryDirectory() as tmp:
            workspace = Path(tmp)
            marker = workspace/'build.json'
            marker.write_text('{"ready":true}\n')
            with patch.object(x86_real, 'require_stopped',
                              side_effect=ValueError('DEPLOYMENT_CONTROLLERS_RUNNING')), \
                    patch.object(x86_real, 'source_digest') as digest, \
                    patch.object(x86_real, 'in_environment') as execute:
                with self.assertRaisesRegex(ValueError, 'CONTROLLERS_RUNNING'):
                    x86_real.build(workspace)
                digest.assert_not_called()
                execute.assert_not_called()
            self.assertEqual(marker.read_text(), '{"ready":true}\n')


if __name__ == '__main__':
    unittest.main()
