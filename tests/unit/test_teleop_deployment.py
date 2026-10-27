from bindu_contracts.resources import package_share
from tests.support.paths import ROOT
import json
import tempfile
import unittest
from pathlib import Path
from bindu_runtime.teleop_deployment import load_deployment

SHARE=ROOT/'src/integration/bindu_runtime'


class LegacyDeploymentSelectionTests(unittest.TestCase):
    def test_configuration_needs_an_explicit_destination_and_remains_readonly(self):
        import os,io
        from contextlib import redirect_stderr,redirect_stdout
        from unittest.mock import patch
        from tools.deployment import snowbot_teleop
        with tempfile.TemporaryDirectory() as tmp,patch.dict(os.environ):
            os.environ.pop('BINDU_TELEOP_WORKSPACE',None)
            with redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):
                snowbot_teleop.parser().parse_args(['configure'])
            root=Path(tmp);self.assertEqual(list(root.iterdir()),[])
            config=root/'selected.json'
            with redirect_stdout(io.StringIO()):
                self.assertEqual(snowbot_teleop.main(['configure','--config',str(config)]),0)
            self.assertFalse(json.loads(config.read_text())['arguments']['command_enabled'])
            self.assertEqual(list(root.iterdir()),[config])
            os.environ['BINDU_TELEOP_WORKSPACE']=str(root/'chosen')
            options=snowbot_teleop.parser().parse_args(['status'])
            self.assertEqual(options.config,root/'chosen/site.json')

    def test_shell_refuses_an_unselected_workspace_before_loading_ros(self):
        import os,subprocess
        env=dict(os.environ);env.pop('BINDU_TELEOP_WORKSPACE',None)
        for command in ('build','check','start','status','gripper'):
            with self.subTest(command=command):
                result=subprocess.run(['bash',str(ROOT/'tools/snowbot_teleop.sh'),command],
                                      env=env,text=True,capture_output=True)
                self.assertEqual(result.returncode,2)
                self.assertIn('BINDU_TELEOP_WORKSPACE',result.stderr)
                self.assertNotIn('缺少 /opt/ros',result.stderr)
        with tempfile.TemporaryDirectory() as tmp:
            env['BINDU_TELEOP_WORKSPACE']=tmp
            env.pop('BINDU_ROS_DISTRO',None);env.pop('ROS_DISTRO',None)
            result=subprocess.run(['bash',str(ROOT/'tools/snowbot_teleop.sh'),'build'],
                                  env=env,text=True,capture_output=True)
            self.assertEqual(result.returncode,2)
            self.assertIn('BINDU_ROS_DISTRO',result.stderr)


class ApplicationResourceTests(unittest.TestCase):
    def test_adapter_launch_selects_an_executable_from_its_owning_package(self):
        import ast
        from types import SimpleNamespace
        from bindu_contracts.runtime_mode import resolve_mode
        source=package_share('snowbot_bringup')/'launch/snowbot_adapter.launch.py'
        definition=next(n for n in ast.parse(source.read_text()).body if isinstance(n,ast.FunctionDef) and n.name=='assemble')
        class Configuration:
            def __init__(self,name,default=''):self.name,self.default=name,default
            def perform(self,context):return context.get(self.name,self.default)
        namespace=dict(Path=Path,LaunchConfiguration=Configuration,resolve_mode=resolve_mode,
                       get_package_share_directory=lambda name:str(package_share(name)),Node=lambda **values:values)
        exec(compile(ast.Module(body=[definition],type_ignores=[]),str(source),'exec'),namespace)
        for mode,package,executable in (('real','snowbot_hardware','snowbot_adapter'),
                                        ('simulation','bindu_runtime','execution')):
            actions=namespace['assemble'](dict(runtime_mode=mode,command_enabled='false'))
            self.assertEqual((actions[0]['package'],actions[0]['executable']),(package,executable))

    def test_selected_source_is_explicit_and_rejects_ambiguous_packages(self):
        import os
        from unittest.mock import patch
        from bindu_contracts.resources import package_share, resource_file
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            for folder in ('first','second'):
                package=root/folder/'fixture'
                package.mkdir(parents=True)
                (package/'package.xml').write_text('<package><name>fixture</name></package>')
                (package/'data.json').write_text('{}')
            with patch.dict(os.environ, BINDU_PACKAGE_PATH=str(root/'first')):
                self.assertEqual(resource_file('package://fixture/data.json'),(root/'first/fixture/data.json').resolve())
                (root/'shared.json').write_text('{}')
                (root/'first/fixture/export.json').symlink_to(root/'shared.json')
                self.assertEqual(resource_file('package://fixture/export.json'),(root/'shared.json').resolve())
                for reference in ('package://fixture/../data.json','package://fixture//data.json'):
                    with self.assertRaisesRegex(ValueError,'PACKAGE_PATH'):resource_file(reference)
            with patch.dict(os.environ, BINDU_PACKAGE_PATH=os.pathsep.join(str(root/p) for p in ('first','second'))):
                with self.assertRaisesRegex(ValueError,'AMBIGUOUS'):package_share('fixture')

    def test_snapshot_records_effective_files_and_does_not_open_tls_material(self):
        from bindu_runtime.teleop_deployment import configuration_snapshot
        recipe=json.loads((package_share('snowbot_bringup')/'config/deployments/snowbot_vr_real.json').read_text())
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);(root/'cert.pem').write_text('certificate fixture')
            (root/'secret.key').write_text('private fixture')
            recipe['arguments'].update(cert_file='cert.pem',key_file='secret.key')
            path=root/'site.json';path.write_text(json.dumps(recipe))
            snapshot=configuration_snapshot(path,SHARE)
            self.assertEqual(snapshot['arguments']['command_enabled'],'false')
            self.assertEqual(snapshot['sources']['profile']['values']['name'],'snowbot_real_position')
            self.assertNotIn('private fixture',json.dumps(snapshot))
            self.assertEqual(snapshot['sha256'],configuration_snapshot(path,SHARE)['sha256'])
            recipe['arguments']['namespace']='different_site';path.write_text(json.dumps(recipe))
            self.assertNotEqual(snapshot['sha256'],configuration_snapshot(path,SHARE)['sha256'])


class DeploymentTests(unittest.TestCase):
    def test_target_dynamic_limits_accept_numeric_recipe_and_reject_invalid_or_wrong_mode(self):
        recipe=json.loads((package_share("snowbot_bringup")/'config/deployments/snowbot_vr_real.json').read_text())
        recipe['arguments'].update(trajectory_max_velocity=.4, trajectory_max_acceleration=.8)
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'deploy.json'
            path.write_text(json.dumps(recipe))
            _,args=load_deployment(path,SHARE)
            self.assertEqual(args['trajectory_max_velocity'],'0.4')
            self.assertEqual(args['trajectory_max_acceleration'],'0.8')
            for key in ('trajectory_max_velocity','trajectory_max_acceleration'):
                for invalid in (0.,-1.,float('nan'),float('inf'),True,'bad',None,[],10**400):
                    bad=json.loads(json.dumps(recipe));bad['arguments'][key]=invalid
                    path.write_text(json.dumps(bad))
                    with self.subTest(key=key,invalid=invalid),self.assertRaisesRegex(ValueError,'TARGET_DYNAMIC_LIMIT'):
                        load_deployment(path,SHARE)
            for key,value in (('position_protocol','session_v1'),('runtime_mode','simulation')):
                bad=json.loads(json.dumps(recipe));bad['arguments'][key]=value
                path.write_text(json.dumps(bad))
                with self.assertRaisesRegex(ValueError,'TARGET_LIMITS_REQUIRE_TARGET_STREAM'):
                    load_deployment(path,SHARE)

    def test_target_recipe_rejects_retired_curve_options_and_impedance_combination(self):
        recipe=json.loads((package_share("snowbot_bringup")/'config/deployments/snowbot_vr_real.json').read_text())
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'deploy.json';path.write_text(json.dumps(recipe))
            _,args=load_deployment(path,SHARE)
            self.assertEqual(args['position_protocol'],'session_v3')
            self.assertEqual(args['command_enabled'],'false')
            for key,value in (('trajectory_lead_ms',60.),('trajectory_fast_receipt',True),
                              ('position_protocol','session_v2'),('execution_mode','impedance')):
                bad=json.loads(json.dumps(recipe));bad['arguments'][key]=value
                path.write_text(json.dumps(bad))
                with self.subTest(key=key),self.assertRaises(ValueError):load_deployment(path,SHARE)
    def test_position_assembly_rejects_cross_mode_calibration_before_launch(self):
        from launch import LaunchContext
        from bindu_runtime.teleop_assembly import assemble
        for mode in ('sim','real'):
            _,args=load_deployment(package_share("snowbot_bringup")/f'config/deployments/snowbot_vr_{mode}.json',SHARE)
            context=LaunchContext();context.launch_configurations.update(args)
            self.assertTrue(assemble(context))
            other='real' if mode=='sim' else 'sim'
            context.launch_configurations['gripper_config']=str(package_share("snowbot_bringup")/f'config/robots/snowbot/snowbot_grippers_{other}.json')
            with self.assertRaisesRegex(ValueError,'GRIPPER_CONFIG_MODE_MISMATCH'):assemble(context)

    def test_position_recipes_share_robot_and_reject_demo_shortcut(self):
        _,sim=load_deployment(package_share("snowbot_bringup")/'config/deployments/snowbot_vr_sim.json',SHARE)
        _,real=load_deployment(package_share("snowbot_bringup")/'config/deployments/snowbot_vr_real.json',SHARE)
        self.assertEqual(sim['robot_config'],real['robot_config'])
        self.assertEqual(real['pico_enabled'],'true')
        self.assertEqual(real['command_enabled'],'false')
        recipe=json.loads((package_share("snowbot_bringup")/'config/deployments/snowbot_vr_sim.json').read_text())
        for key,value in (('position_contract','true'),('pico_simulation_demo',True)):
            bad=json.loads(json.dumps(recipe));bad['arguments'][key]=value
            with tempfile.TemporaryDirectory() as tmp:
                path=Path(tmp)/'bad.json';path.write_text(json.dumps(bad))
                with self.assertRaises(ValueError):load_deployment(path,SHARE)

    def test_default_recipe_has_no_startup_motion_or_vr_requirement(self):
        launch,args=load_deployment(SHARE/'config/deployments/teleop_robot_deployment.json',SHARE)
        self.assertEqual(launch,'robot_teleop.launch.py')
        self.assertNotIn('initialize_tool_pose',args)
        self.assertEqual(args['vr_enabled'],'false')
        self.assertEqual(args['tool_input_source'],'tool_pose')
        self.assertEqual(args['description_enabled'],'false')
        self.assertTrue(Path(args['robot_config']).is_file())

    def test_legacy_schema_still_loads_without_a_duplicate_deployment_file(self):
        recipe={'schema_version':1,'robot_launch':'snowbot_sim.launch.py','arguments':{
            'profile':'@runtime/config/robots/snowbot/snowbot_shovel_sim.json',
            'bimanual_config':'@runtime/config/robots/snowbot/bimanual_snowbot_shovel.json'}}
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'legacy.json';path.write_text(json.dumps(recipe))
            launch,args=load_deployment(path,SHARE)
        self.assertEqual(launch,'snowbot_sim.launch.py')
        self.assertEqual(args['initialize_tool_pose'],'false')
        self.assertTrue(Path(args['profile']).is_file())
        self.assertTrue(Path(args['bimanual_config']).is_file())

    def test_pico_recipe_is_explicit_simulation_and_not_default(self):
        launch,args=load_deployment(package_share("snowbot_bringup")/'config/deployments/teleop_pico_snowbot_sim.json',SHARE)
        self.assertEqual(launch,'robot_teleop.launch.py')
        self.assertEqual(args['runtime_mode'],'simulation')
        self.assertEqual(args['pico_enabled'],'true')
        self.assertEqual(args['tool_input_source'],'vr')
        _,default=load_deployment(SHARE/'config/deployments/teleop_robot_deployment.json',SHARE)
        self.assertNotIn('pico_enabled',default)

    def test_pico_rejects_wrong_source_and_external_actuator_backend(self):
        recipe=json.loads((package_share("snowbot_bringup")/'config/deployments/teleop_pico_snowbot_sim.json').read_text())
        for key,value in [('tool_input_source','tool_pose'),('device_backend','external_simulation')]:
            changed=json.loads(json.dumps(recipe));changed['arguments'][key]=value
            with tempfile.TemporaryDirectory() as tmp:
                path=Path(tmp)/'deploy.json';path.write_text(json.dumps(changed))
                with self.assertRaisesRegex(ValueError,'PICO_REQUIRES'):load_deployment(path,SHARE)

    def test_demo_is_explicit_and_rejects_external_or_missing_pico(self):
        recipe=json.loads((package_share("snowbot_bringup")/'config/deployments/teleop_pico_snowbot_sim.json').read_text())
        recipe['arguments']['pico_simulation_demo']=True
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'demo.json';path.write_text(json.dumps(recipe))
            _,args=load_deployment(path,SHARE)
            self.assertEqual(args['pico_simulation_demo'],'true')
            for key,value in (('pico_enabled',False),('pico_simulation_demo','true')):
                bad=json.loads(json.dumps(recipe));bad['arguments'][key]=value
                path.write_text(json.dumps(bad))
                with self.assertRaises(ValueError):load_deployment(path,SHARE)

    def test_unknown_hardware_auto_pose_and_nonteleop_launch_fail_closed(self):
        recipe=json.loads((SHARE/'config/deployments/teleop_robot_deployment.json').read_text())
        for key,value in (('device_backend','real'),('initialize_tool_pose',True),
                          ('navigation_enabled',True),('vr_enabled','false')):
            changed=json.loads(json.dumps(recipe));changed['arguments'][key]=value
            with self.subTest(key=key),tempfile.TemporaryDirectory() as tmp:
                path=Path(tmp)/'deploy.json';path.write_text(json.dumps(changed))
                with self.assertRaises(ValueError):load_deployment(path,SHARE)
        recipe['robot_launch']='skeleton.launch.py'
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'deploy.json';path.write_text(json.dumps(recipe))
            with self.assertRaisesRegex(ValueError,'UNSUPPORTED'):load_deployment(path,SHARE)
