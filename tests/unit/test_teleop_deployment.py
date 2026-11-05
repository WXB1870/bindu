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
        for command in ('check','start','status','gripper'):
            with self.subTest(command=command):
                result=subprocess.run(['bash',str(ROOT/'tools/snowbot_teleop.sh'),command],
                                      env=env,text=True,capture_output=True)
                self.assertEqual(result.returncode,2)
                self.assertIn('BINDU_TELEOP_WORKSPACE',result.stderr)
                self.assertNotIn('缺少 /opt/ros',result.stderr)
        with tempfile.TemporaryDirectory() as tmp:
            result=subprocess.run(['bash',str(ROOT/'tools/snowbot_teleop.sh'),'build'],
                                  env=env,text=True,capture_output=True)
            self.assertEqual(result.returncode,2)
            self.assertIn('tools.deployment.x86_real build',result.stderr)

    def test_help_does_not_need_an_application_or_ros_installation(self):
        import os,subprocess
        env=dict(os.environ,PYTHONPATH='',PYTHONDONTWRITEBYTECODE='1')
        for key in ('BINDU_PACKAGE_PATH','AMENT_PREFIX_PATH','BINDU_TELEOP_WORKSPACE'):
            env.pop(key,None)
        result=subprocess.run(['bash',str(ROOT/'tools/snowbot_teleop.sh'),'check','--help'],
                              env=env,text=True,capture_output=True)
        self.assertEqual(result.returncode,0,result.stderr)
        self.assertIn('--config',result.stdout)

    def test_selected_environment_failure_cannot_fall_through_to_legacy_loading(self):
        import os,subprocess
        with tempfile.TemporaryDirectory() as tmp:
            (Path(tmp)/'env.sh').write_text('echo selected_environment\nreturn 29\n')
            env=dict(os.environ,BINDU_TELEOP_WORKSPACE=tmp,ROS_DISTRO='jazzy',BINDU_ROS_DISTRO='humble')
            result=subprocess.run(['bash',str(ROOT/'tools/snowbot_teleop.sh'),'check','--config','chosen.json'],
                                  env=env,text=True,capture_output=True)
            self.assertEqual(result.returncode,29)
            self.assertIn('selected_environment',result.stdout)
            self.assertNotIn('不一致',result.stderr)


class ApplicationResourceTests(unittest.TestCase):
    def test_generic_nodes_import_without_snowbot_or_repository_tools(self):
        import importlib.util,os,subprocess,sys
        if importlib.util.find_spec('rclpy') is None:
            self.skipTest('ROS environment required for node imports')
        code='''
import importlib.abc,sys
class BlockApplications(importlib.abc.MetaPathFinder):
    def find_spec(self,fullname,path=None,target=None):
        if fullname.split('.')[0] in ('snowbot_hardware','snowbot_teleop','snowbot_bringup','tools'):
            raise ModuleNotFoundError('unexpected application/source import: '+fullname)
sys.meta_path.insert(0,BlockApplications())
from bindu_runtime.bimanual_tool_node import BimanualToolNode
from bindu_runtime.simulation_devices import SimulationDevices,PhysicsJoints,PhysicsBase
'''
        with tempfile.TemporaryDirectory() as directory:
            result=subprocess.run([sys.executable,'-c',code],cwd=directory,
                env=dict(os.environ,PYTHONPATH=os.pathsep.join(sys.path)),capture_output=True,text=True)
        self.assertEqual(result.returncode,0,result.stdout+result.stderr)

    def test_position_simulation_keeps_its_calibrated_grasp_separate_from_real_operator_policy(self):
        from bindu_contracts.resources import resource_file
        share=package_share('snowbot_bringup')
        deployment=json.loads((share/'config/deployments/snowbot_vr_sim.json').read_text())
        real_deployment=json.loads((share/'config/deployments/snowbot_vr_real.json').read_text())
        self.assertEqual(deployment['arguments']['robot_config'],real_deployment['arguments']['robot_config'])
        robot_path=resource_file(deployment['arguments']['robot_config'])
        robot=json.loads(robot_path.read_text())
        tool=json.loads((robot_path.parent/robot['teleop']['tool']).read_text())
        self.assertNotIn('tool_grasp_confirmation',tool)
        self.assertEqual(tool['grasp_reference_source'],'measured_axial')
        self.assertIs(tool['rigid_reference_prediction'],False)
        for recipe,expected in ((deployment,'calibrated'),(real_deployment,'operator')):
            from snowbot_hardware.parameters import load_mode_config
            pico=load_mode_config(resource_file(recipe['arguments']['pico_config']),recipe['arguments']['runtime_mode'])
            self.assertEqual(pico['tool_grasp_confirmation'],expected)
        grippers=load_mode_config(resource_file(deployment['arguments']['gripper_config']),'simulation')
        self.assertIs(grippers['simulation_only'],True)
        self.assertFalse(grippers.get('independent_trigger_control',False))

    def test_adapter_launch_selects_an_executable_from_its_owning_package(self):
        import ast
        from types import SimpleNamespace
        from bindu_contracts.runtime_mode import resolve_mode
        from bindu_runtime.teleop_deployment import TARGET_STREAM_LIMIT_DEFAULTS, target_stream_limits, execution_backend
        source=package_share('snowbot_bringup')/'launch/snowbot_adapter.launch.py'
        definition=next(n for n in ast.parse(source.read_text()).body if isinstance(n,ast.FunctionDef) and n.name=='assemble')
        class Configuration:
            def __init__(self,name,default=''):self.name,self.default=name,default
            def perform(self,context):return context.get(self.name,self.default)
        namespace=dict(Path=Path,LaunchConfiguration=Configuration,resolve_mode=resolve_mode,
                       TARGET_STREAM_LIMIT_DEFAULTS=TARGET_STREAM_LIMIT_DEFAULTS,target_stream_limits=target_stream_limits,execution_backend=execution_backend,
                       get_package_share_directory=lambda name:str(package_share(name)),Node=lambda **values:values)
        exec(compile(ast.Module(body=[definition],type_ignores=[]),str(source),'exec'),namespace)
        for mode,package,executable in (('real','snowbot_hardware','snowbot_adapter'),
                                        ('simulation','bindu_runtime','execution')):
            actions=namespace['assemble'](dict(runtime_mode=mode,command_enabled='false'))
            self.assertEqual((actions[0]['package'],actions[0]['executable']),(package,executable))
        with tempfile.TemporaryDirectory() as tmp:
            parameters=Path(tmp)/'parameters.json'
            parameters.write_text(json.dumps({'execution':{'target_stream_brake_lead_s':.1}}))
            actions=namespace['assemble'](dict(runtime_mode='real',
                                              parameters_config=str(parameters)))
            self.assertEqual(actions[0]['parameters'][0]['parameters_config'],str(parameters))
            self.assertNotIn('target_stream_brake_lead_s',actions[0]['parameters'][0])
            self.assertEqual(actions[0]['parameters'][0]['vendor_namespace'],'/')
            with self.assertRaisesRegex(ValueError,'BRAKE_LEAD_MOVED'):
                namespace['assemble'](dict(runtime_mode='real',target_stream_brake_lead_s='.1'))

    def test_shared_timing_is_forwarded_to_all_position_nodes_before_launch(self):
        import ast
        from bindu_runtime.robot_setup import RobotSetup
        from bindu_contracts.runtime_mode import resolve_mode
        from bindu_runtime.teleop_deployment import TARGET_STREAM_LIMIT_DEFAULTS, target_stream_limits, execution_backend
        source=package_share('snowbot_bringup')/'snowbot_bringup/assembly.py'
        definition=next(n for n in ast.parse(source.read_text()).body if getattr(n,'name','')=='assemble')
        class Configuration:
            def __init__(self,name,default=''):self.name,self.default=name,default
            def perform(self,context):return context.get(self.name,self.default)
        namespace=dict(Path=Path,json=json,RobotSetup=RobotSetup,resolve_mode=resolve_mode,
            LaunchConfiguration=Configuration,TARGET_STREAM_LIMIT_DEFAULTS=TARGET_STREAM_LIMIT_DEFAULTS,
            target_stream_limits=target_stream_limits,execution_backend=execution_backend,get_package_share_directory=lambda name:str(package_share(name)),
            Node=lambda **values:values,LogInfo=lambda **values:values,IfCondition=lambda value:value,
            ParameterValue=lambda value,**kw:value)
        exec(compile(ast.Module(body=[definition],type_ignores=[]),str(source),'exec'),namespace)
        recipe=package_share('snowbot_bringup')/'config/deployments/snowbot_vr_real.json'
        _,context=load_deployment(recipe,SHARE)
        with tempfile.TemporaryDirectory() as tmp:
            parameters=Path(tmp)/'parameters.json'
            parameters.write_text(json.dumps({'execution':{'target_stream_brake_lead_s':.04},
                                              'pico':{'prepare_command_period_s':.2}}))
            context['parameters_config']=str(parameters)
            actions=namespace['assemble'](context)
            nodes={item['executable']:item['parameters'][0] for item in actions if 'executable' in item}
            for name in ('teleop_tool','pico_control','snowbot_adapter'):
                self.assertEqual(nodes[name]['parameters_config'],str(parameters))
            self.assertNotIn('target_stream_brake_lead_s',nodes['snowbot_adapter'])
            for name in ('teleop_tool','pico_control'):
                self.assertIs(nodes[name]['position_contract'],True)
            context['target_stream_brake_lead_s']='.12'
            with self.assertRaisesRegex(ValueError,'SNOWBOT_BRAKE_LEAD_MOVED'):
                namespace['assemble'](context)
            context.pop('target_stream_brake_lead_s')
            context['position_contract']='true'
            with self.assertRaisesRegex(ValueError,'POSITION_CONTRACT_REMOVED'):
                namespace['assemble'](context)
            context.pop('position_contract')
            _,simulation=load_deployment(package_share('snowbot_bringup')/'config/deployments/snowbot_vr_sim.json',SHARE)
            actions=namespace['assemble'](simulation)
            nodes={item['executable']:item['parameters'][0] for item in actions if 'executable' in item}
            self.assertIn('snowbot_position_sim',nodes)
            for name in ('teleop_tool','pico_control'):
                self.assertIs(nodes[name]['position_contract'],True)
            simulation['pico_simulation_demo']='true'
            with self.assertRaisesRegex(ValueError,'PICO_DEMO_REQUIRES'):
                namespace['assemble'](simulation)
            _,demo=load_deployment(package_share('snowbot_bringup')/'config/examples/snowbot/teleop_pico_snowbot_sim.json',SHARE)
            demo['device_backend']='external_simulation'
            with self.assertRaisesRegex(ValueError,'PICO_REQUIRES'):
                namespace['assemble'](demo)

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
            self.assertEqual(snapshot['sources']['parameters_config']['values']['schema_version'],1)
            self.assertEqual(snapshot['sources']['parameters_config']['values']['pico']['prepare_command_ttl_s'],.25)
            self.assertNotIn('private fixture',json.dumps(snapshot))
            self.assertEqual(snapshot['sha256'],configuration_snapshot(path,SHARE)['sha256'])
            recipe['arguments']['namespace']='different_site';path.write_text(json.dumps(recipe))
            self.assertNotEqual(snapshot['sha256'],configuration_snapshot(path,SHARE)['sha256'])
            effective=dict(snapshot['arguments'],command_enabled='true',namespace='cli_override')
            overridden=configuration_snapshot(path,SHARE,effective_arguments=effective)
            self.assertEqual(overridden['arguments']['command_enabled'],'true')
            self.assertEqual(overridden['sources']['deployment']['values']['arguments']['command_enabled'],False)
            self.assertNotEqual(snapshot['sha256'],overridden['sha256'])
            self.assertIn('urdf',overridden['sources'])
            self.assertIn('display_urdf',overridden['sources'])


class DeploymentTests(unittest.TestCase):
    def test_target_brake_lead_recipe_rejects_the_retired_override(self):
        recipe=json.loads((package_share("snowbot_bringup")/'config/deployments/snowbot_vr_real.json').read_text())
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'deploy.json'
            for lead in (.12, .1, '0.08',0., -1., .4, 1., 1e308, 1e-12, float('nan'), float('inf'), True, 'bad'):
                recipe['arguments']['target_stream_brake_lead_s']=lead
                path.write_text(json.dumps(recipe))
                with self.subTest(lead=lead),self.assertRaisesRegex(ValueError,'BRAKE_LEAD_MOVED'):
                    load_deployment(path,SHARE)
            recipe['arguments']['target_stream_brake_lead_s']=.12
            recipe['arguments']['position_protocol']='session_v1'
            path.write_text(json.dumps(recipe))
            with self.assertRaisesRegex(ValueError,'POSITION_PROTOCOL_REMOVED'):
                load_deployment(path,SHARE)

    def test_target_dynamic_limits_accept_numeric_recipe_and_reject_invalid_or_wrong_mode(self):
        recipe=json.loads((package_share("snowbot_bringup")/'config/deployments/snowbot_vr_real.json').read_text())
        recipe['arguments'].update(trajectory_max_velocity=.4, trajectory_max_acceleration=.8,
                                   trajectory_max_jerk=259.2)
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'deploy.json'
            path.write_text(json.dumps(recipe))
            _,args=load_deployment(path,SHARE)
            self.assertEqual(args['trajectory_max_velocity'],'0.4')
            self.assertEqual(args['trajectory_max_acceleration'],'0.8')
            self.assertEqual(args['trajectory_max_jerk'],'259.2')
            for key in ('trajectory_max_velocity','trajectory_max_acceleration','trajectory_max_jerk'):
                for invalid in (0.,-1.,float('nan'),float('inf'),True,'bad',None,[],10**400):
                    bad=json.loads(json.dumps(recipe));bad['arguments'][key]=invalid
                    path.write_text(json.dumps(bad))
                    with self.subTest(key=key,invalid=invalid),self.assertRaisesRegex(ValueError,'TARGET_DYNAMIC_LIMIT'):
                        load_deployment(path,SHARE)
            for key,value in (('runtime_mode','simulation'),):
                bad=json.loads(json.dumps(recipe));bad['arguments'][key]=value
                path.write_text(json.dumps(bad))
                with self.assertRaisesRegex(ValueError,'TARGET_LIMITS_REQUIRE_TARGET_STREAM'):
                    load_deployment(path,SHARE)

    def test_target_recipe_rejects_retired_curve_options_and_accepts_explicit_impedance(self):
        recipe=json.loads((package_share("snowbot_bringup")/'config/deployments/snowbot_vr_real.json').read_text())
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'deploy.json';path.write_text(json.dumps(recipe))
            _,args=load_deployment(path,SHARE)
            self.assertNotIn('position_protocol',args)
            self.assertEqual(args['command_enabled'],'false')
            for key,value in (('trajectory_lead_ms',60.),('trajectory_fast_receipt',True),
                              ('position_protocol','session_v2')):
                bad=json.loads(json.dumps(recipe));bad['arguments'][key]=value
                path.write_text(json.dumps(bad))
                with self.subTest(key=key),self.assertRaises(ValueError):load_deployment(path,SHARE)
    def test_protocol_selectors_are_rejected_and_execution_mode_remains_separate(self):
        recipe=dict(schema_version=2,robot_launch='snowbot_adapter.launch.py',arguments=dict(
            runtime_mode='real',device_backend='snowbot_vendor'))
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'deploy.json'
            for protocol in ('legacy_guard','session_v1','session_v2','session_v3'):
                recipe['arguments']['position_protocol']=protocol
                path.write_text(json.dumps(recipe))
                with self.subTest(protocol=protocol),self.assertRaisesRegex(ValueError,'POSITION_PROTOCOL_REMOVED'):
                    load_deployment(path,SHARE)
            del recipe['arguments']['position_protocol']
            recipe['arguments']['execution_mode']='impedance'
            path.write_text(json.dumps(recipe))
            self.assertEqual(load_deployment(path,SHARE)[1]['execution_mode'],'impedance')

    def test_gripper_loader_rejects_cross_mode_configuration(self):
        from snowbot_hardware.parameters import load_mode_config
        for mode in ('sim','real'):
            _,args=load_deployment(package_share("snowbot_bringup")/f'config/deployments/snowbot_vr_{mode}.json',SHARE)
            other='real' if mode=='sim' else 'simulation'
            wrong=load_mode_config(args['gripper_config'],other)
            with tempfile.TemporaryDirectory() as tmp:
                path=Path(tmp)/'wrong-mode-grippers.json';path.write_text(json.dumps(wrong))
                with self.assertRaisesRegex(ValueError,'SNOWBOT_CONFIG_MODE_MISMATCH'):
                    load_mode_config(path,args['runtime_mode'])

    def test_position_recipes_share_model_and_reject_retired_contract_switch(self):
        _,sim=load_deployment(package_share("snowbot_bringup")/'config/deployments/snowbot_vr_sim.json',SHARE)
        _,real=load_deployment(package_share("snowbot_bringup")/'config/deployments/snowbot_vr_real.json',SHARE)
        from bindu_runtime.robot_setup import RobotSetup
        sim_robot,real_robot=(RobotSetup.load(recipe['robot_config']) for recipe in (sim,real))
        self.assertEqual(sim_robot.urdf,real_robot.urdf)
        self.assertEqual(sim_robot.display_urdf,real_robot.display_urdf)
        self.assertEqual(sim_robot.profile_path,real_robot.profile_path)
        self.assertEqual(real['pico_enabled'],'true')
        self.assertEqual(real['command_enabled'],'false')
        recipe=json.loads((package_share("snowbot_bringup")/'config/deployments/snowbot_vr_sim.json').read_text())
        for key,value in (('position_contract','true'),('position_contract',True),('position_contract',False)):
            bad=json.loads(json.dumps(recipe));bad['arguments'][key]=value
            with tempfile.TemporaryDirectory() as tmp:
                path=Path(tmp)/'bad.json';path.write_text(json.dumps(bad))
                with self.assertRaisesRegex(ValueError,'POSITION_CONTRACT_REMOVED'):load_deployment(path,SHARE)

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
            'profile':'@runtime/config/examples/snowbot/snowbot_shovel_sim.json',
            'bimanual_config':'@runtime/config/examples/snowbot/bimanual_snowbot_shovel.json'}}
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'legacy.json';path.write_text(json.dumps(recipe))
            launch,args=load_deployment(path,SHARE)
        self.assertEqual(launch,'snowbot_sim.launch.py')
        self.assertEqual(args['initialize_tool_pose'],'false')
        self.assertTrue(Path(args['profile']).is_file())
        self.assertTrue(Path(args['bimanual_config']).is_file())

    def test_pico_recipe_is_explicit_simulation_and_not_default(self):
        launch,args=load_deployment(package_share("snowbot_bringup")/'config/examples/snowbot/teleop_pico_snowbot_sim.json',SHARE)
        self.assertEqual(launch,'robot_teleop.launch.py')
        self.assertEqual(args['runtime_mode'],'simulation')
        self.assertEqual(args['pico_enabled'],'true')
        self.assertEqual(args['tool_input_source'],'vr')
        _,default=load_deployment(SHARE/'config/deployments/teleop_robot_deployment.json',SHARE)
        self.assertNotIn('pico_enabled',default)

    def test_pico_recipe_rejects_wrong_source(self):
        recipe=json.loads((package_share("snowbot_bringup")/'config/examples/snowbot/teleop_pico_snowbot_sim.json').read_text())
        for key,value in [('tool_input_source','tool_pose')]:
            changed=json.loads(json.dumps(recipe));changed['arguments'][key]=value
            with tempfile.TemporaryDirectory() as tmp:
                path=Path(tmp)/'deploy.json';path.write_text(json.dumps(changed))
                with self.assertRaisesRegex(ValueError,'PICO_REQUIRES'):load_deployment(path,SHARE)

    def test_demo_is_explicit_and_rejects_external_or_missing_pico(self):
        recipe=json.loads((package_share("snowbot_bringup")/'config/examples/snowbot/teleop_pico_snowbot_sim.json').read_text())
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
