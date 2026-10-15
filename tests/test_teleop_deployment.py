import json
import tempfile
import unittest
from pathlib import Path
from bindu_runtime.teleop_deployment import load_deployment

ROOT=Path(__file__).resolve().parents[1]
SHARE=ROOT/'src/integration/bindu_runtime'


class DeploymentTests(unittest.TestCase):
    def test_default_recipe_has_no_startup_motion_or_vr_requirement(self):
        launch,args=load_deployment(SHARE/'config/teleop_snowbot_deployment.json',SHARE)
        self.assertEqual(launch,'snowbot_sim.launch.py')
        self.assertEqual(args['initialize_tool_pose'],'false')
        self.assertEqual(args['vr_enabled'],'false')
        self.assertEqual(args['tool_input_source'],'tool_pose')
        self.assertEqual(args['description_enabled'],'false')
        self.assertTrue(Path(args['profile']).is_file())

    def test_pico_recipe_is_explicit_simulation_and_not_default(self):
        launch,args=load_deployment(SHARE/'config/teleop_pico_snowbot_sim.json',SHARE)
        self.assertEqual(launch,'robot_teleop.launch.py')
        self.assertEqual(args['runtime_mode'],'simulation')
        self.assertEqual(args['pico_enabled'],'true')
        self.assertEqual(args['tool_input_source'],'vr')
        _,default=load_deployment(SHARE/'config/teleop_robot_deployment.json',SHARE)
        self.assertNotIn('pico_enabled',default)

    def test_pico_rejects_wrong_source_and_external_actuator_backend(self):
        recipe=json.loads((SHARE/'config/teleop_pico_snowbot_sim.json').read_text())
        for key,value in [('tool_input_source','tool_pose'),('device_backend','external_simulation')]:
            changed=json.loads(json.dumps(recipe));changed['arguments'][key]=value
            with tempfile.TemporaryDirectory() as tmp:
                path=Path(tmp)/'deploy.json';path.write_text(json.dumps(changed))
                with self.assertRaisesRegex(ValueError,'PICO_REQUIRES'):load_deployment(path,SHARE)

    def test_unknown_hardware_auto_pose_and_nonteleop_launch_fail_closed(self):
        recipe=json.loads((SHARE/'config/teleop_snowbot_deployment.json').read_text())
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
