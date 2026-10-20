"""Navigation-only dependency boundary, source-time checks and map evidence."""
import copy
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

from tests.support.paths import ROOT
from tools.deployment import orin_navigation as deploy
from bindu_navigation.orin import InputWindow, site_errors, map_errors, MOTION_BLOCK
from bindu_navigation.scan_grid import ObservedGrid


def configured(route='2d'):
    site = json.loads(deploy.TEMPLATE.read_text())
    site.update(route=route, scan_topic='/measured/scan',odom_topic='/measured/odom',
                base_from_scan=[0.,0.,0.,0.,0.,0.,1.])
    site['frames']['scan'] = 'laser'
    site['laser'] = dict(min_range=.1,max_range=10.)
    site['tf_authorities'] = dict(odom_to_base='wheel_driver',base_to_scan='calibrated_model')
    site['map']['prefix'] = '/tmp/new-map-batch'
    if route == 'lio':
        site.update(points_topic='/measured/points',imu_topic='/measured/imu',input_type='timed_cloud',
                    imu_from_lidar=[0.,0.,0.,0.,0.,0.,1.],imu_from_base=[0.,0.,0.,0.,0.,0.,1.])
        site['frames'].update(lidar='laser',imu='imu')
        site['tf_authorities']['odom_to_base'] = 'bindu_lio'
        site['scan_projection'].update(min_height=.1,max_height=.8,min_range=.1,max_range=10.)
    return site


class OrinNavigationTests(unittest.TestCase):
    def test_only_target_os_architecture_and_python(self):
        self.assertFalse(deploy.target_errors(dict(ID='ubuntu',VERSION_ID='22.04'),'aarch64',(3,10)))
        for osinfo,arch,version in [(dict(ID='ubuntu',VERSION_ID='24.04'),'aarch64',(3,10)),
                                    (dict(ID='ubuntu',VERSION_ID='22.04'),'x86_64',(3,10)),
                                    (dict(ID='ubuntu',VERSION_ID='22.04'),'aarch64',(3,12))]:
            self.assertTrue(deploy.target_errors(osinfo,arch,version))

    def test_template_does_not_guess_route_or_hardware(self):
        site = json.loads(deploy.TEMPLATE.read_text())
        self.assertIsNone(site['route']); self.assertIsNone(site['base_from_scan'])
        self.assertTrue(site_errors(site)); self.assertIn(MOTION_BLOCK,site_errors(site,'navigation'))
        self.assertEqual(site['base']['adapter'],'unimplemented')
        self.assertEqual(site['base']['model'],'agilex_tracer_2_0')

    def test_sensor_routes_and_explicit_calibration(self):
        for route in ('2d','lio'):
            site = configured(route)
            self.assertEqual(site_errors(site,'sensors'),[])
            site['base_from_scan'] = [0.]*7
            self.assertTrue(any('SCAN_CALIBRATION' in s for s in site_errors(site)))
        site = configured('lio'); site['imu_from_lidar'][0] = .2
        self.assertTrue(any('EXTRINSICS_INCONSISTENT' in e for e in site_errors(site)))
        site = configured('lio'); site['input_type'] = 'untimed'
        self.assertTrue(any('TIMED_CLOUD_REQUIRED' in e for e in site_errors(site)))
        site = configured('lio'); site['tf_authorities']['odom_to_base'] = 'wheel_driver'
        self.assertTrue(any('LIO_ODOM_AUTHORITY' in e for e in site_errors(site)))

    def test_no_config_value_enables_motion(self):
        site = configured(); site['base']['adapter'] = 'pretend_tracer_adapter'
        site['limits'] = {key:.1 for key in site['limits']}; site['footprint'] = [[0.,0.],[1.,0.],[0.,1.]]
        self.assertIn(MOTION_BLOCK,site_errors(site,'navigation',files=False))
        for command in ('manual','navigate','cancel','stop'):
            with patch.object(deploy,'run_env') as run:
                self.assertEqual(deploy.main([command]),2); run.assert_not_called()

    def test_saved_map_references_and_pcd_checked(self):
        with tempfile.TemporaryDirectory() as td:
            folder = Path(td); path = folder/'map.yaml'; site = configured()
            site['map']['yaml'] = str(path)
            self.assertTrue(map_errors(site))
            path.write_text('image: map.pgm\nresolution: 0.05\norigin: [0,0,0]\nfree_thresh: 0.2\noccupied_thresh: 0.65\n')
            self.assertTrue(any('MAP_IMAGE' in e for e in map_errors(site)))
            (folder/'map.pgm').write_bytes(b'P5\n1 1\n255\n\xff')
            self.assertEqual(map_errors(site),[])
            site['route'] = 'lio'; self.assertTrue(any('PCD_MAP_REQUIRED' in e for e in map_errors(site)))

    def test_minimal_manifest_graph_excludes_vr_ik_vla(self):
        from catkin_pkg.package import parse_package
        from catkin_pkg.condition import evaluate_condition
        package = parse_package(str(ROOT/'src/integration/bindu_runtime/package.xml'))
        enabled = lambda env:{d.name for d in package.exec_depends if not d.condition or evaluate_condition(d.condition,env)}
        minimal = enabled(dict(BINDU_NAVIGATION_ONLY='1'))
        for key in ('bindu_kinematics','bindu_teleoperation','bindu_vla','bindu_tasks','python3-zmq'):
            self.assertNotIn(key,minimal); self.assertIn(key,enabled({}))
        self.assertTrue({'bindu_execution','bindu_navigation','bindu_hardware'} <= minimal)
        two = deploy.build_command(Path('/tmp/ws'),'2d'); lio = deploy.build_command(Path('/tmp/ws'),'lio')
        self.assertNotIn('bindu_lio',two); self.assertIn('bindu_lio',lio)

    def test_environment_script_preserves_dds_but_clears_other_abi(self):
        script = deploy.env_script(Path("/tmp/ws with ' quotes"),'2d')
        self.assertIn('BINDU_NAVIGATION_ONLY=1',script)
        self.assertNotIn('venv/',script); self.assertNotIn('pinocchio',script)
        subprocess.run(['bash','-n'],input=script,text=True,check=True)
        self.assertNotIn('unset ROS_DOMAIN_ID',script)

    def test_environment_loads_only_configured_driver_with_literal_path(self):
        with tempfile.TemporaryDirectory(prefix="orin ' $ ") as td:
            workspace = Path(td)
            driver = workspace/'sensor $(literal)'/'local_setup.bash'; driver.parent.mkdir()
            driver.write_text('export SELECTED_SENSOR_DRIVER=1\n')
            (workspace/'site.json').write_text(json.dumps(dict(driver_setup=str(driver))))
            script = deploy.env_script(workspace,'2d').replace('source /opt/ros/humble/setup.bash || return', ':')
            env_file = workspace/'env.sh'; env_file.write_text(script)
            result = subprocess.check_output(['bash','--noprofile','--norc','-ec',
                'source "$1"; /usr/bin/python3 -c \'import os; print(os.environ["SELECTED_SENSOR_DRIVER"])\'',
                'test',str(env_file)],text=True)
            self.assertEqual(result.strip(),'1')

    def test_orin_launch_never_selects_simulation_or_starts_placeholder_base(self):
        import importlib.util
        from launch import LaunchContext
        file = ROOT/'src/integration/bindu_runtime/launch/navigation_orin.launch.py'
        spec = importlib.util.spec_from_file_location('orin_launch',file)
        module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
        with tempfile.TemporaryDirectory() as td:
            path = Path(td)/'site.json'; path.write_text(json.dumps(configured()))
            context = LaunchContext(); context.launch_configurations.update(site=str(path),phase='navigation')
            with self.assertRaisesRegex(ValueError,'BASE_ADAPTER_REQUIRED'): module.assemble(context)

    def test_rviz_uses_site_topics_and_public_contract(self):
        site = configured(); config = json.loads(deploy.rviz_text(site))['Visualization Manager']
        text = json.dumps(config)
        self.assertIn('/measured/scan',text); self.assertIn('/measured/odom',text)
        self.assertNotIn('SetGoal',text); self.assertIn('/bindu_navigation/initialpose',text)

    def test_source_timestamp_not_receipt_time_and_stream_loss(self):
        window = InputWindow(.3,.05)
        window.observe('scan',10.,10.); window.observe('scan',10.1,10.1)
        self.assertEqual(window.finish(['scan'],10.2),[])
        self.assertTrue(any('STREAM_STALE' in s for s in window.finish(['scan'],10.5)))
        frozen = InputWindow(.3,.05)
        frozen.observe('odom',10.,10.); frozen.observe('odom',10.,10.1)
        self.assertTrue(any('NO_ADVANCING_DATA' in s for s in frozen.finish(['odom'],10.1)))
        for stamp in (0.,9.,11.):
            with self.subTest(stamp=stamp):
                stale = InputWindow(.3,.05); stale.observe('scan',stamp,10.)
                self.assertTrue(stale.finish(['scan'],10.))

    def test_restart_regression_latches_until_new_precheck(self):
        window = InputWindow(.3,.05)
        for t in (10.,9.9,10.1): window.observe('odom',t,t)
        self.assertTrue(any('REGRESSION' in s for s in window.finish(['odom'],10.1)))
        restarted = InputWindow(.3,.05)
        for t in (10.2,10.3): restarted.observe('odom',t,t)
        self.assertEqual(restarted.finish(['odom'],10.3),[])

    def test_grid_never_invents_unseen_free_space_and_hit_is_conservative(self):
        grid = ObservedGrid(.1,.1,100)
        grid.add([0.,0.],[[1.,0.]])
        self.assertEqual(grid.cells[0,0],0); self.assertEqual(grid.cells[10,0],100)
        self.assertEqual(grid.cells[9,1],100); self.assertNotIn((0,2),grid.cells)
        w,h,origin,data = grid.raster()
        self.assertIn(-1,data); self.assertIn(0,data); self.assertIn(100,data)
        self.assertEqual(len(data),w*h)
        grid.add([1.,0.],[[2.,0.]])
        self.assertEqual(grid.cells[10,0],100)  # Free rays cannot erase measured obstacle.

    def test_grid_rejects_capacity_without_partial_mutation(self):
        grid = ObservedGrid(.1,.1,16); grid.add([0.,0.],[[.5,0.]])
        before = copy.deepcopy(grid.__dict__)
        with self.assertRaisesRegex(ValueError,'GRID_BOUNDS_EXCEEDED'): grid.add([0.,0.],[[20.,0.]])
        self.assertEqual(before,grid.__dict__)


class HumbleSessionTests(unittest.TestCase):
    def test_versioned_nav2_parameters_and_result_schema(self):
        from types import SimpleNamespace
        from bindu_runtime.nav2_session import parameters, set_result_error
        robot = json.loads((ROOT/'tests/config/navigation/navigation_g1_fixture.json').read_text())
        for distro in ('humble','jazzy'):
            cfg = parameters(robot,'/fixture','/fixture/session','/tmp/tree.xml',distro)
            controller = cfg['/fixture/session/controller_server']['ros__parameters']
            self.assertEqual(controller['progress_checker']['plugin'],'nav2_controller::PoseProgressChecker')
            if distro == 'humble':
                self.assertEqual(controller['progress_checker_plugin'],'progress_checker')
                self.assertNotIn('enable_stamped_cmd_vel',controller)
                self.assertEqual(cfg['/fixture/session/planner_server']['ros__parameters']['GridBased']['plugin'],'nav2_navfn_planner/NavfnPlanner')
            else:
                self.assertTrue(controller['enable_stamped_cmd_vel'])
                self.assertEqual(controller['progress_checker_plugins'],['progress_checker'])
        from nav2_msgs.action import NavigateToPose
        result = NavigateToPose.Result()
        set_result_error(result,900,'fixture failure')
        if hasattr(result,'error_code'): self.assertEqual(result.error_code,900)
