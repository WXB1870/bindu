"""Analytical checks for mass, wrench balance and the released shaft axis."""
from bindu_contracts.resources import package_share
from tests.support.paths import ROOT
import copy
import json
import os
from pathlib import Path
import sys
import unittest
import time
import tempfile
import hashlib
from types import SimpleNamespace

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(ROOT))
from tools.simulation.shovel_physics import wooden_parts, compound_mass, grip_wrench, internal_wrench, MeasuredJointVelocity, load_motion, joint_drive_parameters
from tools.simulation.run_shovel_physics import contact_wrench, validate_grasp_config
from tools.simulation.shovel_control import GripLoadObserver, ControlClient


class ShovelPhysicsTests(unittest.TestCase):
    def test_feedback_wrapper_selects_physics_test_without_inheriting_ros_environment(self):
        import subprocess
        with tempfile.TemporaryDirectory() as directory:
            directory=Path(directory);sdk=directory/'sdk';sdk.mkdir()
            (sdk/'exts/isaacsim.pip.nv/pip_prebundle/nvidia/test/lib').mkdir(parents=True)
            capture=directory/'arguments';environment=directory/'environment'
            launcher=sdk/'python.sh'
            launcher.write_text('#!/usr/bin/env bash\n'
                'printf "%s\\n" "$@" > "$FAKE_ISAAC_ARGUMENTS"\n'
                'printf "%s\\n" "${ROS_DISTRO-unset}" "${AMENT_PREFIX_PATH-unset}" "$PYTHONPATH" > "$FAKE_ISAAC_ENVIRONMENT"\n')
            launcher.chmod(0o755)
            env=dict(os.environ,ISAAC_SIM_PATH=str(sdk),BINDU_ISAAC_STATE=str(directory/'state'),
                FAKE_ISAAC_ARGUMENTS=str(capture),FAKE_ISAAC_ENVIRONMENT=str(environment),
                ROS_DISTRO='unexpected_real_environment',AMENT_PREFIX_PATH='/unexpected/install',
                PYTHONPATH='/unexpected/python')
            subprocess.run(['bash',str(ROOT/'tools/run_shovel_physics.sh'),'--task-feedback',
                'settings.json','--output','results'],env=env,check=True)
            arguments=capture.read_text().splitlines()
            self.assertEqual(arguments[:4],['--no-ros-env',str(ROOT/'tests/physics/validate_shovel_isaac.py'),
                '--task-feedback-comparison','settings.json'])
            self.assertIn('--/isaac/startup/ros_bridge_extension=',arguments)
            self.assertEqual(environment.read_text().splitlines(),['unset','unset',str(ROOT)])

    def test_drive_calibration_preserves_defaults_and_urdf_effort_ceiling(self):
        names = ['arm', 'waist']; limits = {n:dict(effort='80') for n in names}
        cfg = dict(joint_stiffness_nm_rad=100000., joint_damping_nm_s_rad=2000.)
        kp, kd, effort = joint_drive_parameters(names, limits, cfg)
        np.testing.assert_array_equal(kp, [100000.,100000.])
        np.testing.assert_array_equal(kd, [2000.,2000.])
        np.testing.assert_array_equal(effort, [80.,80.])
        cfg['joint_drive_overrides'] = {'waist':dict(stiffness_nm_rad=80000., damping_nm_s_rad=4000., max_effort_nm=60.)}
        kp, kd, effort = joint_drive_parameters(names, limits, cfg)
        np.testing.assert_array_equal(kp, [100000.,80000.])
        np.testing.assert_array_equal(kd, [2000.,4000.])
        np.testing.assert_array_equal(effort, [80.,60.])
        for override in ({'missing':{}}, {'arm':{'velocity':.5}},
                         {'arm':{'damping_nm_s_rad':float('nan')}},
                         {'arm':{'stiffness_nm_rad':0.}}, {'arm':{'max_effort_nm':81.}}):
            cfg['joint_drive_overrides'] = override
            with self.assertRaisesRegex(ValueError, 'PHYSICS_DRIVE_'):
                joint_drive_parameters(names, limits, cfg)

    def test_contact_parameters_reject_negative_effort_or_malformed_joints(self):
        cfg = copy.deepcopy(self.cfg)
        validate_grasp_config(cfg)
        for key,value in [('axial_effort_nm',-1.),('max_effort_nm',0.),('contact_offset_m',.1)]:
            bad=copy.deepcopy(cfg);bad['contact_grasp'][key]=value
            with self.assertRaisesRegex(ValueError,'INVALID_CONTACT_GRASP'):
                validate_grasp_config(bad)
        cfg['contact_grasp']['closed_positions_rad']['arm_l_joint_1']=.2
        with self.assertRaisesRegex(ValueError,'INVALID_CONTACT_GRASP_JOINTS'):
            validate_grasp_config(cfg)

    def test_old_motion_cannot_silently_load_a_changed_grasp_config(self):
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory);config=p/'config.json';config.write_text('{}')
            digest=hashlib.sha256(config.read_bytes()).hexdigest()
            motion=dict(schema=2,source_sha256={},display_urdf=str(config),display_sha256=digest,
                tool_input_path=str(config),tool_input_sha256=digest,config_path=str(config),config_sha256=digest,
                samples=[dict(t=0.,q={'a':0.}),dict(t=1.,q={'a':0.})],duration_s=1.)
            path=p/'motion.json';path.write_text(json.dumps(motion));load_motion(path)
            config.write_text('{"changed":true}')
            # Keep the display/input digests current to isolate the config guard.
            motion.update(display_sha256=hashlib.sha256(config.read_bytes()).hexdigest(),tool_input_sha256=hashlib.sha256(config.read_bytes()).hexdigest())
            path.write_text(json.dumps(motion))
            with self.assertRaisesRegex(ValueError,'MOTION_SOURCE_IDENTITY'):
                load_motion(path)
    def test_grasp_center_clears_actual_jaws_and_old_center_intersects(self):
        import coal as fcl
        rod = fcl.Cylinder(.018, .4)
        for short, y in (('l', .0085), ('r', -.0085)):
            for link in (7, 8):
                mesh = fcl.MeshLoader().load(str(package_share('snowbot_description')/f'meshes/snowbot/arm_{short}_link_{link}.STL'),np.ones(3))
                jaw = fcl.Transform3s(np.eye(3),np.array([.122 if link==8 else 0.,0.,0.]))
                for center, expected in (([.151,0,0], True), ([.195,y,0], False)):
                    result = fcl.CollisionResult()
                    fcl.collide(mesh,jaw,rod,fcl.Transform3s(np.eye(3),np.array(center)),fcl.CollisionRequest(),result)
                    self.assertEqual(result.isCollision(),expected,(short,link,center))
    def test_velocity_uses_measured_positions_and_actual_sample_time(self):
        estimator=MeasuredJointVelocity()
        np.testing.assert_allclose(estimator.update(0.,[1.,2.],[.1,.2]),[.1,.2])
        np.testing.assert_allclose(estimator.update(.02,[1.002,1.996],[99.,99.]),[.1,-.2])
        np.testing.assert_allclose(estimator.update(.03,[1.002,1.996],[99.,99.]),[0.,0.])
        with self.assertRaisesRegex(ValueError,'ENCODER_TIME_OR_LAYOUT'):
            estimator.update(.03,[1.,2.],[0.,0.])
        with self.assertRaisesRegex(ValueError,'ENCODER_TIME_OR_LAYOUT'):
            estimator.update(.2,[1.,2.],[0.,0.])

    def setUp(self):
        config = json.loads((package_share("snowbot_bringup")/'config/robots/snowbot/bimanual_snowbot_shovel.json').read_text())
        self.cfg = json.loads((package_share("snowbot_bringup")/'config/robots/snowbot/snowbot_shovel_physics.json').read_text())
        self.recipe = copy.deepcopy(config['demo'])
        self.recipe['left_grip_in_tool_m'] = self.recipe['pivot_in_tool_m']
        self.cfg['shaft_axis_in_tool'] = np.array([0., 0., 1.])

    def test_disjoint_tray_volume_and_density_scaling(self):
        parts = wooden_parts(self.recipe, self.cfg['material'])
        mass, com, inertia = compound_mass(parts)
        x, y, z = self.recipe['pan_size_m']
        w = self.cfg['material']['tray_wall_m']
        volume = x*y*z-(x-w)*(y-2*w)*(z-w)
        volume += np.pi*self.recipe['shaft_radius_m']**2*1.35
        self.assertAlmostEqual(mass, volume*650)
        self.assertGreater(np.linalg.eigvalsh(inertia).min(), 0)
        heavy = wooden_parts(self.recipe, dict(self.cfg['material'], density_kg_m3=1300))
        m2, c2, i2 = compound_mass(heavy)
        self.assertAlmostEqual(m2, 2*mass)
        np.testing.assert_allclose(c2, com)
        np.testing.assert_allclose(i2, 2*inertia)

    def test_deployed_short_tool_has_only_its_shaft_mass_and_contact_shape(self):
        cfg=json.loads((package_share("snowbot_bringup")/'config/robots/snowbot/bimanual_snowbot_real.json').read_text())
        recipe=cfg['demo'];mount=np.array(cfg['left_to_tool']).reshape(4,4)
        recipe['left_grip_in_tool_m']=(np.linalg.inv(mount) @ np.r_[recipe['grip_offset_m'],1.])[:3].tolist()
        recipe['pan_size_m']=[100.,100.,100.]
        parts=wooden_parts(recipe,self.cfg['material'])
        self.assertEqual([p['name'] for p in parts],['Shaft'])
        self.assertAlmostEqual(parts[0]['length'],.54)
        mass,_,inertia=compound_mass(parts)
        self.assertAlmostEqual(mass,650*np.pi*.018**2*.54)
        self.assertGreater(np.linalg.eigvalsh(inertia).min(),0.)

    def test_parallel_axis_matches_two_point_masses(self):
        parts = [dict(mass=2., center=np.array([x, 0., 0.]), inertia=np.eye(3)*.1) for x in (-1., 1.)]
        mass, com, inertia = compound_mass(parts)
        self.assertEqual(mass, 4)
        np.testing.assert_allclose(com, 0)
        np.testing.assert_allclose(inertia, np.diag([.2, 4.2, 4.2]))

    def test_axial_pair_has_zero_external_wrench_and_known_squeeze(self):
        points = np.array([[0., 0., 0.], [.34, 0., 0.]])
        f = np.array([[100., 0., 10.], [-100., 0., 10.]])
        d = internal_wrench(f, np.zeros((2, 3)), points, np.array([.17, 0., 0.]))
        self.assertAlmostEqual(d['axial_squeeze_n'], 100)
        np.testing.assert_allclose(d['net_force'], [0, 0, 20])
        np.testing.assert_allclose(d['net_torque'], 0, atol=1e-12)
        np.testing.assert_allclose(d['wrenches'][:, :3], [[100, 0, 0], [-100, 0, 0]], atol=1e-12)
        self.assertLess(d['null_residual'], 1e-12)

    def test_general_internal_wrench_is_self_equilibrating(self):
        rng = np.random.default_rng(7)
        for _ in range(30):
            points, com = rng.normal(size=(2,3)), rng.normal(size=3)
            d = internal_wrench(rng.normal(size=(2,3)), rng.normal(size=(2,3)), points, com)
            w = d['wrenches']
            np.testing.assert_allclose(w[:, :3].sum(axis=0), 0, atol=1e-12)
            np.testing.assert_allclose(w[:, 3:].sum(axis=0)+np.cross(points-com, w[:, :3]).sum(axis=0), 0, atol=1e-12)

    def test_released_rotation_transmits_no_axial_couple(self):
        hand, tool = np.eye(4), np.eye(4)
        hand[:3,:3] = Rotation.from_euler('z', 70, degrees=True).as_matrix()
        velocity = np.array([0., 0., 0., 0., 0., 2.])
        args = (hand, tool, velocity, np.zeros(6), np.zeros(3), np.zeros(3),
                np.zeros(3), np.zeros(3), np.eye(3), self.cfg)
        force, torque, point, error = grip_wrench(*args, axial=True)
        np.testing.assert_allclose(torque, 0, atol=1e-12)
        self.assertGreater(np.linalg.norm(grip_wrench(*args, axial=False)[1]), 60)
        hand[:3,:3] = Rotation.from_euler('x', .01).as_matrix()
        self.assertGreater(np.linalg.norm(grip_wrench(*args, axial=True)[1]), .4)

    def test_internal_projection_respects_released_shaft_couple(self):
        rng=np.random.default_rng(21)
        for _ in range(20):
            axis=rng.normal(size=3);axis/=np.linalg.norm(axis)
            forces,moments,points=rng.normal(size=(3,2,3))
            moments[1]-=axis*(moments[1] @ axis)
            com=rng.normal(size=3)
            w=internal_wrench(forces,moments,points,com,released_axis=axis)['wrenches']
            self.assertAlmostEqual(w[1,3:] @ axis,0.,places=12)
            np.testing.assert_allclose(w[:,:3].sum(axis=0),0.,atol=1e-12)
            np.testing.assert_allclose(w[:,3:].sum(axis=0)+np.cross(points-com,w[:,:3]).sum(axis=0),0.,atol=1e-12)

    def test_one_mm_spring_deflection_produces_twenty_newtons(self):
        hand, tool = np.eye(4), np.eye(4)
        hand[0,3] = .001
        f, tau, point, error = grip_wrench(hand, tool, np.zeros(6), np.zeros(6),
            np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3), np.eye(3), self.cfg)
        np.testing.assert_allclose(f, [20., 0., 0.])
        np.testing.assert_allclose(tau, 0)
        np.testing.assert_allclose(point, [.0005, 0, 0])
        self.assertAlmostEqual(error, .001)

    def test_invalid_material_and_degenerate_grips_fail(self):
        for density in (0, -1, float('nan')):
            with self.assertRaisesRegex(ValueError, 'INVALID_WOOD'):
                wooden_parts(self.recipe, dict(self.cfg['material'], density_kg_m3=density))
        with self.assertRaisesRegex(ValueError, 'INVALID_GRASP'):
            internal_wrench(np.zeros((2,3)), np.zeros((2,3)), np.zeros((2,3)), np.zeros(3))

    def test_contact_wrench_includes_friction_and_excludes_unused_slots(self):
        class Contacts:
            def get_contact_force_data(self, dt):
                return (np.array([[np.nan],[10.],[np.nan]]),
                        np.array([[np.nan]*3,[.2,0.,0.],[np.nan]*3]),
                        np.array([[np.nan]*3,[0.,0.,1.],[np.nan]*3]),
                        None,np.array([[1]]),np.array([[1]]))
            def get_friction_data(self, dt):
                return (np.array([[np.nan]*3,[-2.,0.,0.],[np.nan]*3]),
                        np.array([[np.nan]*3,[0.,.1,0.],[np.nan]*3]),
                        np.array([[1]]),np.array([[1]]))
        force, moment, normal, friction = contact_wrench(Contacts(),np.zeros(3),.001)
        np.testing.assert_allclose(force,[-2.,0.,10.])
        np.testing.assert_allclose(moment,[0.,-2.,.2])
        np.testing.assert_allclose(normal,[0.,0.,10.])
        np.testing.assert_allclose(friction,[-2.,0.,0.])

    def test_flange_wrench_recovers_static_tool_deflection_in_rotated_frame(self):
        config = json.loads((package_share("snowbot_bringup")/'config/robots/snowbot/bimanual_snowbot_shovel.json').read_text())
        parameters = dict(self.cfg, max_age_s=.2)
        observer = GripLoadObserver(parameters,config)
        hand=np.eye(4);hand[:3,:3]=Rotation.from_euler('xyz',[.3,-.2,.8]).as_matrix();hand[:3,3]=[.1,.2,.3]
        tool=hand @ observer.mount
        tool[:3,:3]=Rotation.from_rotvec([-.02,.01,.005]).as_matrix() @ tool[:3,:3]
        hp=hand[:3,3]+hand[:3,:3] @ observer.offset
        tool[:3,3]=hp-np.array([.001,-.002,.0005])-tool[:3,:3] @ observer.tool_grip
        f,tau,point,_=grip_wrench(hand,tool,np.zeros(6),np.zeros(6),np.zeros(3),np.zeros(3),
            observer.offset,observer.tool_grip,observer.mount[:3,:3],parameters)
        on_hand=-f;moment=-tau+np.cross(point-hand[:3,3],on_hand)
        sample=dict(stamp=1.,frame_id=config['left_frame'],
            wrench=np.r_[hand[:3,:3].T @ on_hand,hand[:3,:3].T @ moment])
        estimated=observer.estimate(hand,sample,1.)
        np.testing.assert_allclose(estimated,tool,atol=1e-12)
        with self.assertRaisesRegex(ValueError,'GRIP_LOAD_STALE_OR_FRAME'):
            observer.estimate(hand,sample,1.21)
        with self.assertRaisesRegex(ValueError,'GRIP_LOAD_STALE_OR_FRAME'):
            observer.estimate(hand,dict(sample,frame_id='wrong'),1.)
        with self.assertRaisesRegex(ValueError,'GRIP_LOAD_INVALID'):
            observer.estimate(hand,dict(sample,wrench=[float('nan')]*6),1.)

    def test_reply_deadline_distinguishes_completed_worker_from_paused_gui(self):
        for worker_delay in (.005,.3):
            with self.subTest(worker_delay=worker_delay):
                read_fd,write_fd=os.pipe()
                with os.fdopen(read_fd,'rb',buffering=0) as stream:
                    client=ControlClient.__new__(ControlClient)
                    client.proc=SimpleNamespace(poll=lambda:None,stdout=stream)
                    sent=time.monotonic()-.31
                    client.pending=(1,sent,1.);client.buffer=b'';client.records=[]
                    os.write(write_fd,(json.dumps(dict(seq=1,completed_at_wall=sent+worker_delay))+'\n').encode())
                    os.close(write_fd)
                    if worker_delay < .23:
                        reply=client.poll(1.001)
                        self.assertGreater(reply['ipc_wall_s'],.3)
                        self.assertAlmostEqual(reply['worker_wall_s'],.005,places=7)
                    else:
                        with self.assertRaisesRegex(RuntimeError,'PHYSICS_CONTROL_REPLY_TIMEOUT'):
                            client.poll(1.001)


if __name__ == '__main__':
    unittest.main()
