"""Ground and tool/body clearance regressions against the Snowbot CAD model."""
import copy
import json
from pathlib import Path
import sys
import unittest

import numpy as np
import pinocchio as pin

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
from plan_shovel_motion import collision_check, validate_recipe, target_at, grasp_mode_at, run_input_stream
from shovel_geometry import bin_boxes, check_dump
from scipy.spatial.transform import Rotation
from bindu_kinematics.tool_geometry import tracking_acceptance, coupling_metrics


class ShovelGeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config_path = ROOT/'src/integration/bindu_runtime/config/robots/snowbot/bimanual_snowbot_shovel.json'
        cls.config = json.loads(cls.config_path.read_text())
        cls.urdf = ROOT/'src/hardware/bindu_description/urdf/snowbot_display.urdf'
        cls.recipe = copy.deepcopy(cls.config['demo'])
        positions = cls.recipe['initial_positions_rad']
        model = pin.buildModelFromUrdf(str(cls.urdf))
        data = model.createData()
        q = pin.neutral(model)
        for name,value in positions.items():
            q[model.joints[model.getJointId(name)].idx_q] = value
        pin.framesForwardKinematics(model, data, q)
        flange = data.oMf[model.getFrameId(cls.config['left_frame'])].homogeneous.copy()
        tool = flange @ np.array(cls.config['left_to_tool']).reshape(4,4)
        cls.recipe['left_grip_in_tool_m'] = (np.linalg.inv(tool) @ flange @ np.r_[cls.recipe['grip_offset_m'],1.])[:3].tolist()
        cls.sample = {'t':0., 'q':positions, 'tool':tool.tolist()}
        cls.body_point = data.oMf[model.getFrameId('body_link_2')].translation.copy()

    def coupling_sample(self,t,right=None,world=None):
        world=np.eye(4) if world is None else world
        left=np.linalg.inv(np.array(self.config['left_to_tool']).reshape(4,4))
        right=np.linalg.inv(np.array(self.config['right_to_tool']).reshape(4,4)) if right is None else right
        return dict(t=t,flanges={self.config['left_frame']:world @ left,self.config['right_frame']:world @ right})

    def right_roll(self,angle):
        axis=np.array(self.config['right_axial_grasp']['axis_in_tool'])
        mount=np.array(self.config['right_to_tool']).reshape(4,4)
        point=(np.linalg.inv(mount) @ np.r_[self.config['right_axial_grasp']['grip_in_right_flange_m'],1.])[:3]
        spin=np.eye(4);spin[:3,:3]=Rotation.from_rotvec(axis*angle).as_matrix()
        spin[:3,3]=point-spin[:3,:3] @ point
        return spin @ np.linalg.inv(mount)

    def test_tracking_is_report_only_for_coordination_and_required_by_default(self):
        recipe=copy.deepcopy(self.recipe)
        recipe['acceptance']['tool_tracking_policy']='report_only'
        report=tracking_acceptance(recipe,.05,.1)
        self.assertTrue(report['passed']);self.assertFalse(report['within_reference'])
        del recipe['acceptance']['tool_tracking_policy']
        self.assertFalse(tracking_acceptance(recipe,.05,.1)['passed'])
        for policy,value in [('report_only',float('nan')),('unknown',.05)]:
            recipe['acceptance']['tool_tracking_policy']=policy
            with self.assertRaisesRegex(ValueError,'INVALID_TOOL_TRACKING'):
                tracking_acceptance(recipe,value,.1)

    def test_short_rod_uses_grip_midpoint_and_does_not_keep_hidden_pan_collision(self):
        from bindu_teleoperation.simulation_fixture import short_rod_config
        from bindu_kinematics.tool_geometry import make_collision_checker
        rod=short_rod_config(self.config)
        old_mount=np.array(self.config['left_to_tool']).reshape(4,4)
        new_mount=np.array(rod['left_to_tool']).reshape(4,4)
        tool=np.array(self.sample['tool']) @ np.linalg.inv(old_mount) @ new_mount
        # The obsolete pan deliberately covers the whole robot. A rod must
        # contain no pan collision queries, including its ground corners.
        rod['demo']['pan_size_m']=[100.,100.,100.]
        check=make_collision_checker(self.urdf,rod['demo'],conservative_convex=True)
        report=check([dict(self.sample,tool=tool.tolist())])
        self.assertEqual(report['geometry_kind'],'rod')
        self.assertEqual(report['violations'],0)
        self.assertNotEqual((report['closest_tool_robot_pair'] or {}).get('part'),'PAN')

    def test_coupling_accepts_common_motion_but_rejects_one_hand_drift(self):
        world=np.eye(4);world[:3,:3]=Rotation.from_euler('z',.5).as_matrix();world[:3,3]=[.12,-.3,.2]
        rows=[self.coupling_sample(0.),self.coupling_sample(1.,world=world)]
        self.assertTrue(coupling_metrics(rows,self.config,[dict(t=0.,mode='rigid')])['passed'])
        rows[-1]['flanges'][self.config['right_frame']][0,3]+=.002
        self.assertFalse(coupling_metrics(rows,self.config,[dict(t=0.,mode='rigid')])['passed'])

    def test_coupling_allows_axial_roll_but_checks_axis_and_rigid_rotation(self):
        rows=[self.coupling_sample(0.),self.coupling_sample(1.,right=self.right_roll(2.))]
        self.assertTrue(coupling_metrics(rows,self.config,[dict(t=0.,mode='right_axial')])['passed'])
        self.assertFalse(coupling_metrics(rows,self.config,[dict(t=0.,mode='rigid')])['passed'])
        tilt=np.eye(4);tilt[:3,:3]=Rotation.from_rotvec([0.,.04,0.]).as_matrix()
        rows[-1]=self.coupling_sample(1.,right=tilt @ self.right_roll(0.))
        result=coupling_metrics(rows,self.config,[dict(t=0.,mode='right_axial')])
        self.assertFalse(result['passed']);self.assertGreater(result['modes']['right_axial']['max_shaft_axis_rad'],.02)

    def test_coupling_relocks_current_angle_and_requires_capture_coverage(self):
        rows=[self.coupling_sample(0.),self.coupling_sample(1.,right=self.right_roll(2.)),
              self.coupling_sample(2.,right=self.right_roll(2.)),self.coupling_sample(3.,right=self.right_roll(2.))]
        captures=[dict(t=0.,mode='rigid'),dict(t=1.,mode='right_axial'),dict(t=2.,mode='rigid')]
        self.assertTrue(coupling_metrics(rows,self.config,captures)['passed'])
        rows[-1]=self.coupling_sample(3.)
        self.assertFalse(coupling_metrics(rows,self.config,captures)['passed'])
        with self.assertRaisesRegex(ValueError,'MISSING_COUPLING_CAPTURE'):
            coupling_metrics(rows[:1]+rows[2:],self.config,captures)

    def test_selected_ready_pose_has_real_ground_and_tool_clearance(self):
        validate_recipe(self.config)
        result = collision_check(self.urdf, [self.sample], self.recipe)
        self.assertEqual(result['violations'],0)
        self.assertGreaterEqual(result['min_tool_robot_clearance_m'],.02)
        self.assertGreater(result['aabb_bounded_queries'],0)
        self.assertEqual(len(result['support_heights_m']),4)

    def test_tool_ground_contact_can_be_observed_without_blocking_arm_motion(self):
        recipe=copy.deepcopy(self.recipe)
        recipe['acceptance']['min_ground_clearance_m']=.05
        with self.assertRaisesRegex(ValueError,'TOOL_BELOW_GROUND'):
            collision_check(self.urdf,[self.sample],recipe)
        recipe['acceptance']['ground_policy']='report_only'
        result=collision_check(self.urdf,[self.sample],recipe)
        self.assertEqual(result['violations'],0)
        self.assertEqual(result['ground_warning_samples'],1)
        self.assertLess(result['min_tool_ground_clearance_m'],.05)
        self.assertGreaterEqual(result['min_tool_robot_clearance_m'],.02)

    def test_contact_check_does_not_skip_the_shaft_at_the_grip(self):
        # Put the shaft centre through the actual palm. The legacy contact
        # band used to hide this region; the contact path must reject it.
        recipe = copy.deepcopy(self.recipe)
        recipe['check_grasp_contacts'] = True
        sample = copy.deepcopy(self.sample)
        tool = np.array(sample['tool'])
        model = pin.buildModelFromUrdf(str(self.urdf)); data = model.createData(); q = pin.neutral(model)
        for name,value in sample['q'].items():q[model.joints[model.getJointId(name)].idx_q]=value
        pin.framesForwardKinematics(model,data,q)
        palm = data.oMf[model.getFrameId('arm_l_link_7')].homogeneous @ np.array([.151,0,0,1.])
        tool[:3,3] += palm[:3]-(tool @ np.r_[recipe['left_grip_in_tool_m'],1.])[:3]
        sample['tool'] = tool.tolist()
        with self.assertRaisesRegex(ValueError,'GRASP_PENETRATION'):
            collision_check(self.urdf,[sample],recipe)

    def test_wrong_ground_empty_mesh_and_empty_motion_fail(self):
        with self.assertRaisesRegex(ValueError,'EMPTY_COLLISION'):
            collision_check(self.urdf, [], self.recipe)
        for field,value,reason in [('ground',dict(self.recipe['ground'],z_in_base_m=0.),'GROUND_SUPPORT_MISMATCH'),
                                    ('empty_mesh_proxies',{},'MISSING_COLLISION_GEOMETRY')]:
            recipe = dict(self.recipe,**{field:value})
            with self.subTest(field=field), self.assertRaisesRegex(ValueError,reason):
                collision_check(self.urdf,[self.sample],recipe)

    def test_tool_through_body_and_contact_outside_grip_band_fail(self):
        sample = copy.deepcopy(self.sample)
        tool = np.array(sample['tool'])
        tool[:3,3] = self.body_point-tool[:3,:3] @ np.array(self.recipe['pan_center_m'])
        sample['tool'] = tool.tolist()
        with self.assertRaisesRegex(ValueError,'CLEARANCE'):
            collision_check(self.urdf,[sample],self.recipe)
        recipe = copy.deepcopy(self.recipe)
        recipe['check_grasp_contacts'] = False
        for contact in recipe['shaft_contacts'].values():
            contact['half_length_m'] = .01
        with self.assertRaisesRegex(ValueError,'SHAFT_CLEARANCE'):
            collision_check(self.urdf,[self.sample],recipe)

    def test_angle_drift_and_nonfinite_inputs_fail(self):
        config = copy.deepcopy(self.config)
        config['demo']['selected_design']['outward_swing_deg'] = 10.
        with self.assertRaisesRegex(ValueError,'KEYFRAME_DESIGN_MISMATCH'):
            validate_recipe(config)
        config = copy.deepcopy(self.config)
        config['demo']['pan_size_m'][0] = float('nan')
        with self.assertRaisesRegex(ValueError,'NONFINITE_SHOVEL_CONFIG'):
            validate_recipe(config)
        sample = copy.deepcopy(self.sample)
        sample['q']['body_joint_1'] = float('nan')
        with self.assertRaisesRegex(ValueError,'NONFINITE_COLLISION_SAMPLE'):
            collision_check(self.urdf,[sample],self.recipe)


    def test_receiving_bin_is_open_but_walls_are_checked(self):
        recipe = copy.deepcopy(self.recipe)
        recipe['receiving_bin'] = {'center_xy_m':[10.,10.], 'inner_size_xy_m':[.7,.6],
            'height_m':.45, 'wall_thickness_m':.02, 'bottom_z_m':recipe['ground']['z_in_base_m'],
            'min_clearance_m':.02}
        self.assertEqual(len(bin_boxes(recipe)),5)
        result = collision_check(self.urdf,[self.sample],recipe)
        self.assertGreater(result['min_bin_clearance_m'],1.)
        tool = np.array(self.sample['tool'])
        center = (tool @ np.r_[recipe['pan_center_m'],1.])[:3]
        recipe['receiving_bin']['center_xy_m'] = [float(center[0]-.36),float(center[1])]
        with self.assertRaisesRegex(ValueError,'BIN_CLEARANCE'):
            collision_check(self.urdf,[self.sample],recipe)

    def test_dump_requires_inversion_containment_and_complete_hold(self):
        recipe = copy.deepcopy(self.recipe)
        recipe['receiving_bin'] = {'center_xy_m':[0.,0.], 'inner_size_xy_m':[.8,.8],
            'height_m':.4,'wall_thickness_m':.02,'bottom_z_m':0.,'min_clearance_m':.02}
        recipe['dump'] = {'hold_seconds':[.5,1.5],'transfer_seconds':[0.,2.],
            'mouth_margin_m':.04,'min_rim_clearance_m':.05,'min_pan_tilt_deg':100.}
        tool = np.eye(4)
        tool[:3,:3] = Rotation.from_euler('x',120,degrees=True).as_matrix()
        tool[:3,3] = np.array([0.,0.,.9])-tool[:3,:3] @ np.array(recipe['pan_center_m'])
        samples = [{'t':t,'tool':tool.tolist()} for t in (0.,.5,1.,1.5,2.)]
        result = check_dump(samples,recipe)
        self.assertAlmostEqual(result['min_pan_tilt_deg'],120.)
        with self.assertRaisesRegex(ValueError,'MISSING_DUMP_SAMPLES'):
            check_dump(samples[1:],recipe)
        for field,reason in [('upright','DUMP_PAN_NOT_INVERTED'),('outside','DUMP_MISSES_BIN'),
                             ('low','DUMP_RIM_CLEARANCE'),('nan','NONFINITE_DUMP_POSE')]:
            bad = copy.deepcopy(samples)
            for sample in bad:
                tf = np.array(sample['tool'])
                if field == 'upright': tf[:3,:3]=np.eye(3)
                if field == 'outside': tf[0,3]+=1.
                if field == 'low': tf[2,3]-=.6
                if field == 'nan': tf[0,3]=float('nan')
                sample['tool']=tf.tolist()
            with self.subTest(field=field), self.assertRaisesRegex(ValueError,reason):
                check_dump(bad,recipe)

    def test_shaft_roll_preserves_both_grip_points(self):
        axis=np.array(self.recipe['shaft_axis_in_tool'])
        pivot=np.array(self.recipe['pivot_in_tool_m'])
        keys=[{'t':0.,'translation_m':[0.,0.,0.],'yaw_pitch_rad':[0.,0.],'shaft_roll_rad':0.},
              {'t':1.,'translation_m':[0.,0.,0.],'yaw_pitch_rad':[0.,0.],'shaft_roll_rad':2.}]
        for t in np.linspace(0,1,11):
            tf=target_at(keys,np.eye(4),pivot,t,axis)
            for point in (pivot,pivot-.34*axis):
                np.testing.assert_allclose((tf @ np.r_[point,1.])[:3],point,atol=1e-10)

    def test_target_final_grid_point_tolerates_only_roundoff(self):
        keys = self.recipe['keyframes']
        t = np.nextafter(keys[-1]['t'], float('inf'))
        target_at(keys,np.eye(4),np.array(self.recipe['pivot_in_tool_m']),t,np.array(self.recipe['shaft_axis_in_tool']))
        for invalid in (t+.01, float('nan')):
            with self.assertRaisesRegex(ValueError,'TIME_OUTSIDE_RECIPE'):
                target_at(keys,np.eye(4),np.zeros(3),invalid)

    def test_grasp_changes_require_support_but_not_a_prearranged_hold(self):
        recipe = self.config['demo']
        schedule = recipe['grasp_schedule']
        validate_recipe(self.config)
        for change in schedule[1:]:
            self.assertEqual(grasp_mode_at(recipe, change['t']), change['mode'])
            self.assertNotEqual(grasp_mode_at(recipe, change['t']-.01), change['mode'])
        # Choose moving segments from the recipe: fixed timestamps can fall
        # after re-lock when the demonstration is sped up.
        fields = ('translation_m', 'yaw_pitch_rad', 'shaft_roll_rad')
        moving_times = [(a['t']+b['t'])/2 for a,b in zip(recipe['keyframes'], recipe['keyframes'][1:])
                        if b['t'] < schedule[-1]['t'] and
                        any(a.get(field,0.) != b.get(field,0.) for field in fields)]
        self.assertGreaterEqual(len(moving_times),2)
        for when in moving_times:
            moving=copy.deepcopy(self.config);moving['demo']['grasp_schedule'][1]['t']=when
            validate_recipe(moving)
        for field,value,reason in [('mode','open','INVALID_SHOVEL_GRASP_SCHEDULE'),
                                   ('mode','rigid','INVALID_SHOVEL_GRASP_SCHEDULE')]:
            bad = copy.deepcopy(self.config)
            bad['demo']['grasp_schedule'][1][field] = value
            with self.subTest(field=field,value=value), self.assertRaisesRegex(ValueError,reason):
                validate_recipe(bad)
        bad = copy.deepcopy(self.config)
        del bad['right_axial_grasp']
        with self.assertRaisesRegex(ValueError,'INVALID_SHOVEL_GRASP_SCHEDULE'):
            validate_recipe(bad)
        self.assertEqual(grasp_mode_at({}, 10.), 'rigid')

    def test_motion_cannot_supply_staged_waist_joint_targets(self):
        validate_recipe(self.config)
        for late_key in (False, True):
            config=copy.deepcopy(self.config)
            if late_key:
                config['demo']['keyframes'][-1]['shared_targets_rad']={'body_joint_2':.2}
            else:
                config['shared_targets']={'body_joint_1':0.,'body_joint_2':.2}
            with self.subTest(late_key=late_key), self.assertRaisesRegex(ValueError,'REQUIRES_TOOL_POSE_INPUT'):
                validate_recipe(config)

    def test_offline_smoothing_and_planning_overrides_are_forbidden(self):
        for key in ('joint_filter_sigma_s','grasp_switch_hold_s','grasp_switch_ramp_s',
                    'finish_hold_s','finish_ramp_s'):
            bad=copy.deepcopy(self.config);bad['demo'][key]=.3
            with self.subTest(key=key),self.assertRaisesRegex(ValueError,'OFFLINE_CONDITION'):
                validate_recipe(bad)
        bad=copy.deepcopy(self.config);bad['simulation_only']=False
        with self.assertRaisesRegex(ValueError,'REQUIRES_SIMULATION'):
            validate_recipe(bad)

    def test_stream_causality_and_unplanned_grasp_change_while_moving(self):
        from bindu_contracts.profile import Profile
        from bindu_contracts.teleoperation import ToolPoseFrame
        from bindu_kinematics.bimanual import BimanualQP
        cfg=copy.deepcopy(self.config)
        cfg['urdf']=str(ROOT/'src/hardware/bindu_description/urdf'/cfg['urdf'])
        profile=Profile.load(ROOT/'src/integration/bindu_runtime/config/robots/snowbot'/cfg['demo']['profile'])
        runs=[]
        for future_direction in (-1.,1.):
            solver=BimanualQP(cfg,profile)
            try:
                home=solver.capture(cfg['demo']['initial_positions_rad'])
                def frames():
                    for k in range(151):
                        t=round(k*.01,9);pose=home.copy()
                        pose[0,3]+=.006*t if t<=1. else .006+future_direction*.006*(t-1.)
                        yield ToolPoseFrame('test',k,t,cfg['input_frame'],tuple(pose.ravel()),t<.8 or t>=1.2)
                records,events,packets,_=run_input_stream(solver,profile,cfg,cfg['demo']['initial_positions_rad'],frames())
                self.assertTrue(any(e['event']=='grasp_requested' and e['t']==.8 for e in events))
                applied=[e for e in events if e['event']=='grasp_applied'][1:]
                self.assertEqual([e['mode'] for e in applied],['right_axial','rigid'])
                self.assertTrue(all(e['speed_rad_s']<=.005 for e in applied))
                self.assertGreaterEqual(applied[0]['t'],.8)
                self.assertEqual(len(packets),151)
                self.assertEqual(events[-1]['event'],'input_timeout_stop')
                runs.append(np.array([[row['q'][n] for n in solver.names] for row in records]))
            finally:
                solver.close()
        np.testing.assert_allclose(runs[0][:101],runs[1][:101],atol=1e-12)



if __name__ == '__main__':
    unittest.main()


class OnlineGeometryTests(unittest.TestCase):
    setUpClass = classmethod(ShovelGeometryTests.setUpClass.__func__)
    def test_fast_recipe_tracks_both_grips_without_losing_coupling(self):
        from bindu_contracts.profile import Profile
        from bindu_contracts.teleoperation import ToolPoseFrame
        from bindu_kinematics.bimanual import BimanualQP
        cfg=copy.deepcopy(self.config)
        cfg['urdf']=str(ROOT/'src/hardware/bindu_description/urdf'/cfg['urdf'])
        profile=Profile.load(self.config_path.parent/cfg['demo']['profile'])
        solver=BimanualQP(cfg,profile);self.addCleanup(solver.close)
        initial=cfg['demo']['initial_positions_rad'];home=solver.capture(initial)
        recipe=cfg['demo'];dt=profile.control_period
        pivot=np.asarray(recipe['pivot_in_tool_m']);axis=recipe['shaft_axis_in_tool']
        targets=[]
        def frames():
            for step in range(round(recipe['keyframes'][-1]['t']/dt)+1):
                t=round(step*dt,9);pose=target_at(recipe['keyframes'],home,pivot,t,axis)
                targets.append(pose)
                yield ToolPoseFrame('fast_recipe_regression',step,t,cfg['input_frame'],
                    tuple(pose.ravel()),grasp_mode_at(recipe,t)=='rigid')
        records,events,_,_=run_input_stream(solver,profile,cfg,initial,frames())
        model=pin.buildModelFromUrdf(cfg['urdf']);data=model.createData()
        offsets={s:np.r_[recipe.get(s+'_grip_offset_m',recipe['grip_offset_m']),1.]
                 for s in ('left','right')}
        nominal={s:np.linalg.inv(np.asarray(cfg[s+'_to_tool']).reshape(4,4)) @ p
                 for s,p in offsets.items()}
        peaks=dict(left=0.,right=0.);samples=[]
        for row in records:
            q=pin.neutral(model)
            for n,v in row['q'].items():q[model.joints[model.getJointId(n)].idx_q]=v
            pin.framesForwardKinematics(model,data,q)
            poses={s:data.oMf[model.getFrameId(cfg[s+'_frame'])].homogeneous.copy()
                   for s in ('left','right')}
            target=targets[min(round(row['t']/dt),len(targets)-1)]
            for s in peaks:
                peaks[s]=max(peaks[s],float(np.linalg.norm((poses[s] @ offsets[s]-target @ nominal[s])[:3])))
            samples.append(dict(t=row['t'],flanges={cfg[s+'_frame']:p for s,p in poses.items()}))
        # Capture the former 0.18 m lag independently of report_only policy;
        # this does not reinstate the old 20 mm task acceptance gate.
        for side,error in peaks.items():self.assertLess(error,.04,side+' tracking lag')
        captures=[dict(t=e['t'],mode=e['mode']) for e in events if e['event']=='grasp_applied']
        self.assertTrue(coupling_metrics(samples,cfg,captures)['passed'])
        self.assertEqual(events[-1]['event'],'input_timeout_stop')

    def test_online_outer_guard_preserves_ready_and_rejects_body_overlap(self):
        # Check the conservative online representation against a real CAD fixture.
        from bindu_kinematics.tool_geometry import make_collision_checker
        check=make_collision_checker(self.urdf,self.recipe,conservative_convex=True)
        result=check([self.sample])
        self.assertEqual(result['tool_body_representation'],'convex_outer')
        self.assertGreaterEqual(result['min_tool_robot_clearance_m'],.02)
        bad=copy.deepcopy(self.sample)
        bad['tool']=np.eye(4);bad['tool'][:3,3]=self.body_point
        with self.assertRaisesRegex(ValueError,'CLEARANCE|COLLISION'):
            check([bad])
        with self.assertRaisesRegex(ValueError,'EMPTY'):
            check([])
