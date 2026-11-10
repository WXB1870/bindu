"""Production solve/contact guard checks; no ROS or hardware."""
from bindu_contracts.resources import package_share
import ast
import ctypes as c
import math
from pathlib import Path
import time
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock, patch
import numpy as np
from bindu_kinematics.tool_geometry import _shaft_contact_span

SOURCE=Path(__file__).resolve().parents[2]/'src/capabilities/bindu_kinematics/bindu_kinematics/bimanual.py'
TREE=ast.parse(SOURCE.read_text())
# Load the production class without the ROS package locator; fixtures bypass
# native construction, and every solve executes the full production method.
TREE.body=[n for n in TREE.body if not (isinstance(n,ast.ImportFrom) and n.module=='ament_index_python.packages')]
NS={};exec(compile(TREE,str(SOURCE),'exec'),NS);QP=NS['BimanualQP']


class GraspReferenceLifetimeTests(unittest.TestCase):
    def solver(self):
        s=QP.__new__(QP);s.handle=object();s.captured=False
        s.names=s.model_names=('joint',);s.nq=1;s._ptr=lambda x:x
        s.cfg={'flange_distance_range':[0.,1.]}
        s.limits={'joint':(-4.,4.)};s.posture_policy=None;s.posture_reference=None
        s.measured_grasp_policy='enforce';s.measured_rigid_grasp=True
        s.expected_relative=np.eye(4);s.rigid_grasp_q=s.rigid_grasp_reference=None
        s.reference_kinematics=True;s.max_reference_error=.15
        s.grasp_position_tolerance=.01;s.grasp_rotation_tolerance=.02
        s.axial=s.left_axial=None;s.tool=np.eye(4);s.collision_check=None;s.tracking_point=np.zeros(3)
        s.native_config=NS['NativeConfig']();s.coupled_locked_joints=()
        s.shared_motion_weight=0.;s.limit_avoidance=(0.,0.)
        def relative(handle,q,n,out):
            angle=q[0];out[:]=np.eye(4)
            out[:2,:2]=[[math.cos(angle),-math.sin(angle)],
                         [math.sin(angle),math.cos(angle)]]
            out[0,3]=.1*angle
            return 1
        def tool(handle,q,n,out):out[:]=np.eye(4);return 1
        s.lib=S(bq_relative_pose=relative,bq_tool_pose=tool,
                bq_capture=Mock(return_value=1),bq_destroy=Mock(),
                bq_capture_independent=Mock(return_value=1))
        def flanges(handle,q,n,left,right):left[:]=right[:]=np.eye(4);return 1
        s.lib.bq_flange_poses=flanges;s.independent_abi=Mock()
        s.check_independent_path=Mock()
        s.check_path=Mock();s.set_shared_targets=Mock()
        self.addCleanup(s.close)
        return s

    def test_mode_preview_then_a_return_can_preview_and_capture_the_new_pose(self):
        s=self.solver()
        s.capture({'joint':0.},commit_grasp=False)
        self.assertIsNone(s.rigid_grasp_q)
        self.assertIsNone(s.rigid_grasp_reference)
        # B back to independent mode is also only a readiness check. A may
        # then change the actual pose; neither query establishes a held grasp.
        s.capture_independent({'joint':0.},preserve_rigid_grasp=True)
        ready={'joint':.8}
        s.capture(ready,commit_grasp=False)
        self.assertIsNone(s.rigid_grasp_reference)
        s.capture(ready)
        np.testing.assert_array_equal(s.rigid_grasp_q,[.8])
        self.assertAlmostEqual(s.rigid_grasp_reference[0,3],.08)
        self.assertEqual(s.check_path.call_count,3)

    def test_previews_and_reclutches_keep_the_first_actual_grasp_and_reject_drift(self):
        s=self.solver();s.capture({'joint':0.})
        original=s.rigid_grasp_reference.copy()
        for drift in (.006,.012,.018):
            s.capture_independent({'joint':drift},preserve_rigid_grasp=True)
            for commit in (False,True):
                s.capture({'joint':drift},commit_grasp=commit)
                np.testing.assert_array_equal(s.rigid_grasp_reference,original)
                np.testing.assert_array_equal(s.rigid_grasp_q,[0.])
        for commit in (False,True,False):
            with self.assertRaisesRegex(ValueError,'BIMANUAL_GRASP_POSE'):
                s.capture({'joint':.024},commit_grasp=commit)
            self.assertFalse(s.captured)
            np.testing.assert_array_equal(s.rigid_grasp_reference,original)
        s.capture({'joint':0.},commit_grasp=False)
        np.testing.assert_array_equal(s.rigid_grasp_reference,original)

    def test_rebuilt_worker_does_not_recapture_a_drifted_pose_as_the_correct_grasp(self):
        import queue
        import sys
        from bindu_kinematics.bimanual_worker import _run
        original=self.solver();original.capture({'joint':0.})
        checkpoint=(original.rigid_grasp_q.copy(),original.rigid_grasp_reference.copy(),None)
        rebuilt=self.solver()
        requests,results=queue.Queue(),queue.Queue()
        requests.put(('capture',1,{'joint':.024},None,None,None,'rigid',None,None,False))
        requests.put(('capture',2,{'joint':0.},None,None,None,'rigid',None,None,False))
        requests.put(None)
        with patch.dict(sys.modules,{'bindu_kinematics.bimanual':S(BimanualQP=lambda *a:rebuilt)}), \
                patch('bindu_kinematics.bimanual_worker.gc.collect'),patch('bindu_kinematics.bimanual_worker.gc.freeze'):
            _run({},None,requests,results,grasp_checkpoint=checkpoint)
        replies=list(results.queue)
        self.assertIn(('error',1,'BIMANUAL_GRASP_POSE'),replies)
        self.assertTrue(any(kind=='result' and token==2 for kind,token,_ in replies))
        np.testing.assert_array_equal(rebuilt.rigid_grasp_reference,checkpoint[1])

    def test_failed_preview_does_not_latch_or_erase_a_grasp(self):
        s=self.solver()
        with patch.object(s,'check_path',side_effect=ValueError('SELF_COLLISION')):
            with self.assertRaisesRegex(ValueError,'SELF_COLLISION'):
                s.capture({'joint':0.},commit_grasp=False)
        self.assertIsNone(s.rigid_grasp_reference)
        s.capture({'joint':0.});original=s.rigid_grasp_reference.copy()
        with patch.object(s,'check_path',side_effect=ValueError('SELF_COLLISION')):
            with self.assertRaisesRegex(ValueError,'SELF_COLLISION'):
                s.capture({'joint':.001},commit_grasp=False)
        np.testing.assert_array_equal(s.rigid_grasp_reference,original)
        self.assertFalse(s.captured)

    def test_preview_still_checks_nominal_grasp_limits_and_feedback(self):
        s=self.solver();s.measured_rigid_grasp=False
        with self.assertRaisesRegex(ValueError,'BIMANUAL_GRASP_POSE'):
            s.capture({'joint':.1},commit_grasp=False)
        s.lib.bq_capture.assert_not_called()
        with self.assertRaisesRegex(ValueError,'BIMANUAL_FEEDBACK_NONFINITE'):
            s.capture({'joint':float('nan')},commit_grasp=False)
        s.measured_rigid_grasp=True;s.capture({'joint':0.})
        with self.assertRaisesRegex(ValueError,'BIMANUAL_MEASURED_POSITION_LIMIT'):
            s.capture({'joint':4.01},commit_grasp=False)

    def axial_solver(self):
        s=self.solver();s.measured_rigid_grasp=False
        s.cfg['grasp_reference_source']='measured_axial'
        s.cfg['right_axial_grasp']={}
        s.axial={};s.right_grip=np.eye(4);s.right_grip[0,3]=.1
        s.tool_grip=s.right_grip.copy()
        s.lib.bq_capture_axial=Mock(return_value=1)
        # Rotate about the grip point, not the flange origin.
        def relative(handle,q,n,out):
            angle=q[0];out[:]=np.eye(4)
            out[:2,:2]=[[math.cos(angle),-math.sin(angle)],
                         [math.sin(angle),math.cos(angle)]]
            out[:3,3]=s.right_grip[:3,3]-out[:3,:3] @ s.right_grip[:3,3]
            return 1
        s.lib.bq_relative_pose=relative
        return s

    def test_axial_reclutch_keeps_original_grip_but_allows_large_roll(self):
        s=self.axial_solver()
        s.capture({'joint':0.},'right_axial',commit_grasp=False)
        self.assertIsNone(s.rigid_grasp_reference)
        s.capture({'joint':.1},'right_axial')
        self.assertIsNotNone(s.rigid_grasp_reference)
        original=s.rigid_grasp_reference.copy();grip=s.tool_grip.copy()
        for roll in (1.,3.1,-3.1):
            s.capture_independent({'joint':roll},preserve_rigid_grasp=True)
            for commit in (False,True):
                s.capture({'joint':roll},'right_axial',commit_grasp=commit)
                np.testing.assert_array_equal(s.grasp_reference,original)
                np.testing.assert_allclose(s.tool_grip,grip,atol=1e-12)
                np.testing.assert_allclose(s._grip_error(s.grasp_reference),(0.,0.),atol=1e-7)
        s.lib.bq_capture.assert_not_called()
        s.capture_independent({'joint':0.})
        self.assertIsNone(s.rigid_grasp_reference)

    def test_axial_default_capture_cannot_be_changed_back_to_rigid(self):
        s=self.axial_solver();s.capture({'joint':0.})
        self.assertEqual(s.grasp_mode,'right_axial')
        with self.assertRaisesRegex(ValueError,'REQUIRES_AXIAL'):
            s.capture({'joint':0.},'rigid')
        s.lib.bq_capture.assert_not_called()

    def test_axial_recapture_rejects_sliding_tilt_and_reversed_axis_without_rebasing(self):
        for axis,amount in ((0,.011),(2,.011),('tilt',.021),('tilt',math.pi)):
            with self.subTest(axis=axis,amount=amount):
                s=self.axial_solver();s.capture({'joint':0.},'right_axial')
                original=s.rigid_grasp_reference.copy()
                bad=original.copy()
                if axis=='tilt':
                    bad[:3,:3]=[[1,0,0],[0,math.cos(amount),-math.sin(amount)],
                                          [0,math.sin(amount),math.cos(amount)]]
                else:bad[axis,3]+=amount
                s.lib.bq_relative_pose=lambda handle,q,n,out:(out.__setitem__(slice(None),bad) or 1)
                for commit in (False,True,False):
                    with self.assertRaisesRegex(ValueError,'BIMANUAL_GRASP_POSE'):
                        s.capture({'joint':0.},'right_axial',commit_grasp=commit)
                    self.assertFalse(s.captured)
                    np.testing.assert_array_equal(s.rigid_grasp_reference,original)

    def test_axial_path_checks_allow_roll_but_reject_grip_translation(self):
        s=self.axial_solver();s.capture({'joint':0.},'right_axial')
        s.check_path=QP.check_path.__get__(s);s.collision_step=.01
        s.collision_check=Mock()
        s.check_path({'joint':0.},np.array([.3]))
        s.collision_check.assert_called_once()
        native=s.lib.bq_relative_pose
        def slide(handle,q,n,out):
            native(handle,q,n,out);out[2,3]+=.1*q[0];return 1
        s.lib.bq_relative_pose=slide
        with self.assertRaisesRegex(ValueError,'GRASP_CLOSURE'):
            s.check_path({'joint':0.},np.array([.3]))
        self.assertEqual(s.collision_check.call_count,1)

    def test_loaded_path_uses_axial_closure_instead_of_flange_pose(self):
        s=self.axial_solver();s.cfg['demo']={'check_grasp_contacts':True}
        s.collision_step=.01;s.collision_check=Mock(return_value={})
        result=s.check_loaded_prepare({'joint':0.},{'joint':.3})
        self.assertTrue(result['coupling_checked'])
        native=s.lib.bq_relative_pose
        def slide(handle,q,n,out):
            native(handle,q,n,out);out[2,3]+=.1*q[0];return 1
        s.lib.bq_relative_pose=slide
        with self.assertRaisesRegex(ValueError,'PREPARE_TOOL_COUPLING'):
            s.check_loaded_prepare({'joint':0.},{'joint':.3})


class ShaftContactBandTests(unittest.TestCase):
    ends = (-.44, .1)

    def test_default_keeps_strict_endpoint_check(self):
        np.testing.assert_allclose(_shaft_contact_span(self.ends, -.34, .1), (-.44, -.24))
        with self.assertRaisesRegex(ValueError, 'INVALID_SHAFT_CONTACT_BAND'):
            _shaft_contact_span(self.ends, -.3432, .1)

    def test_recorded_recenterings_clip_only_the_outside_part(self):
        import json
        config=package_share('snowbot_bringup')/'config/robots/snowbot/bimanual_snowbot_real.json'
        tolerance=json.loads(config.read_text())['demo']['shaft_contact_endpoint_tolerance_m']
        for center in (-.3431461408896104, -.3431818229817779, -.34047735756758685,
                       -.3404849855910973, -.3404773575675868, -.3404779169447534):
            with self.subTest(center=center):
                start,stop=_shaft_contact_span(self.ends, center, .1, tolerance)
                self.assertEqual(start, self.ends[0])
                self.assertAlmostEqual(stop, center+.1)

    def test_both_ends_have_six_centimetres_of_capture_margin(self):
        for center in (-.34-.06, .06, -.34-.1, .1):
            with self.subTest(center=center):
                start,stop=_shaft_contact_span(self.ends, center, .1, .1)
                self.assertGreaterEqual(start, self.ends[0])
                self.assertLessEqual(stop, self.ends[1])
                self.assertLess(start, stop)
        for center in (-.34-.1001, .1001):
            with self.subTest(center=center), self.assertRaisesRegex(ValueError, 'INVALID_SHAFT_CONTACT_BAND'):
                _shaft_contact_span(self.ends, center, .1, .1)

    def test_a_band_outside_the_rod_is_rejected_even_inside_the_margin(self):
        for center in (-.48, .14):
            with self.subTest(center=center), self.assertRaisesRegex(ValueError, 'INVALID_SHAFT_CONTACT_BAND'):
                _shaft_contact_span(self.ends, center, .02, .1)

    def test_invalid_numbers_and_geometry_still_reject(self):
        for args in ((self.ends, -.34, .1, -.1), (self.ends, -.34, 0., .1),
                     (self.ends, -.34, -.1, .1), (self.ends, float('nan'), .1, .1),
                     (self.ends, -.34, float('inf'), .1), (self.ends, -.34, .1, float('nan')),
                     (self.ends, -.34, .1, float('inf')), (self.ends, -.34, .1, True),
                     ((-.44, -.44), -.34, .1, .1), ((-.44,), -.34, .1, .1)):
            with self.subTest(args=args), self.assertRaisesRegex(ValueError, 'INVALID_SHAFT_CONTACT_BAND'):
                _shaft_contact_span(*args)

    def test_production_query_layout_keeps_full_rod_for_non_contact_bodies(self):
        # Exercise the real layout builder with shape dimensions only; no
        # collision engine or fabricated distance result is needed here.
        tree=ast.parse(SOURCE.with_name('tool_geometry.py').read_text())
        layout=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='shaft_queries')
        shape=lambda radius,length:S(radius=radius,length=length)
        for check_contacts in (False,True):
            namespace=dict(np=np,names=['hand','base'],ends=np.array(self.ends),radius=.018,
                grasp_penetration=.005,geometry_kind='rod',has_pan=False,
                recipe={'left_grip_in_tool_m':[0.,0.,0.]},shaft_axis=np.array([1.,0.,0.]),
                check_grasp_contacts=check_contacts,contact_endpoint_tolerance=.1,
                _shaft_contact_span=_shaft_contact_span,fcl=S(Cylinder=shape,Capsule=shape))
            exec(compile(ast.Module(body=[layout],type_ignores=[]),str(SOURCE),'exec'),namespace)
            parts,queries,_,_=namespace['shaft_queries']({'hand':{'center_m':-.4,'half_length_m':.1}})
            base=[parts[i] for body,i in queries if body==1]
            hand=[parts[i] for body,i in queries if body==0]
            self.assertEqual(len(base),1);self.assertEqual(len(hand),1)
            for _,cylinder,center in base+hand:
                self.assertGreaterEqual(center[0]-cylinder.length/2.,self.ends[0]-1e-12)
                self.assertLessEqual(center[0]+cylinder.length/2.,self.ends[1]+1e-12)
            self.assertAlmostEqual(base[0][1].length,.54)
            self.assertAlmostEqual(base[0][1].radius,.018)
            self.assertAlmostEqual(hand[0][1].length,.54 if check_contacts else .4)
            self.assertAlmostEqual(hand[0][1].radius,.013 if check_contacts else .018)


class UnheldGraspPolicyTests(unittest.TestCase):
    def solver(self,policy='enforce',distance=.0,angle=.0):
        s=QP.__new__(QP);s.handle=None;s.captured=True
        s.measured_grasp_policy=policy;s.cfg={'max_relative_error':.03,'solve_timeout_us':1e8}
        s.max_interval=.1;s.reference_kinematics=True;s.tracking_feedback='reference'
        s.tracking=True;s.feedforward=False;s.max_reference_error=.15
        s.names=('joint',);s.model_names=s.names;s.nq=1
        s.posture_policy=None;s.limits={'joint':(-1.,1.)};s.speed={'joint':1.}
        s.grasp_mode='rigid';s.grasp_reference=np.eye(4);s.measured_rigid_grasp=True
        s.expected_relative=np.eye(4);s.grasp_position_tolerance=.01;s.grasp_rotation_tolerance=.02
        s._q=lambda feedback:np.array([feedback['joint']]);s._ptr=lambda x:x
        relative=np.eye(4);relative[0,3]=distance
        relative[:2,:2]=[[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]]
        def tool(h,q,n,out):out[:]=np.eye(4);return 1
        def rel(h,q,n,out):out[:]=relative;return 1
        def solve(h,q,n,reference,count,target,velocity,measured,dt,out,metrics):
            self.assertIsNone(measured);np.testing.assert_array_equal(q,reference)
            out[:]=reference+.001;metrics[:]=[1.,0.,1.];return 1
        s.lib=S(bq_tool_pose=tool,bq_relative_pose=rel,bq_solve_tracking=Mock(side_effect=solve))
        s.checked_geometry_step=Mock(side_effect=lambda fb,ref,out,**kw:(out,1.))
        return s

    def solve(self,s,feedback=None,reference=None,**kw):
        return s.solve({'joint':0.} if feedback is None else feedback,np.eye(4),
            reference=[0.] if reference is None else reference,dt=.01,**kw)

    def test_qp_timeout_keeps_capture_and_the_next_fresh_solve_does_not_require_recapture(self):
        for stage in ('native','validation'):
            with self.subTest(stage=stage):
                s=self.solver('unheld_test');s.cfg['solve_timeout_us']=1000.
                s.feedforward=True;s.target_sample=None
                clock=[0.];calls=[0];grasp=s.grasp_reference.copy()
                method=s.lib.bq_solve_tracking if stage=='native' else s.checked_geometry_step
                original=method.side_effect
                def delayed(*args,**kwargs):
                    result=original(*args,**kwargs)
                    calls[0]+=1
                    if calls[0]==1:clock[0]+=.002
                    return result
                method.side_effect=delayed
                with patch.object(NS['time'],'perf_counter',side_effect=lambda:clock[0]):
                    with self.assertRaisesRegex(NS['DeferredSolve'],'BIMANUAL_QP_TIMEOUT:'+stage):
                        self.solve(s,target_stamp=1.)
                    self.assertTrue(s.captured)
                    self.assertIsNone(s.target_sample)
                    out,_=self.solve(s,target_stamp=1.01)
                self.assertEqual(out,(.001,))
                self.assertEqual(s.target_sample[0],1.01)
                np.testing.assert_array_equal(s.grasp_reference,grasp)
                self.assertEqual(s.lib.bq_solve_tracking.call_count,2)

    def test_independent_timeout_keeps_capture_and_allows_the_next_fresh_solve(self):
        s=self.solver('unheld_test');s.grasp_mode='independent';s.cfg['solve_timeout_us']=1000.
        clock=[0.];calls=[0]
        def native(*args):
            args[-2][:]=.001;args[-1][:]=[1.,0.,1.]
            calls[0]+=1
            if calls[0]==1:clock[0]+=.002
            return 1
        s.lib.bq_solve_independent=Mock(side_effect=native)
        targets=np.array([np.eye(4),np.eye(4)])
        with patch.object(NS['time'],'perf_counter',side_effect=lambda:clock[0]):
            with self.assertRaisesRegex(NS['DeferredSolve'],'BIMANUAL_QP_TIMEOUT:independent'):
                s.solve_independent({'joint':0.},targets,active_mask=1,reference=[0.],dt=.01)
            self.assertTrue(s.captured)
            out,_=s.solve_independent({'joint':0.},targets,active_mask=1,reference=[0.],dt=.01)
        self.assertEqual(out,(.001,));self.assertEqual(calls[0],2)

    def test_axial_solve_allows_roll_but_checks_measured_slide_and_tilt(self):
        for failure in (None,'slide','tilt'):
            with self.subTest(failure=failure):
                s=self.solver(angle=2.4);s.grasp_mode='right_axial'
                s.cfg['grasp_reference_source']='measured_axial'
                s.measured_rigid_grasp=False;s.axial={}
                s.tool=s.tool_grip=s.right_grip=np.eye(4)
                if failure is not None:
                    relative=np.eye(4)
                    if failure=='slide':relative[2,3]=.011
                    else:
                        angle=.021
                        relative[:3,:3]=[[1,0,0],[0,math.cos(angle),-math.sin(angle)],
                                                   [0,math.sin(angle),math.cos(angle)]]
                    def measured(handle,q,n,out):out[:]=relative;return 1
                    s.lib.bq_relative_pose=measured
                    with self.assertRaisesRegex(ValueError,'BIMANUAL_MEASURED_GRASP_ERROR'):
                        self.solve(s)
                    s.lib.bq_solve_tracking.assert_not_called()
                else:
                    out,_=self.solve(s)
                    np.testing.assert_allclose(out,[.001])

    def test_real_recipe_measured_axial_grasp_limits(self):
        from bindu_contracts.teleoperation import load_tool_config
        config_path=package_share('snowbot_bringup')/'config/robots/snowbot/bimanual_snowbot_real.json'
        config=load_tool_config(config_path)
        self.assertEqual(config['grasp_reference_source'],'measured_axial')
        self.assertEqual(config['grasp_position_tolerance_m'],.08)
        self.assertAlmostEqual(config['grasp_rotation_tolerance_rad'],math.radians(7))
        self.assertEqual(config['solve_timeout_us'],150000)
        self.assertEqual(config['demo']['acceptance']['min_tool_robot_clearance_m'],.02)
        # Include the Orin stop and the corner where both errors approach their limits.
        for distance,angle,accept in ((.04681671,.08004057,True),(.07999,math.radians(6.999),True),
                                      (.08001,0.,False),(0.,math.radians(7.001),False)):
            with self.subTest(distance=distance,angle=angle):
                solver=self.solver()
                solver.cfg.update(config)
                solver.grasp_mode='right_axial'
                solver.left_axial=None
                solver.grasp_position_tolerance=config['grasp_position_tolerance_m']
                solver.grasp_rotation_tolerance=config['grasp_rotation_tolerance_rad']
                solver.tool=solver.tool_grip=solver.right_grip=np.eye(4)
                relative=np.eye(4)
                relative[0,3]=distance
                relative[1:3,1:3]=[[math.cos(angle),-math.sin(angle)],
                                    [math.sin(angle),math.cos(angle)]]
                def read_relative(handle,q,count,out):
                    out[:]=relative
                    return 1
                solver.lib.bq_relative_pose=read_relative
                if accept:
                    self.solve(solver)
                    solver.lib.bq_solve_tracking.assert_called_once()
                else:
                    with self.assertRaisesRegex(ValueError,'BIMANUAL_MEASURED_GRASP_ERROR'):
                        self.solve(solver)
                    self.assertEqual(solver.last_constraint['stage'],'measured_grasp_points')

    def test_unheld_capture_uses_actual_relative_pose_without_nominal_grasp_gate(self):
        for policy in ('enforce','unheld_test'):
            s=self.solver(policy,.04,.05);s.handle=object();s.axial=None
            s.cfg.update(flange_distance_range=[0.,1.],left_to_tool=np.eye(4).ravel().tolist())
            s.tool=np.eye(4);s.measured_rigid_grasp=False;s.native_config=NS['NativeConfig']()
            s.tracking_point=np.zeros(3);s.posture_reference=None;s.collision_check=None
            s.rigid_grasp_q=s.rigid_grasp_reference=None
            s.limit_avoidance=(0.,0.);s.coupled_locked_joints=()
            s.lib.bq_capture=Mock(return_value=1);s.lib.bq_destroy=Mock();s._apply_shared_motion_weight=Mock()
            s.check_path=Mock();s.set_shared_targets=Mock()
            if policy=='enforce':
                with self.assertRaisesRegex(ValueError,'BIMANUAL_GRASP_POSE'):s.capture({'joint':0.})
                s.lib.bq_capture.assert_not_called()
            else:
                s.capture({'joint':0.})
                s.lib.bq_capture.assert_called_once();s.check_path.assert_called_once_with({'joint':0.},[0.])
                self.assertAlmostEqual(s.grasp_reference[0,3],.04)
                self.assertAlmostEqual(s.grasp_reference[0,0],math.cos(.05))
                self.assertTrue(s.captured)
                s.axial={}
                with self.assertRaisesRegex(ValueError,'REQUIRES_RIGID'):s.capture({'joint':0.},'right_axial')

    def test_collision_factory_omits_only_explicitly_absent_tool(self):
        from unittest.mock import patch
        init=next(n for n in next(n for n in ast.parse(SOURCE.read_text()).body if getattr(n,'name','')=='BimanualQP').body if getattr(n,'name','')=='__init__')
        begin=next(i for i,n in enumerate(init.body) if isinstance(n,ast.Assign) and isinstance(n.targets[0],ast.Attribute) and n.targets[0].attr=='collision_check')
        end=next(i for i,n in enumerate(init.body) if isinstance(n,ast.Assign) and isinstance(n.targets[0],ast.Attribute) and n.targets[0].attr=='limits')
        code=compile(ast.Module(body=init.body[begin:end],type_ignores=[]),str(SOURCE),'exec')
        config=dict(collision_scope='configured_tool_geometry',urdf='/fixture/robot.urdf',
                    demo=dict(display_urdf='display.urdf',grip_offset_m=[0.,0.,0.]))
        for policy in ('enforce','unheld_test'):
            owner=S(measured_grasp_policy=policy,tool=np.eye(4),close=Mock())
            with patch('bindu_kinematics.tool_geometry.make_collision_checker',return_value=Mock()) as factory:
                exec(code,dict(self=owner,config=config,np=np,Path=Path,math=math,__package__='bindu_kinematics'))
            self.assertIs(factory.call_args.kwargs['tool_present'],policy!='unheld_test')
            self.assertTrue(factory.call_args.kwargs['conservative_convex'])
            self.assertTrue(factory.call_args.kwargs['validation_only'])
            self.assertIsNotNone(owner.collision_check)

    def test_default_and_invalid_policy_fail_closed(self):
        # Missing policy keeps the existing guard; bad values reject before IO.
        s=QP.__new__(QP)
        with self.assertRaises(KeyError):QP.__init__(s,{ },None)
        self.assertEqual(s.measured_grasp_policy,'enforce')
        for value in (False,None,'disabled','unheld',[],1):
            with self.subTest(value=value),self.assertRaisesRegex(ValueError,'MEASURED_GRASP_POLICY'):
                QP({'measured_grasp_policy':value},None)

    def test_strict_policy_keeps_position_rotation_and_mixed_guards(self):
        for distance,angle in ((.0101,0.),(0.,.0201),(.04,0.)):
            s=self.solver(distance=distance,angle=angle)
            with self.subTest(distance=distance,angle=angle),self.assertRaisesRegex(ValueError,'MEASURED_GRASP_ERROR'):
                self.solve(s)
            self.assertFalse(s.captured);s.lib.bq_solve_tracking.assert_not_called()

    def test_unheld_mode_reaches_same_native_coupled_solve_and_collision_check(self):
        for distance,angle in ((.0101,0.),(0.,.0201),(.04,.05)):
            s=self.solver('unheld_test',distance,angle)
            out,_=self.solve(s);self.assertEqual(out,(.001,))
            s.lib.bq_solve_tracking.assert_called_once();s.checked_geometry_step.assert_called_once()

    def test_unheld_path_skips_grasp_closure_but_runs_cad_checker(self):
        for policy in ('enforce','unheld_test'):
            s=self.solver(policy,.04,.05);s.collision_check=Mock();s.collision_step=.01
            if policy=='enforce':
                with self.assertRaisesRegex(ValueError,'GRASP_CLOSURE'):s.check_path({'joint':0.},[.001])
            else:
                s.check_path({'joint':0.},[.001]);s.collision_check.assert_called_once()
                s.collision_check.side_effect=ValueError('SELF_COLLISION:fixture')
                with self.assertRaisesRegex(ValueError,'SELF_COLLISION'):s.check_path({'joint':0.},[.001])

    def test_actual_joint_limits_tracking_and_task_posture_still_reject(self):
        for feedback,reference,reason in ((1.001,1.001,'MEASURED_POSITION_LIMIT:joint'),
                                        (0.,.151,'REFERENCE_TRACKING_ERROR')):
            s=self.solver('unheld_test',.04,.05)
            with self.subTest(reason=reason),self.assertRaisesRegex(ValueError,reason):
                self.solve(s,{'joint':feedback},[reference])
        s=self.solver('unheld_test');s.posture_policy={};s.posture_lower=np.array([-.45]);s.posture_upper=np.array([.45])
        with self.assertRaisesRegex(ValueError,'MEASURED_POSTURE_ERROR'):
            self.solve(s,{'joint':.451},[.451])

    def test_collision_and_tool_observation_remain_independent(self):
        s=self.solver('unheld_test',.04,.05)
        s.checked_geometry_step.side_effect=ValueError('SELF_COLLISION:fixture')
        with self.assertRaisesRegex(ValueError,'SELF_COLLISION'):self.solve(s)
        self.assertFalse(s.captured)
        s=self.solver('unheld_test',.04,.05)
        s.cfg.update(max_tool_feedback_translation_m=.01,max_tool_feedback_rotation_rad=.02)
        observed=np.eye(4);observed[0,3]=.011
        with self.assertRaisesRegex(ValueError,'TOOL_FEEDBACK_DISAGREEMENT'):
            self.solve(s,tool_feedback=observed)

    def test_native_rejection_nonfinite_and_dynamic_limits_remain(self):
        for kind,reason in (('reject','BIMANUAL_QP:'),('nan','NONFINITE_RESULT'),('speed','POSITION_LIMIT:joint')):
            s=self.solver('unheld_test',.04,.05);s._error=lambda:'fixture'
            def native(*args):
                args[-2][:]=float('nan') if kind=='nan' else .02
                args[-1][:]=[1.,0.,1.];return 0 if kind=='reject' else 1
            s.lib.bq_solve_tracking.side_effect=native
            with self.subTest(kind=kind),self.assertRaisesRegex(ValueError,reason):self.solve(s)
            self.assertFalse(s.captured)

if __name__=='__main__':unittest.main()
