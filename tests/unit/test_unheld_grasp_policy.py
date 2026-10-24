"""Production solve/contact guard checks; no ROS or hardware."""
import ast
import ctypes as c
import math
from pathlib import Path
import time
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock
import numpy as np
from bindu_kinematics.tool_geometry import _shaft_contact_span

SOURCE=Path(__file__).resolve().parents[2]/'src/capabilities/bindu_kinematics/bindu_kinematics/bimanual.py'
TREE=ast.parse(SOURCE.read_text())
# Load the production class without the ROS package locator; fixtures bypass
# native construction, and every solve executes the full production method.
TREE.body=[n for n in TREE.body if not (isinstance(n,ast.ImportFrom) and n.module=='ament_index_python.packages')]
NS={};exec(compile(TREE,str(SOURCE),'exec'),NS);QP=NS['BimanualQP']


class ShaftContactBandTests(unittest.TestCase):
    ends = (-.44, .1)

    def test_default_keeps_strict_endpoint_check(self):
        np.testing.assert_allclose(_shaft_contact_span(self.ends, -.34, .1), (-.44, -.24))
        with self.assertRaisesRegex(ValueError, 'INVALID_SHAFT_CONTACT_BAND'):
            _shaft_contact_span(self.ends, -.3432, .1)

    def test_recorded_recenterings_clip_only_the_outside_part(self):
        import json
        config=Path(__file__).resolve().parents[2]/'src/integration/bindu_runtime/config/robots/snowbot/bimanual_snowbot_real.json'
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

    def test_unheld_capture_uses_actual_relative_pose_without_nominal_grasp_gate(self):
        for policy in ('enforce','unheld_test'):
            s=self.solver(policy,.04,.05);s.handle=object();s.axial=None
            s.cfg.update(flange_distance_range=[0.,1.],left_to_tool=np.eye(4).ravel().tolist())
            s.tool=np.eye(4);s.measured_rigid_grasp=False;s.native_config=NS['NativeConfig']()
            s.tracking_point=np.zeros(3);s.posture_reference=None;s.collision_check=None
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
