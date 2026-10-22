"""Production solve guard checks with a native-call substitute; no ROS/hardware."""
import ast
import ctypes as c
import math
from pathlib import Path
import time
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock
import numpy as np

SOURCE=Path(__file__).resolve().parents[2]/'src/capabilities/bindu_kinematics/bindu_kinematics/bimanual.py'
TREE=ast.parse(SOURCE.read_text())
# Load the production class without the ROS package locator; fixtures bypass
# native construction, and every solve executes the full production method.
TREE.body=[n for n in TREE.body if not (isinstance(n,ast.ImportFrom) and n.module=='ament_index_python.packages')]
NS={};exec(compile(TREE,str(SOURCE),'exec'),NS);QP=NS['BimanualQP']

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
