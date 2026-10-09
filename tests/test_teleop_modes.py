import unittest
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace as S
import numpy as np
from bindu_contracts.profile import Profile
from bindu_contracts.teleoperation import VRFrame, ToolPoseFrame
from bindu_teleoperation.modes import ModeSelection
from bindu_teleoperation.bimanual_session import BimanualSession, ToolPoseSession, AcceptedReference, ToolPause


ROOT = Path(__file__).resolve().parents[1]


class AcceptedReferenceTests(unittest.TestCase):
    def test_only_acceptance_advances_reference_and_elapsed_time(self):
        ref = AcceptedReference()
        ref.capture([.1, .2], 10.)
        self.assertEqual(ref.step(10.02)[0], (.1, .2))
        # A discarded or rejected proposal requires no state rollback.
        self.assertAlmostEqual(ref.step(10.04)[1], .04)
        ref.accept([.11, .21], 10.04)
        self.assertEqual(ref.step(10.06)[0], (.11, .21))
        self.assertAlmostEqual(ref.step(10.06)[1], .02)
        with self.assertRaisesRegex(ValueError, 'INTERVAL'):
            ref.accept([9., 9.], 10.02)
        self.assertEqual(ref.positions, (.11, .21))

    def test_clutch_reset_long_gap_and_clock_reversal_require_capture(self):
        ref = AcceptedReference()
        ref.capture([0.], 1.)
        for when in (1., .99, 1.101, float('nan')):
            with self.subTest(when=when), self.assertRaisesRegex(ValueError, 'INTERVAL'):
                ref.step(when)
        ref.reset()
        with self.assertRaisesRegex(ValueError, 'NOT_CAPTURED'):
            ref.step(1.02)
        ref.capture([.5], 2.)
        self.assertEqual(ref.step(2.02)[0], (.5,))


class ModeTests(unittest.TestCase):
    def setUp(self):
        self.profile = Profile.load(ROOT/'src/integration/bindu_runtime/config/g1_provisional_sim.json')
        self.m = ModeSelection(self.profile,'left',10.)
        self.m.begin('bimanual_tool','',1,11.)
        names = [n for group in self.profile.groups.values() for n in group]
        self.state = S(profile_hash=self.profile.digest,lease_active=False,state='IDLE',
                       reference=S(name=[]),stop_failures=[],feedback_stamp=11.1,
                       joints=S(name=names,velocity=[0.]*len(names)))
        self.health = S(ready=True,stamp=11.1)
        left=np.eye(4);right=np.eye(4);right[0,3]=.3
        self.frames = {side:VRFrame('source',1,11.1,side,tuple(pose.ravel()),grip=0.)
                       for side,pose in (('left',left),('right',right))}

    def ready(self):
        return self.m.ready(self.state,self.health,self.frames,11.15,tuple(self.profile.groups))

    def test_switch_requires_quiescence_and_fresh_paired_release(self):
        self.assertEqual(self.ready(),'')
        self.state.lease_active=True
        self.assertEqual(self.ready(),'EXECUTION_NOT_QUIESCENT')
        self.state.lease_active=False
        self.frames['right']=VRFrame('source',2,11.1,'right',self.frames['right'].pose,grip=0.)
        self.assertEqual(self.ready(),'VR_PAIR_MISMATCH')
        self.frames['right']=VRFrame('source',1,11.1,'right',self.frames['right'].pose,grip=1.)
        self.assertEqual(self.ready(),'VR_RELEASE_REQUIRED')

    def test_mapped_tool_selection_needs_only_fresh_pose_and_grip(self):
        self.m.tool_config = dict(input_frame='base', input_max_age=.25, right_axial_grasp={})
        frame = ToolPoseFrame('tool', 1, 11.1, 'base', tuple(np.eye(4).ravel()), True)
        def ready(f=frame):
            return self.m.ready(self.state, self.health, {}, 11.15,
                                tuple(self.profile.groups), tool_frame=f)
        self.assertEqual(ready(), '')
        self.assertEqual(ready(replace(frame, tight=False)), '')
        for bad in (None, replace(frame, stamp=11.), replace(frame, stop=True),
                    replace(frame, frame_id='wrong'), replace(frame, valid=False)):
            self.assertEqual(ready(bad), 'TOOL_INPUT_INVALID')
        self.state.lease_active = True
        self.assertEqual(ready(), 'EXECUTION_NOT_QUIESCENT')
        self.state.lease_active = False
        self.m.target = ('single_arm', 'left')
        self.assertEqual(ready(), 'VR_RELEASE_REQUIRED')

    def test_generation_fences_old_requests_and_failure_is_safe(self):
        self.assertEqual(self.ready(),'')
        self.m.finish(11.15)
        self.assertEqual((self.m.mode,self.m.phase,self.m.generation),('bimanual_tool','READY',2))
        with self.assertRaisesRegex(ValueError,'STALE_MODE_GENERATION'):
            self.m.begin('single_arm','left',1,11.2)
        self.m.begin('single_arm','left',2,11.2)
        self.m.fail('STOP_UNCONFIRMED',11.3)
        self.assertEqual((self.m.mode,self.m.phase,self.m.generation),('','SAFE',3))

    def test_mode_switch_rejects_stop_and_init_frames(self):
        for flag in ('stop', 'init'):
            with self.subTest(flag=flag):
                self.frames['left'] = replace(self.frames['left'], **{flag: True})
                self.assertEqual(self.ready(), 'VR_RELEASE_REQUIRED')
                self.frames['left'] = replace(self.frames['left'], **{flag: False})

    def test_mode_switch_uses_session_pair_geometry_and_timing(self):
        right = self.frames['right']
        self.frames['right'] = replace(right, stamp=11.07)
        self.assertEqual(self.ready(), 'VR_PAIR_MISMATCH')
        self.frames['right'] = right
        pose = np.asarray(right.pose).reshape(4, 4).copy()
        pose[:3, :3] = np.diag([1., -1., -1.])
        self.frames['right'] = replace(right, pose=tuple(pose.ravel()))
        self.assertEqual(self.ready(), 'VR_ORIENTATION_DEGENERATE')

    def test_left_orientation_readiness_matches_axial_session(self):
        right=self.frames['right'];pose=np.asarray(right.pose).reshape(4,4).copy()
        pose[:3,:3]=np.diag([1.,-1.,-1.])
        self.frames['right']=replace(right,pose=tuple(pose.ravel()))
        self.assertEqual(self.ready(),'VR_ORIENTATION_DEGENERATE')
        self.m.left_orientation=True
        self.assertEqual(self.ready(),'')

    def test_mode_switch_rejects_nonfinite_grip(self):
        self.frames['right'] = replace(self.frames['right'], grip=float('nan'))
        self.assertEqual(self.ready(), 'VR_RELEASE_REQUIRED')


class BimanualSessionTests(unittest.TestCase):
    def test_historical_clutch_counter_does_not_bypass_initial_release(self):
        cfg = dict(input_max_age=.25, engage_seconds=.2, position_scale=1., operator_yaw_rad=0.)
        session = BimanualSession(cfg, 2, 10.)
        for seq, when in ((1,10.1),(2,10.4)):
            frames=[]
            for side,x in (('left',0.),('right',.3)):
                pose=np.eye(4);pose[0,3]=x
                frames.append(VRFrame('source',seq,when,side,tuple(pose.ravel()),
                                      grip=1.,clutch_seq=8))
            session.ingest(*frames,when)
            self.assertEqual(session.mode,'idle')
            self.assertFalse(session.released)
        with self.assertRaisesRegex(ValueError,'BIMANUAL_NOT_ENGAGED'):
            session.capture(np.eye(4))

    def test_coalesced_release_from_either_hand_requires_new_capture(self):
        cfg = dict(input_max_age=.25, engage_seconds=.2, position_scale=1., operator_yaw_rad=0.)
        for released_side in ('left', 'right'):
            with self.subTest(side=released_side):
                session = BimanualSession(cfg, 2, 10.)
                def frames(seq, when, grip, offset=0., clutch=0):
                    result=[]
                    for side, x in (('left', 0.), ('right', .3)):
                        pose=np.eye(4); pose[0,3]=x+offset
                        result.append(VRFrame('source',seq,when,side,tuple(pose.ravel()),
                            grip=grip,clutch_seq=clutch if side==released_side else 0))
                    return result
                for seq, when, grip in ((1,10.1,0.),(2,10.2,1.),(3,10.41,1.)):
                    session.ingest(*frames(seq,when,grip),when)
                session.capture(np.eye(4))
                session.ingest(*frames(5,10.43,1.,.2,1),10.43)
                self.assertEqual(session.mode,'idle')
                self.assertIsNone(session.target)
                self.assertIsNone(session.anchor_tool)
                with self.assertRaisesRegex(ValueError,'BIMANUAL_NOT_ENGAGED'):
                    session.capture(np.eye(4))
                session.ingest(*frames(6,10.64,1.,.2,1),10.64)
                self.assertEqual(session.mode,'follow')
                self.assertIsNone(session.target)
                session.capture(np.eye(4))
                np.testing.assert_allclose(session.target,np.eye(4),atol=1e-12)
                with self.assertRaisesRegex(ValueError,'VR_CLUTCH_UNORDERED'):
                    session.ingest(*frames(7,10.65,1.,.2,0),10.65)

    def test_two_hand_release_engage_and_new_anchor(self):
        cfg={'input_max_age':.25,'engage_seconds':.2,'position_scale':1.,'operator_yaw_rad':0.}
        session=BimanualSession(cfg,2,10.)
        left=np.eye(4);right=np.eye(4);right[0,3]=.3
        def frames(seq,when,grip,offset=0.):
            a=left.copy();b=right.copy();a[0,3]+=offset;b[0,3]+=offset
            return (VRFrame('source',seq,when,'left',tuple(a.ravel()),grip=grip),
                    VRFrame('source',seq,when,'right',tuple(b.ravel()),grip=grip))
        session.ingest(*frames(1,10.1,0.),10.1)
        session.ingest(*frames(2,10.2,1.),10.2)
        session.ingest(*frames(3,10.41,1.),10.41)
        self.assertEqual(session.mode,'follow')
        session.capture(np.eye(4))
        session.ingest(*frames(4,10.42,1.,.01),10.42)
        self.assertAlmostEqual(float(np.linalg.norm(session.target[:3,3])),.01)
        session.ingest(*frames(5,10.43,0.),10.43)
        self.assertEqual(session.mode,'idle')
        self.assertIsNone(session.target)
        with self.assertRaisesRegex(ValueError,'VR_PAIR_INVALID'):
            session.ingest(*frames(6,10.44,1.)[:1],frames(5,10.44,1.)[1],10.44)


class AxialTriggerTests(unittest.TestCase):
    def setUp(self):
        self.session=BimanualSession(dict(input_max_age=.25,engage_seconds=.05,
            position_scale=1.,operator_yaw_rad=0.,right_axial_grasp={}),2,10.)
        self.seq=0

    def frame(self,when,grip=1.,trigger=0.,left_angle=0.,right_angle=0.):
        self.seq+=1
        from scipy.spatial.transform import Rotation
        frames=[]
        for side,x,angle in (('left',0.,left_angle),('right',.3,right_angle)):
            pose=np.eye(4);pose[0,3]=x
            pose[:3,:3]=Rotation.from_rotvec([angle,0.,0.]).as_matrix()
            frames.append(VRFrame('input',self.seq,when,side,tuple(pose.ravel()),
                grip=grip,trigger=trigger if side=='right' else 0.))
        return frames

    def engage(self):
        for t,grip in ((10.1,0.),(10.2,1.),(10.3,1.)):
            self.session.ingest(*self.frame(t,grip),t)
        self.session.capture(np.eye(4))

    def test_trigger_requires_release_then_hold_and_hysteresis(self):
        self.session.ingest(*self.frame(10.1,trigger=1.),10.1)
        self.assertEqual(self.session.grasp_mode,'rigid')
        self.engage()
        self.session.ingest(*self.frame(10.4,trigger=1.),10.4)
        self.assertEqual(self.session.grasp_mode,'right_axial')
        self.assertIsNone(self.session.target)
        generation=self.session.grasp_generation
        self.session.ingest(*self.frame(10.41,trigger=.5),10.41)
        self.assertEqual(self.session.grasp_generation,generation)
        self.session.capture(np.eye(4))
        self.session.ingest(*self.frame(10.42,trigger=0.),10.42)
        self.assertEqual(self.session.grasp_mode,'rigid')
        self.assertIsNone(self.session.target)
        self.assertEqual(self.session.grasp_generation,generation+1)

    def test_right_wrist_orientation_does_not_command_tool_roll(self):
        self.engage()
        self.session.ingest(*self.frame(10.4,trigger=1.),10.4)
        self.session.capture(np.eye(4))
        self.session.ingest(*self.frame(10.41,trigger=1.,right_angle=2.9),10.41)
        np.testing.assert_allclose(self.session.target,np.eye(4),atol=1e-12)
        self.session.ingest(*self.frame(10.42,trigger=1.,left_angle=.2,right_angle=2.9),10.42)
        self.assertGreater(np.linalg.norm(self.session.target[:3,:3]-np.eye(3)),.1)
        self.session.ingest(*self.frame(10.43,grip=0.,trigger=1.),10.43)
        self.assertEqual(self.session.mode,'idle')
        self.assertIsNone(self.session.target)

    def test_invalid_trigger_is_rejected_and_stop_reanchor_uses_fresh_input(self):
        self.engage()
        with self.assertRaisesRegex(ValueError,'VR_PAIR_INVALID'):
            self.session.ingest(*self.frame(10.4,trigger=float('nan')),10.4)
        self.session.reanchor()
        with self.assertRaisesRegex(ValueError,'NOT_ENGAGED'):
            self.session.capture(np.eye(4))
        self.session.ingest(*self.frame(10.41,left_angle=.5),10.41)
        self.session.capture(np.eye(4))
        np.testing.assert_allclose(self.session.target,np.eye(4),atol=1e-12)


class ToolPoseInputTests(unittest.TestCase):
    def setUp(self):
        self.cfg={'input_frame':'base','input_max_age':.25,'right_axial_grasp':{}}
        self.session=ToolPoseSession(self.cfg,1,0.)

    def frame(self, seq=1, stamp=.1, tight=True):
        pose=np.eye(4);pose[0,3]=.3
        return ToolPoseFrame('source',seq,stamp,'base',tuple(pose.ravel()),tight)

    def test_absolute_target_survives_grasp_stop_and_recapture(self):
        s=self.session;s.ingest(self.frame(),.1)
        s.capture(np.eye(4))
        first=s.target.copy()
        s.ingest(self.frame(2,.2,False),.2)
        self.assertEqual(s.grasp_mode,'right_axial')
        self.assertEqual(s.grasp_generation,1)
        self.assertIsNone(s.anchor_tool)
        shifted=np.eye(4);shifted[0,3]=.2
        s.capture(shifted)
        np.testing.assert_array_equal(s.target,first)
        s.ingest(self.frame(3,.3,True),.3)
        self.assertEqual(s.grasp_mode,'rigid')
        self.assertEqual(s.grasp_generation,2)

    def test_invalid_stale_wrong_frame_stop_and_connection_change_fail(self):
        f=self.frame()
        for change in ({'pose':tuple([float('nan')]*16)},{'stamp':-.1},
                       {'stamp':.2},{'frame_id':'wrong'},{'tight':1},
                       {'valid':False},{'stop':True},{'seq':-1}):
            with self.subTest(change=change),self.assertRaises(ValueError):
                self.session.ingest(replace(f,**change),.1)
        self.session.ingest(f,.1)
        with self.assertRaisesRegex(ValueError,'CONNECTION_CHANGED'):
            self.session.ingest(replace(f,source_id='another',seq=2),.1)
        self.assertFalse(self.session.ingest(f,.1))


class ToolPauseTests(unittest.TestCase):
    def test_resume_requires_stop_and_post_resume_input(self):
        p=ToolPause()
        p.request(True,1.)
        self.assertFalse(p.accepts(2.))
        with self.assertRaisesRegex(ValueError,'NOT_CONFIRMED'):
            p.request(False,2.)
        p.confirm_stopped()
        p.request(True,2.1)  # Repeated held pause must not initiate another stop.
        self.assertEqual(p.phase,'paused')
        p.request(False,3.)
        for stamp in (2.9,3.,float('nan')):
            self.assertFalse(p.accepts(stamp))
        self.assertTrue(p.accepts(3.01))
        p.request(True,3.02)
        self.assertFalse(p.accepts(3.03))

    def test_feedback_keeps_pause_and_distinguishes_virtual_grasp(self):
        from bindu_teleoperation.feedback import ToolFeedback
        f=ToolFeedback(dict(input_max_age=.25,feedback_max_age=.2,command_max_age=.4,input_frame='base'))
        f.event('BIMANUAL_STARTED','{}','test',1.)
        f.event('BIMANUAL_CAPTURED','{"grasp_mode":"rigid"}','test',1.1)
        f.event('BIMANUAL_QP_RESULT','{"target":'+str(np.eye(4).ravel().tolist())+',"input_stamp":1.2}', 'test',1.2)
        f.event('BIMANUAL_PAUSED','STOP_CONFIRMED','test',1.3)
        view=f.snapshot(3.,list(np.eye(4).ravel()),3.,1.2,False)
        self.assertEqual(view['mode'],'PAUSED')
        self.assertEqual(view['reason'],'STOP_CONFIRMED')
        self.assertIsNone(view['target'])
        self.assertIsNone(view['physical_gripper_confirmed'])
        self.assertEqual(view['grasp_applied'],'rigid')
        stale=f.snapshot(4.,None,3.,1.2,False)
        self.assertEqual(stale['reason'],'BIMANUAL_FEEDBACK_STALE')
