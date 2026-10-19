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
    def test_capture_requires_new_input_without_moving_fence_on_acceptance(self):
        ref=AcceptedReference()
        # Field failure: frame .545738 preceded stop .549435 and capture .557080.
        ref.capture([0.],10.557080)
        for stamp in (10.545738,10.549435,10.557080,float('nan')):
            self.assertFalse(ref.permits_input(stamp))
        self.assertTrue(ref.permits_input(10.56))
        ref.accept([.01],10.58)
        self.assertTrue(ref.permits_input(10.56))
        ref.reset();self.assertFalse(ref.permits_input(10.6))
        ref.capture([.01],10.7)
        self.assertFalse(ref.permits_input(10.6))
        self.assertTrue(ref.permits_input(10.71))

    def test_deployment_tolerates_205ms_gap_without_integrating_more_than_100ms(self):
        import json
        config=json.loads((ROOT/'src/integration/bindu_runtime/config/robots/snowbot/bimanual_snowbot_real.json').read_text())
        ref=AcceptedReference(config['max_integration_interval'],max_sample_gap=config['integration_max_sample_gap'])
        self.assertEqual(ref.max_sample_gap,.25)
        self.assertLessEqual(ref.max_sample_gap,config['input_max_age'])
        ref.capture((0.,),10.)
        self.assertEqual(ref.step(10.205),((0.,),.1))
        with self.assertRaisesRegex(ValueError,'INTEGRATION_INTERVAL'):ref.step(10.251)

    def test_200ms_gap_keeps_100ms_step_and_rejects_longer_stall(self):
        ref = AcceptedReference(.1, max_sample_gap=.2)
        ref.capture((0.,), 10.)
        self.assertEqual(ref.step(10.18), ((0.,), .1))
        ref.accept((.01,), 10.18)
        with self.assertRaisesRegex(ValueError, 'INTEGRATION_INTERVAL'):
            ref.step(10.381)
        self.assertEqual(ref.positions, (.01,))
        self.assertEqual(ref.sampled_at, 10.18)

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

    def test_explicit_sample_gap_budget_caps_integration_without_advancing_time(self):
        ref=AcceptedReference(.1,max_sample_gap=.25)
        ref.capture([.2],1.)
        self.assertAlmostEqual(ref.step(1.18)[1],.1)
        self.assertEqual(ref.sampled_at,1.)
        ref.accept([.21],1.18)
        self.assertAlmostEqual(ref.step(1.21)[1],.03)
        for when in (1.18,1.17,1.431,float('nan')):
            with self.subTest(when=when),self.assertRaisesRegex(ValueError,'INTEGRATION_INTERVAL'):
                ref.step(when)
        self.assertEqual(ref.positions,(.21,))
        with self.assertRaisesRegex(ValueError,'SAMPLE_GAP_CONFIG'):
            AcceptedReference(.1,max_sample_gap=.3)


class ModeTests(unittest.TestCase):
    def setUp(self):
        self.profile = Profile.load(ROOT/'src/integration/bindu_runtime/config/robots/g1/g1_provisional_sim.json')
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

    def test_explicit_no_recording_keeps_feedback_and_stop_checks(self):
        self.health=None
        self.assertEqual(self.ready(),'RECORDER_UNAVAILABLE')
        self.m.require_recording=False
        self.assertEqual(self.ready(),'')
        self.state.stop_failures=['unconfirmed']
        self.assertEqual(self.ready(),'EXECUTION_NOT_QUIESCENT')

    def test_dual_only_starts_unselected_and_rejects_single_arm(self):
        m=ModeSelection(self.profile,'left',10.,require_recording=False,enabled_modes=('bimanual_tool',))
        self.assertEqual((m.mode,m.phase,m.code),('','SAFE','MODE_NOT_SELECTED'))
        with self.assertRaisesRegex(ValueError,'INVALID_MODE_SELECTION'):
            m.begin('single_arm','left',m.generation,11.)
        self.assertTrue(m.begin('bimanual_tool','',m.generation,11.))

    def test_independent_mode_checks_each_pose_without_tool_hand_spacing(self):
        m=ModeSelection(self.profile,'left',10.,require_recording=False,
                        enabled_modes=('independent_arms','bimanual_tool'))
        m.begin('independent_arms','',m.generation,11.)
        frames={side:replace(f,pose=tuple(np.eye(4).ravel())) for side,f in self.frames.items()}
        self.assertEqual(m.ready(self.state,None,frames,11.15,tuple(self.profile.groups)),'')
        frames['left']=replace(frames['left'],grip=1.)
        self.assertEqual(m.ready(self.state,None,frames,11.15,tuple(self.profile.groups)),'VR_RELEASE_REQUIRED')

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

    def test_mode_switch_rejects_invalid_buttons_and_frame_identity(self):
        right = self.frames['right']
        for change in ({'grip': -.1}, {'grip': 1.1}, {'trigger': float('nan')},
                       {'trigger': -.1}, {'trigger': 1.1}, {'side': 'left'},
                       {'seq': -1}, {'seq': True}, {'seq': 1.5},
                       {'clutch_seq': -1}, {'clutch_seq': True}, {'stamp': 11.}):
            with self.subTest(change=change):
                self.frames['right'] = replace(right, **change)
                self.assertEqual(self.ready(), 'VR_RELEASE_REQUIRED')
        self.frames['right'] = right
        self.assertEqual(self.ready(), '')


class BimanualSessionTests(unittest.TestCase):
    def pair(self, sequence, when, **changes):
        frames = []
        for side, x in (('left', 0.), ('right', .3)):
            pose = np.eye(4); pose[0,3] = x
            frames.append(replace(VRFrame('source', sequence, when, side, tuple(pose.ravel())),
                                  **changes))
        return frames

    def session(self):
        return BimanualSession(dict(input_max_age=.25, engage_seconds=.2,
            position_scale=1., operator_yaw_rad=0., right_axial_grasp={}), 2, 10.)

    def test_invalid_pair_metadata_is_rejected(self):
        for change in ({'seq': -1}, {'seq': True}, {'seq': 1.5},
                       {'clutch_seq': True}, {'clutch_seq': -1}, {'side': 'left'}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.session().ingest(*self.pair(1, 10.1, **change), 10.1)

    def test_new_sequence_cannot_move_either_hand_clock_backwards(self):
        session = self.session()
        session.ingest(*self.pair(1, 10.11), 10.12)
        for side in (0, 1):
            frames = self.pair(2, 10.115)
            frames[side] = replace(frames[side], stamp=10.105)
            with self.subTest(side=side), self.assertRaisesRegex(ValueError, 'VR_INPUT_UNORDERED'):
                session.ingest(*frames, 10.12)
            self.assertEqual(session.seq, 1)
        self.assertTrue(session.ingest(*self.pair(2, 10.12), 10.12))

    def test_invalid_pose_does_not_change_clutch_or_grasp(self):
        session = self.session()
        session.ingest(*self.pair(1, 10.1), 10.1)
        frames = self.pair(2, 10.2, grip=1., trigger=1., clutch_seq=1)
        frames[1] = replace(frames[1], pose=tuple([float('nan')]*16))
        with self.assertRaises(ValueError):
            session.ingest(*frames, 10.2)
        self.assertEqual((session.seq, session.clutch_seq, session.grasp_generation),
                         (1, (0, 0), 0))
        self.assertTrue(session.ingest(*self.pair(2, 10.2), 10.2))

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


class RelativeRodMappingTests(unittest.TestCase):
    def test_relative_common_roll_cancels_different_initial_wrists_and_unwraps_half_turn(self):
        from scipy.spatial.transform import Rotation
        hands=[np.eye(4),np.eye(4)]
        for hand,x,angles in zip(hands,(.17,-.17),((.3,.5,-.2),(-.4,.2,.7))):
            hand[0,3]=x
            hand[:3,:3]=Rotation.from_euler('xyz',angles).as_matrix()
        home,_=BimanualSession.paired_rod_pose(hands)
        swing=Rotation.from_euler('z',.4).as_matrix()
        axis=swing @ np.array([1.,0.,0.]);rolls=(0.,0.);previous=None
        for angle in np.linspace(0.,3.5,71):
            rotation=Rotation.from_rotvec(axis*angle).as_matrix() @ swing
            current=[]
            for hand in hands:
                pose=hand.copy();pose[:3,3]=swing @ hand[:3,3]
                pose[:3,:3]=rotation @ hand[:3,:3];current.append(pose)
            pose,rolls=BimanualSession.paired_rod_pose(current,hands,home[:3,:3],rolls)
            np.testing.assert_allclose(pose[:3,:3],rotation @ home[:3,:3],atol=1e-10)
            np.testing.assert_allclose(pose[:3,0],axis,atol=1e-10)
            np.testing.assert_allclose(rolls,[angle,angle],atol=1e-10)
            if previous is not None:
                self.assertLess(Rotation.from_matrix(pose[:3,:3] @ previous.T).magnitude(),.051)
            previous=pose[:3,:3]

    def setUp(self):
        self.session=BimanualSession(dict(input_max_age=.25,engage_seconds=.05,
            position_scale=.5,operator_yaw_rad=0.,right_axial_grasp={}),1,0.)
        self.seq=0

    def feed(self,t,held=True,offset=(0.,0.,0.),angle=0.,clutch=0):
        from scipy.spatial.transform import Rotation
        from bindu_teleoperation.vr.mapping import OPENXR_TO_ROBOT
        self.seq+=1
        frames=[]
        for side,y in (('left',.17),('right',-.17)):
            pose=np.eye(4);pose[:3,:3]=Rotation.from_rotvec([0.,0.,angle]).as_matrix()
            pose[:3,3]=np.array(offset)+[0.,y,0.]
            raw=OPENXR_TO_ROBOT.T @ pose @ OPENXR_TO_ROBOT
            frames.append(VRFrame('pico',self.seq,t,side,tuple(raw.ravel()),grip=float(held),
                clutch_seq=clutch,mapping='pico_snowbot_v1'))
        self.session.ingest(*frames,t)

    def engage(self,t,home,offset=(0.,0.,0.),angle=0.,clutch=0):
        self.feed(t,False,offset,angle,clutch)
        self.feed(t+.01,True,offset,angle,clutch)
        self.feed(t+.07,True,offset,angle,clutch)
        self.session.capture(home)

    def test_all_axes_are_half_scale_and_rotation_does_not_translate_midpoint(self):
        home=np.eye(4);home[:3,3]=[.6,-.1,.7]
        self.engage(.1,home)
        self.feed(.18,offset=(.1,-.06,.04),angle=.2)
        np.testing.assert_allclose(self.session.target[:3,3],home[:3,3]+[.05,-.03,.02],atol=1e-12)
        from scipy.spatial.transform import Rotation
        self.assertAlmostEqual(Rotation.from_matrix(self.session.target[:3,:3]).magnitude(),.2)

    def test_worker_capture_delay_does_not_replay_hand_motion(self):
        home=np.eye(4)
        self.feed(.1,False);self.feed(.11);self.feed(.18)
        self.feed(.2,offset=(.08,.02,0.),angle=.2)
        self.session.capture(home)
        np.testing.assert_allclose(self.session.target,home,atol=1e-12)
        self.feed(.21,offset=(.10,.02,0.),angle=.2)
        np.testing.assert_allclose(self.session.target[:3,3],[.01,0.,0.],atol=1e-12)

    def test_rod_direction_uses_both_positions_and_roll_uses_both_relative_wrists(self):
        from bindu_teleoperation.vr.mapping import OPENXR_TO_ROBOT
        from scipy.spatial.transform import Rotation
        self.session.cfg['two_hand_rod']=True
        self.session.cfg['rod_axis_in_tool']=[1.,0.,0.]
        def feed(t,angle=0.,left_roll=0.,right_roll=0.,held=True):
            self.seq+=1;frames=[]
            swing=Rotation.from_euler('z',angle).as_matrix()
            for side,x,roll in (('left',.17,left_roll),('right',-.17,right_roll)):
                pose=np.eye(4);pose[:3,3]=swing @ [x,0.,0.]
                pose[:3,:3]=swing @ Rotation.from_euler('x',roll).as_matrix()
                raw=OPENXR_TO_ROBOT.T @ pose @ OPENXR_TO_ROBOT
                frames.append(VRFrame('pico',self.seq,t,side,tuple(raw.ravel()),grip=float(held)))
            self.session.ingest(*frames,t)
        feed(.1,held=False);feed(.11);feed(.18);self.session.capture(np.eye(4))
        feed(.19,angle=.15)
        np.testing.assert_allclose(self.session.target[:3,:3],Rotation.from_euler('z',.15).as_matrix(),atol=1e-12)
        np.testing.assert_allclose(self.session.target[:3,3],0.,atol=1e-12)
        feed(.2,angle=.15,left_roll=.2,right_roll=.2)
        np.testing.assert_allclose(self.session.target[:3,:3],
            Rotation.from_euler('z',.15).as_matrix() @ Rotation.from_euler('x',.2).as_matrix(),atol=1e-12)
        feed(.21,angle=.15,left_roll=.4,right_roll=0.)
        np.testing.assert_allclose(self.session.target[:3,:3],
            Rotation.from_euler('z',.15).as_matrix() @ Rotation.from_euler('x',.2).as_matrix(),atol=1e-12)
        feed(.22,angle=.15,left_roll=.3,right_roll=-.3)
        np.testing.assert_allclose(self.session.target[:3,:3],Rotation.from_euler('z',.15).as_matrix(),atol=1e-12)
        # Human and robot rods need not have the same direction at engagement.
        # Shared twist must spin around the robot rod, never tilt that axis.
        self.session.cfg['rod_axis_in_tool']=[0.,0.,1.]
        self.session.reanchor();feed(.23);self.session.capture(np.eye(4))
        feed(.24,left_roll=.3,right_roll=.3)
        np.testing.assert_allclose(self.session.target[:3,2],[0.,0.,1.],atol=1e-12)
        np.testing.assert_allclose(self.session.target[:3,:3],Rotation.from_euler('z',.3).as_matrix(),atol=1e-12)
        feed(.25,angle=.15)
        self.assertAlmostEqual(np.arccos(np.clip(self.session.target[2,2],-1.,1.)),.15)

    def test_repeated_release_reposition_and_reengage_use_current_measured_home(self):
        home=np.eye(4)
        for i in range(4):
            t=.1+i
            offset=np.array([i*.1,-i*.02,i*.01]);angle=i*.25
            home=home.copy();home[:3,3]+=[.01,.005,0.]
            self.engage(t,home,offset,angle,i)
            np.testing.assert_allclose(self.session.target,home,atol=1e-12)
            self.feed(t+.08,offset=offset+[.02,0.,0.],angle=angle,clutch=i)
            np.testing.assert_allclose(self.session.target[:3,3],home[:3,3]+[.01,0.,0.],atol=1e-12)
            self.feed(t+.09,False,offset+[.2,.1,0.],angle+.4,i+1)
            self.assertIsNone(self.session.target)


class DeployedToolMappingTests(unittest.TestCase):
    def setUp(self):
        import json
        self.config=json.loads((ROOT/'src/integration/bindu_runtime/config/robots/snowbot/bimanual_snowbot_real.json').read_text())
        self.config['engage_seconds']=.05
        self.config['pico_grasp_resume_seconds']=.04
        self.session=BimanualSession(self.config,1,0.)
        self.seq=0

    def feed(self,t,angle=0.,offset=(0.,0.,0.),held=True,clutch=0):
        from scipy.spatial.transform import Rotation
        from bindu_teleoperation.vr.mapping import OPENXR_TO_ROBOT
        swing=Rotation.from_euler('z',angle).as_matrix()
        self.seq+=1;frames=[]
        for side,x in (('left',.17),('right',-.17)):
            # Change the line direction while both wrist orientations stay fixed.
            pose=np.eye(4);pose[:3,3]=swing @ [x,0.,0.]+np.array(offset)
            raw=OPENXR_TO_ROBOT.T @ pose @ OPENXR_TO_ROBOT
            frames.append(VRFrame('pico',self.seq,t,side,tuple(raw.ravel()),grip=float(held),
                                  clutch_seq=clutch,mapping='pico_snowbot_v1'))
        self.session.ingest(*frames,t)

    def engage(self,t,home,**kwargs):
        self.feed(t,held=False,**kwargs);self.feed(t+.01,**kwargs);self.feed(t+.07,**kwargs)
        self.session.capture(home)
        np.testing.assert_allclose(self.session.target,home,atol=1e-12)

    def test_shared_recipe_moves_rod_axis_without_wrist_rotation_and_keeps_grip_midpoint(self):
        from scipy.spatial.transform import Rotation
        home=np.array(self.config['left_to_tool']).reshape(4,4)
        self.engage(.1,home)
        point=self.session.control_point_in_tool
        midpoint=home[:3,3]+home[:3,:3] @ point
        initial_axis=home[:3,:3] @ np.array(self.session.cfg['rod_axis_in_tool'])
        for i,angle in enumerate((.15,.4,-.2)):
            self.feed(.18+i*.01,angle)
            target=self.session.target
            self.assertAlmostEqual(Rotation.from_matrix(target[:3,:3] @ home[:3,:3].T).magnitude(),abs(angle))
            self.assertAlmostEqual(np.arccos(np.clip(initial_axis @ (target[:3,:3] @ np.array(self.session.cfg['rod_axis_in_tool'])),-1.,1.)),abs(angle))
            np.testing.assert_allclose(target[:3,3]+target[:3,:3] @ point,midpoint,atol=1e-12)

    def test_translation_and_recapture_follow_current_midpoint_without_a_jump(self):
        home=np.eye(4);home[:3,3]=[.1,.2,.4]
        self.engage(.1,home)
        self.feed(.18,.3,offset=(.02,-.01,.03))
        point=self.session.control_point_in_tool
        np.testing.assert_allclose(self.session.target[:3,3]+self.session.target[:3,:3] @ point,
                                   home[:3,3]+point+[.02,-.01,.03],atol=1e-12)
        self.feed(.19,.4,offset=(.1,.2,0.),held=False,clutch=1)
        self.assertIsNone(self.session.target)
        home=np.eye(4)
        self.engage(.3,home,angle=.4,offset=(.1,.2,0.),clutch=1)
        np.testing.assert_allclose(self.session.target,home,atol=1e-12)

    def test_mapping_and_degenerate_mounts_fail_before_input(self):
        with self.assertRaisesRegex(ValueError,'VR_TOOL_MAPPING_CONFIG'):
            BimanualSession(dict(self.config,vr_tool_mapping='unknown'),1,0.)
        broken=dict(self.config,right_to_tool=self.config['left_to_tool'])
        broken['demo']=dict(self.config['demo'],right_grip_offset_m=self.config['demo']['grip_offset_m'])
        with self.assertRaisesRegex(ValueError,'VR_TOOL_GRIP_GEOMETRY'):
            BimanualSession(broken,1,0.)


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
