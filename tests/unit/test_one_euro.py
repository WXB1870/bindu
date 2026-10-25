from tests.support.paths import ROOT
import copy
import json
import unittest
from pathlib import Path
import numpy as np
from bindu_contracts.teleoperation import VRFrame, IKResult
from bindu_teleoperation.session import TeleopSession
from bindu_teleoperation.vr.mapping import robot_pose
from bindu_teleoperation.vr.one_euro import OneEuroPose, rotation_exp, rotation_log, validate_config
from bindu_teleoperation.independent_session import IndependentArmsSession
from bindu_teleoperation.bimanual_session import BimanualSession
from bindu_teleoperation.vr.mapping import OPENXR_TO_ROBOT
from dataclasses import replace

def config():
    return json.loads((ROOT/'src/integration/bindu_runtime/config/robots/g1/teleop_g1_left.json').read_text())


class OneEuroTests(unittest.TestCase):
    def test_recorded_history_roundtrip_preserves_future_motion_after_clock_rebase(self):
        f=OneEuroPose(config()['pose_filter'])
        for i in range(20):
            pose=np.eye(4);pose[0,3]=.01*i
            pose[:3,:3]=rotation_exp(np.array([.01,.02,0.])*i)
            f.update(pose,1.+i*.01)
        state=json.loads(json.dumps(f.snapshot()))
        restored=OneEuroPose(config()['pose_filter']);restored.restore(state,stamp=11.19)
        for i in range(20,30):
            pose[0,3]=.01*i;pose[:3,:3]=rotation_exp(np.array([.01,.02,0.])*i)
            np.testing.assert_allclose(f.update(pose,1.+i*.01),restored.update(pose,11.+i*.01),atol=1e-12)
        for bad in (dict(state,velocity=[0.,0.]),dict(state,stamp=float('nan'))):
            with self.assertRaisesRegex(ValueError,'INVALID_FILTER_STATE'):restored.restore(bad)
    def test_same_measurement_time_holds_filter_without_advancing_state(self):
        f=OneEuroPose(config()['pose_filter']);p=np.eye(4);f.update(p,1.)
        p[0,3]=.1
        np.testing.assert_array_equal(f.update(p,1.),np.eye(4))
        self.assertEqual(f.stamp,1.)
        np.testing.assert_array_equal(f.velocity,np.zeros(3))
        with self.assertRaisesRegex(ValueError,'TIME_ORDER'):f.update(p,.99)
    def test_first_sample_passthrough_and_known_fixed_cutoff(self):
        cfg=config()['pose_filter'];cfg['position']['beta']=0
        f=OneEuroPose(cfg);p=np.eye(4);np.testing.assert_equal(f.update(p,1.),p)
        p[0,3]=1.;out=f.update(p,1.02)
        expected=1/(1+1/(2*np.pi*4*.02))
        self.assertAlmostEqual(out[0,3],expected)
        f.reset();np.testing.assert_equal(f.update(p,2.),p)

    def test_stationary_jitter_is_reduced(self):
        f=OneEuroPose(config()['pose_filter']);rng=np.random.default_rng(7);raw=[];filtered=[]
        for i in range(400):
            p=np.eye(4);p[:3,3]=rng.normal(0,.003,3)
            out=f.update(p,1+i/90)
            if i>50:raw.append(p[:3,3]);filtered.append(out[:3,3])
        self.assertLess(np.sqrt(np.mean(np.square(filtered))),.6*np.sqrt(np.mean(np.square(raw))))

    def test_adaptation_reduces_fast_motion_lag(self):
        cfg=config()['pose_filter'];fixed=copy.deepcopy(cfg);fixed['position']['beta']=0
        adaptive,slow=OneEuroPose(cfg),OneEuroPose(fixed);a=[];b=[]
        for i in range(200):
            p=np.eye(4);p[0,3]=i/90
            fa=adaptive.update(p,1+i/90);fb=slow.update(p,1+i/90)
            if i>100:a.append(p[0,3]-fa[0,3]);b.append(p[0,3]-fb[0,3])
        self.assertLess(np.mean(a),.6*np.mean(b))

    def test_rotations_cross_pi_without_long_way_or_nonrigid_matrix(self):
        f=OneEuroPose(config()['pose_filter']);p=np.eye(4)
        p[:3,:3]=rotation_exp(np.array([0,0,np.deg2rad(179)]));initial=p[:3,:3].copy();f.update(p,1.)
        p[:3,:3]=rotation_exp(np.array([0,0,np.deg2rad(-179)]));out=f.update(p,1.02)
        delta=rotation_log(out[:3,:3]@initial.T)
        self.assertGreater(delta[2],0);self.assertLess(delta[2],np.deg2rad(2))
        for i in range(300):
            p[:3,:3]=rotation_exp(np.array([.3,1.,-.2])*(i/100))
            out=f.update(p,1.04+i*.013)
            np.testing.assert_allclose(out[:3,:3].T@out[:3,:3],np.eye(3),atol=1e-10)
            self.assertAlmostEqual(np.linalg.det(out[:3,:3]),1.)
        for axis in np.eye(3):
            np.testing.assert_allclose(rotation_exp(rotation_log(rotation_exp(axis*np.pi))),rotation_exp(axis*np.pi),atol=1e-10)

    def test_disabled_and_invalid_configs(self):
        cfg=config()['pose_filter'];cfg['enabled']=False;f=OneEuroPose(cfg)
        f.update(np.eye(4),1.);p=np.eye(4);p[0,3]=1.;np.testing.assert_equal(f.update(p,1.01),p)
        for key,value in [('min_cutoff_hz',0),('beta',-1),('beta',float('nan')),('derivative_cutoff_hz',True)]:
            bad=copy.deepcopy(cfg);bad['position'][key]=value
            with self.assertRaisesRegex(ValueError,'INVALID_FILTER'):validate_config(bad)
        for value in (None,[],{'enabled':True,'position':3}):
            with self.assertRaisesRegex(ValueError,'INVALID_FILTER'):validate_config(value)


class FilterLifecycleTests(unittest.TestCase):
    def setUp(self):
        self.cfg=config();self.cfg['kinematics']['measured_context']=False
        self.session=TeleopSession(self.cfg,('joint',));self.seq=0

    def ingest(self,t,pose=None,grip=1.,**kw):
        self.seq+=1
        frame=VRFrame('device',self.seq,t,'left',tuple((np.eye(4) if pose is None else pose).ravel()),grip=grip,**kw)
        self.session.ingest(frame,t);return frame

    def engage(self,start=1.,pose=None):
        self.ingest(start,pose,grip=0.)
        for i in range(1,14):self.ingest(start+i*.1,pose)
        self.assertEqual(self.session.mode,'follow')

    def test_raw_frame_and_command_timestamp_preserved(self):
        self.engage();s=self.session
        req=s.request({'joint':0.},2.3,2.3);s.accept_result(IKResult(req,True,'OK',(0.,),tuple(np.eye(4).ravel())),2.3)
        p=np.eye(4);p[0,3]=.1;frame=self.ingest(2.32,p)
        np.testing.assert_equal(s.frame.pose,frame.pose)
        req=s.request({'joint':0.},2.32,2.32)
        self.assertEqual(req.stamp,frame.stamp);self.assertEqual(req.expires,frame.stamp+self.cfg['command_max_age'])
        self.assertGreater(abs(np.array(req.target).reshape(4,4)[1,3]),0)
        self.assertLess(abs(np.array(req.target).reshape(4,4)[1,3]),.1)
        saved=s.filtered_pose.copy();self.assertFalse(s.ingest(frame,2.33));np.testing.assert_equal(s.filtered_pose,saved)

    def test_release_reengage_and_clutch_counter_reset_history(self):
        self.engage();p=np.eye(4);p[0,3]=.3;self.ingest(2.32,p)
        self.ingest(2.34,p,grip=0.)
        np.testing.assert_allclose(self.session.filtered_pose,robot_pose(p))
        self.assertEqual(self.session.mode,'idle')
        p[0,3]=1.;self.engage(3.,p)
        np.testing.assert_allclose(self.session.filtered_pose,robot_pose(p))
        self.assertIsNone(self.session.anchor_robot)
        p[0,3]=2.;self.ingest(4.32,p,clutch_seq=1)
        self.assertEqual(self.session.mode,'idle');np.testing.assert_allclose(self.session.filtered_pose,robot_pose(p))

    def test_stale_tracking_connection_and_stop_fail_without_filtering_buttons(self):
        for extra,code in [({'valid':False},'TRACKING_INVALID'),({'stop':True},'VR_STOP')]:
            self.setUp();self.engage()
            with self.assertRaisesRegex(ValueError,code):self.ingest(2.32,**extra)
            self.assertIsNone(self.session.pose_filter.stamp)
        self.setUp();self.engage()
        with self.assertRaisesRegex(ValueError,'VR_INPUT_TIMEOUT'):self.session.check(2.6)
        self.assertIsNone(self.session.filtered_pose)
        self.setUp();self.engage()
        with self.assertRaisesRegex(ValueError,'VR_INPUT_TIMEOUT'):self.ingest(2.7)
        self.setUp();self.engage()
        with self.assertRaisesRegex(ValueError,'VR_CONNECTION_CHANGED'):
            self.session.ingest(VRFrame('other',90,2.32,'left',tuple(np.eye(4).ravel())),2.32)

    def test_reversed_source_time_is_rejected_and_legacy_still_works(self):
        self.engage()
        with self.assertRaisesRegex(ValueError,'VR_FILTER_TIME_ORDER'):self.ingest(2.29)
        cfg=config();cfg.pop('pose_filter');s=TeleopSession(cfg,('joint',))
        f=VRFrame('legacy',1,1.,'left',tuple(np.eye(4).ravel()))
        self.assertTrue(s.ingest(f,1.));np.testing.assert_equal(s.filtered_pose,robot_pose(f.pose))


class DualArmFilterTests(unittest.TestCase):
    def setUp(self):
        self.seq=0
        self.cfg=json.loads((ROOT/'src/integration/bindu_runtime/config/robots/snowbot/bimanual_snowbot_real.json').read_text())
        self.cfg['engage_seconds']=.05
        self.cfg['pico_grasp_resume_seconds']=.04

    def session(self,ordinary=False):
        return (IndependentArmsSession if ordinary else BimanualSession)(copy.deepcopy(self.cfg),1,0.)

    def feed(self,session,t,translations=None,rotation=None,grips=(1.,1.),clutch=(0,0)):
        self.seq+=1;frames=[]
        rotation=np.eye(3) if rotation is None else rotation
        translations=np.zeros((2,3)) if translations is None else translations
        for i,(side,x) in enumerate((('left',.17),('right',-.17))):
            pose=np.eye(4);pose[:3,:3]=rotation;pose[:3,3]=rotation @ [x,0.,0.]+translations[i]
            raw=OPENXR_TO_ROBOT.T @ pose @ OPENXR_TO_ROBOT
            frames.append(VRFrame('pico',self.seq,t,side,tuple(raw.ravel()),grip=grips[i],
                clutch_seq=clutch[i],mapping='pico_snowbot_v1',head_pose=tuple(np.eye(4).ravel()),
                head_stamp=t,head_source_id='pico',head_seq=self.seq))
        session.ingest(*frames,t)
        return frames

    def engage(self,session,capture=True):
        self.feed(session,1.,grips=(0.,0.));self.feed(session,1.01);self.feed(session,1.07)
        home=np.eye(4);home[:3,3]=[.6,-.1,.7]
        if isinstance(session,IndependentArmsSession):home=np.array([home,home])
        if capture:session.capture(home)
        return home

    def test_both_modes_reduce_stationary_position_and_rotation_jitter(self):
        for ordinary in (False,True):
            with self.subTest(ordinary=ordinary):
                session=self.session(ordinary);self.engage(session)
                rng=np.random.default_rng(13);raw=[];filtered=[];raw_angles=[];angles=[]
                for i in range(260):
                    delta=rng.normal(0,.003,3);rotation=rotation_exp(rng.normal(0,.008,3))
                    frames=self.feed(session,1.09+i/90,np.array([delta,delta]),rotation)
                    if ordinary:
                        source=session.raw_input[0].copy();result=session.current_input[0].copy()
                        # Remove the hand's nominal location; rotation swings it.
                        source[:3,3]-=source[:3,:3] @ [.17,0.,0.]
                        result[:3,3]-=result[:3,:3] @ [.17,0.,0.]
                    else:source,result=session.raw_input,session.current_input
                    if i>30:
                        raw.append(source[:3,3]);filtered.append(result[:3,3])
                        raw_angles.append(rotation_log(source[:3,:3]));angles.append(rotation_log(result[:3,:3]))
                    self.assertEqual(session.stamp,frames[0].stamp)
                    if ordinary:self.assertEqual(session.hands[0].frame,frames[0])
                self.assertLess(np.sqrt(np.mean(np.square(filtered))),.7*np.sqrt(np.mean(np.square(raw))))
                self.assertLess(np.sqrt(np.mean(np.square(angles))),.7*np.sqrt(np.mean(np.square(raw_angles))))

    def test_filtered_tool_rotation_keeps_the_grip_midpoint_fixed(self):
        session=self.session();home=self.engage(session)
        point=session.control_point_in_tool
        midpoint=home[:3,3]+home[:3,:3] @ point
        for i in range(1,61):
            self.feed(session,1.07+i*.02,rotation=rotation_exp(np.array([.3,.5,.2])*i/60))
            target=session.target
            np.testing.assert_allclose(target[:3,3]+target[:3,:3] @ point,midpoint,atol=1e-10)
            np.testing.assert_allclose(target[:3,:3].T @ target[:3,:3],np.eye(3),atol=1e-10)

    def test_enabled_filter_preserves_all_six_spatial_directions(self):
        for ordinary in (False,True):
            for turning in (False,True):
                for axis in np.eye(3):
                    for sign in (-1,1):
                        with self.subTest(ordinary=ordinary,turning=turning,axis=axis,sign=sign):
                            session=self.session(ordinary);home=self.engage(session)
                            direction=sign*axis
                            if turning:self.feed(session,1.09,rotation=rotation_exp(.2*direction))
                            else:self.feed(session,1.09,np.array([.04*direction,.04*direction]))
                            target=session.target[0] if ordinary else session.target
                            origin=home[0] if ordinary else home
                            if turning:delta=rotation_log(target[:3,:3] @ origin[:3,:3].T)
                            else:
                                point=np.zeros(3) if ordinary else session.control_point_in_tool
                                delta=target[:3,3]+target[:3,:3] @ point-origin[:3,3]-origin[:3,:3] @ point
                            amount=float(delta @ direction)
                            self.assertGreater(amount,0.)
                            self.assertLess(amount,.2 if turning else .04)
                            np.testing.assert_allclose(delta,amount*direction,atol=1e-10)

    def test_async_capture_discards_pending_hand_motion_and_filter_tail(self):
        for ordinary in (False,True):
            with self.subTest(ordinary=ordinary):
                session=self.session(ordinary);home=self.engage(session,capture=False)
                delta=np.array([[.1,.02,.03],[.1,.02,.03]])
                self.feed(session,1.09,delta)
                session.capture(home)
                np.testing.assert_allclose(session.target,home,atol=1e-12)
                self.feed(session,1.11,delta)
                np.testing.assert_allclose(session.target,home,atol=1e-12)
        session=self.session();home=self.engage(session)
        self.feed(session,1.09,delta);session.reanchor()
        self.feed(session,1.10,delta);session.capture(home)
        self.feed(session,1.12,delta)
        np.testing.assert_allclose(session.target,home,atol=1e-12)

    def test_recapturing_one_hand_preserves_the_other_filter_and_anchor(self):
        session=self.session(True);home=self.engage(session)
        anchor=session.anchors[1].copy()
        delta=np.array([[.02,0.,0.],[.02,0.,0.]])
        self.feed(session,1.09,delta)
        self.feed(session,1.11,delta,grips=(0.,1.))
        right=session.hands[1].pose_filter
        velocity=right.velocity.copy();stamp=right.stamp
        session.capture(home)
        np.testing.assert_array_equal(right.velocity,velocity);self.assertEqual(right.stamp,stamp)
        np.testing.assert_array_equal(session.anchors[1],anchor)
        delta[0,0]=.2
        self.feed(session,1.13,delta)
        self.feed(session,1.20,delta)
        velocity=right.velocity.copy();stamp=right.stamp
        session.capture(home)
        np.testing.assert_allclose(session.target[0],home[0],atol=1e-12)
        np.testing.assert_array_equal(right.velocity,velocity);self.assertEqual(right.stamp,stamp)
        np.testing.assert_array_equal(session.anchors[1],anchor)

    def test_explicit_pause_accepts_fresh_input_and_discards_paused_motion(self):
        for ordinary in (False,True):
            with self.subTest(ordinary=ordinary):
                session=self.session(ordinary);home=self.engage(session)
                self.feed(session,1.09,np.full((2,3),.02))
                session.reanchor()
                self.feed(session,2.,np.full((2,3),.2))
                session.capture(home)
                np.testing.assert_allclose(session.target,home,atol=1e-12)
                self.feed(session,2.02,np.full((2,3),.2))
                np.testing.assert_allclose(session.target,home,atol=1e-12)

    def test_same_source_timestamp_keeps_short_release_and_grasp_events(self):
        session=self.session(True);self.engage(session)
        left,right=self.feed(session,1.09)
        session.ingest(replace(left,seq=left.seq+1,clutch_seq=1),
                       replace(right,seq=right.seq+1,stamp=1.10),1.10)
        self.assertEqual(session.active_mask,2)
        self.assertIsNone(session.target)
        session=self.session();home=self.engage(session)
        left,right=self.feed(session,1.09)
        session.ingest(replace(left,seq=left.seq+1),
                       replace(right,seq=right.seq+1,virtual_grip=True),1.09)
        self.assertEqual(session.grasp_mode,'right_axial');self.assertIsNone(session.target)
        session.capture(home)
        np.testing.assert_allclose(session.target,home,atol=1e-12)

    def test_raw_geometry_stop_stale_and_reversed_time_are_not_hidden(self):
        for ordinary in (False,True):
            for fault in ('stop','stale','reversed'):
                with self.subTest(ordinary=ordinary,fault=fault):
                    session=self.session(ordinary);self.engage(session)
                    left,right=self.feed(session,1.09)
                    if fault=='stop':right=replace(right,stop=True)
                    if fault=='reversed':left=replace(left,seq=left.seq+1,stamp=1.08);right=replace(right,seq=right.seq+1,stamp=1.08)
                    with self.assertRaises(ValueError):
                        session.ingest(left,right,1.40 if fault=='stale' else 1.09)
                    filters=[h.pose_filter for h in session.hands] if ordinary else [session.pose_filter]
                    self.assertTrue(all(f.stamp is None for f in filters))
        session=self.session();self.engage(session)
        with self.assertRaisesRegex(ValueError,'VR_GRASP_GEOMETRY'):
            self.feed(session,1.09,np.array([[0.,0.,0.],[3.,0.,0.]]))


if __name__=='__main__':unittest.main()
