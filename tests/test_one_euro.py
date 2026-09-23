import copy
import json
import unittest
from pathlib import Path
import numpy as np
from bindu_contracts.teleoperation import VRFrame, IKResult
from bindu_teleoperation.session import TeleopSession
from bindu_teleoperation.vr.mapping import robot_pose
from bindu_teleoperation.vr.one_euro import OneEuroPose, rotation_exp, rotation_log, validate_config

ROOT=Path(__file__).resolve().parents[1]


def config():
    return json.loads((ROOT/'src/integration/bindu_runtime/config/teleop_g1_left.json').read_text())


class OneEuroTests(unittest.TestCase):
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

    def test_reversed_or_same_source_time_is_rejected_and_legacy_still_works(self):
        self.engage()
        with self.assertRaisesRegex(ValueError,'VR_FILTER_TIME_ORDER'):self.ingest(2.29)
        cfg=config();cfg.pop('pose_filter');s=TeleopSession(cfg,('joint',))
        f=VRFrame('legacy',1,1.,'left',tuple(np.eye(4).ravel()))
        self.assertTrue(s.ingest(f,1.));np.testing.assert_equal(s.filtered_pose,robot_pose(f.pose))


if __name__=='__main__':unittest.main()
