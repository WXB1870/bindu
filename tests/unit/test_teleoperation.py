from tests.support.paths import ROOT
import json
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path
import numpy as np
from bindu_contracts.teleoperation import VRFrame, IKRequest, IKResult
from bindu_contracts.profile import Profile
from bindu_teleoperation.config import load_config
from bindu_teleoperation.vr.receiver import decode_controller
from bindu_teleoperation.vr.mapping import pose_matrix, robot_pose, relative_target
from bindu_teleoperation.session import TeleopSession
from bindu_teleoperation.feedback import TeleopFeedback, display_snapshot
from bindu_teleoperation.vr.display import viewer_matrix

POSE = tuple(np.eye(4).ravel())


def configuration():
    profile = Profile.load(ROOT/'src/integration/bindu_runtime/config/robots/huawei_v34/huawei_v34_left_sim.json')
    return load_config(ROOT/'src/integration/bindu_runtime/config/robots/huawei_v34/teleop_v34.json', profile,
                       ROOT/'src/hardware/bindu_description/urdf')


class DisplayFeedbackTests(unittest.TestCase):
    def setUp(self):
        self.view = TeleopFeedback(configuration())
        self.view.event('TELEOP_STARTED', '{}', 'task', 10.)
        self.view.event('TELEOP_MODE', 'follow', 'task', 10.1)

    def target(self, stamp=10.15):
        pose = np.eye(4); pose[0, 3] = .01
        self.view.event('TELEOP_IK_RESULT', json.dumps({'code': 'OK', 'input_stamp': stamp,
            'target': pose.ravel().tolist()}), 'task', 10.16)

    def snapshot(self, now=10.2, **kwargs):
        values = dict(measured=list(POSE), measured_stamp=10.15, input_stamp=10.15, input_valid=True)
        values.update(kwargs)
        return self.view.snapshot(now, **values)

    def test_error_uses_feedback_and_target(self):
        self.target()
        view = self.snapshot()
        self.assertEqual(view['mode'], 'FOLLOW')
        self.assertAlmostEqual(view['error']['position_m'], .01)
        self.assertAlmostEqual(view['error']['rotation_rad'], 0.)

    def test_pause_and_late_result_cannot_resurrect_target(self):
        self.target()
        self.view.event('TELEOP_MODE', 'idle', 'task', 10.17)
        self.view.event('TELEOP_IK_RESULT', json.dumps({'code': 'OK', 'input_stamp': 10.15,
            'target': list(POSE)}), 'task', 10.18)
        self.assertEqual(self.snapshot()['mode'], 'PAUSED')
        self.assertIsNone(self.snapshot()['target'])
        self.view.event('TELEOP_MODE', 'follow', 'task', 10.19)
        self.view.event('TELEOP_IK_RESULT', json.dumps({'code': 'OK', 'input_stamp': 10.15,
            'target': list(POSE)}), 'task', 10.20)
        self.assertIsNone(self.snapshot()['target'])

    def test_stale_feedback_input_and_observer_clear_live_error(self):
        self.target()
        for changes, reason in (({'measured_stamp': 9.}, 'TELEOP_FEEDBACK_STALE'),
                                ({'input_stamp': 9.}, 'VR_INPUT_TIMEOUT'),
                                ({'input_valid': False}, 'VR_TRACKING_INVALID')):
            view = self.snapshot(**changes)
            self.assertIsNone(view['target'])
            self.assertIsNone(view['error'])
            self.assertEqual(view['reason'], reason)
        view = display_snapshot(self.snapshot(), .7)
        self.assertEqual(view['mode'], 'DISPLAY_STALE')
        self.assertIsNone(view['measured'])
        target_stale = self.snapshot(now=10.8, measured_stamp=10.8, input_stamp=10.8)
        self.assertIsNone(target_stale['target'])
        self.assertEqual(target_stale['reason'], 'Waiting for fresh target')

    def test_end_reason_sticky_until_new_session(self):
        self.target()
        self.view.event('TELEOP_ENDED', '{"code":"IK_RESIDUAL"}', 'task', 10.2)
        self.assertEqual(self.snapshot(now=11.)['reason'], 'IK_RESIDUAL')
        self.assertIsNone(self.snapshot()['target'])
        self.view.event('TELEOP_STARTED', '{}', 'new', 11.)
        self.assertEqual(self.snapshot()['mode'], 'PAUSED')
        self.view.event('TELEOP_ENDED', '{"code":"OLD"}', 'task', 11.1)
        self.assertEqual(self.snapshot()['mode'], 'PAUSED')

    def test_viewer_transform_preserves_local_axis_and_position(self):
        pose = np.eye(4); pose[:3, 3] = [1, 2, 3]
        shown = np.array(viewer_matrix(pose)).reshape(4, 4, order='F')
        np.testing.assert_allclose(shown[:3, 3], [-2, 3, -2])
        np.testing.assert_allclose(shown[:3, 0], [0, 0, -1])

    def test_full_display_queue_never_waits(self):
        from queue import Queue
        from bindu_teleoperation.vr.receiver import _latest_put
        output = Queue(maxsize=1)
        for i in range(1000):
            _latest_put(output, i)
        self.assertEqual(output.qsize(), 1)
        self.assertEqual(output.get_nowait(), 999)


class VRConnectivityTests(unittest.TestCase):
    def setUp(self):
        from bindu_teleoperation.vr.connectivity import VRConnectivity
        self.monitor = VRConnectivity()
        self.pair = tuple(VRFrame('headset', 1, 10., side, POSE, .8, .4,
                                  mapping='pico_snowbot_v1', buttons=1)
                          for side in ('left', 'right'))

    def test_waiting_live_and_stale_do_not_reuse_pose_or_buttons(self):
        self.assertEqual(self.monitor.snapshot(10.)['mode'], 'INPUT_WAITING')
        self.monitor.ingest(self.pair)
        live = self.monitor.snapshot(10.1)
        self.assertEqual(live['mode'], 'INPUT_LIVE')
        self.assertEqual(live['sides']['left']['position_m'], [0., 0., 0.])
        for now in (11., 9.):
            stale = self.monitor.snapshot(now)
            self.assertEqual(stale['mode'], 'INPUT_STALE')
            for side in stale['sides'].values():
                self.assertFalse(side['valid'])
                self.assertIsNone(side['position_m'])
                self.assertIsNone(side['buttons'])

    def test_invalid_payload_is_distinct_from_no_events_and_partial(self):
        self.monitor.ingest((self.pair[0], replace(self.pair[1], valid=False, pose=())))
        self.assertEqual(self.monitor.snapshot(10.1)['mode'], 'INPUT_PARTIAL')
        self.monitor.ingest(tuple(replace(f, seq=2, stamp=10.1, valid=False, pose=()) for f in self.pair))
        self.assertEqual(self.monitor.snapshot(10.2)['mode'], 'INPUT_INVALID')

    def test_reconnect_resets_rate_and_old_side_and_duplicates_do_not_refresh(self):
        self.monitor.ingest(self.pair)
        self.monitor.ingest(tuple(replace(f, seq=6, stamp=10.1) for f in self.pair))
        self.assertAlmostEqual(self.monitor.snapshot(10.2)['receive_hz'], 50.)
        self.monitor.ingest(tuple(replace(f, seq=5, stamp=10.3) for f in self.pair))
        self.assertEqual(self.monitor.snapshot(10.4)['sides']['left']['seq'], 6)
        self.monitor.ingest(replace(self.pair[0], source_id='new', stamp=11.))
        fresh = self.monitor.snapshot(11.1)
        self.assertEqual(fresh['mode'], 'INPUT_PARTIAL')
        self.assertIsNone(fresh['receive_hz'])
        self.assertIsNone(fresh['sides']['right']['seq'])

    def test_connectivity_panel_names_physical_keys_and_has_no_robot_target(self):
        from bindu_teleoperation.vr.display import status_lines
        self.monitor.ingest(self.pair)
        view = self.monitor.snapshot(10.1)
        lines = ' '.join(line for line, _ in status_lines(view))
        for label in ('NO ROBOT', 'INPUT_LIVE', 'X:1', 'Y:0', 'A:1', 'B:0'):
            self.assertIn(label, lines)
        self.assertIsNone(view['target'])


class VRContracts(unittest.TestCase):
    def setUp(self):
        self.cfg = configuration()
        # These contract/mapping assertions retain the unfiltered legacy mode.
        # Filter-enabled lifecycle and geometry checks live in test_one_euro.py.
        self.cfg['pose_filter']['enabled'] = False
        self.names = tuple(self.cfg['kinematics']['joint_names'])
        self.session = TeleopSession(self.cfg, self.names)
        self.frame = VRFrame('connection', 0, 10., 'left', POSE)

    def follow(self):
        self.session.ingest(self.frame, 10.)
        for i in range(1,13):
            self.session.ingest(replace(self.frame, seq=i, stamp=10.+i*.1, grip=1.), 10.+i*.1)

    def test_column_major_wire_and_basis(self):
        raw = np.eye(4); raw[:3,3] = [.1,.2,.3]
        value = {'left':raw.ravel(order='F').tolist(), 'leftState':{'squeezeValue':.9,'triggerValue':.4}}
        frame = decode_controller(value, 'left', 'source', 1, 10.)
        self.assertTrue(frame.valid)
        np.testing.assert_allclose(robot_pose(frame.pose)[:3,3], [-.3,-.1,.2])
        self.assertEqual((frame.grip,frame.trigger),(.9,.4))

    def test_malformed_event_invalidates_instead_of_reusing_pose(self):
        for value in ({}, {'left':[0.]*16,'leftState':{}},
                      {'left':list(POSE),'leftState':{'squeezeValue':float('nan'),'triggerValue':0.}}):
            self.assertFalse(decode_controller(value,'left','source',1,10.).valid)
        bad=np.eye(4);bad[0,0]=-1
        with self.assertRaisesRegex(ValueError,'ROTATION'):pose_matrix(bad)

    def test_relative_mapping_matches_v34_composition(self):
        anchor=np.eye(4);anchor[:3,3]=[1,2,3]
        current=anchor.copy();current[0,3]+=.1
        robot=np.eye(4);robot[:3,3]=[.3,.2,.5]
        target=np.array(relative_target(anchor,current,robot,.5)).reshape(4,4)
        np.testing.assert_allclose(target[:3,3],[.35,.2,.5])
        np.testing.assert_allclose(relative_target(anchor,anchor,robot),robot.ravel())

    def test_base_mapping_translation_is_independent_of_starting_wrist(self):
        robot=np.eye(4);robot[:3,3]=[.3,.2,.5]
        robot[:3,:3]=[[0,0,1],[0,1,0],[-1,0,0]]
        for angle in (0.,np.pi/2,-np.pi/3):
            c,s=np.cos(angle),np.sin(angle)
            anchor=np.eye(4);anchor[:3,3]=[1,2,3]
            anchor[:3,:3]=[[c,-s,0],[s,c,0],[0,0,1]]
            current=anchor.copy();current[0,3]+=.1
            for yaw,expected in ((0.,[.35,.2,.5]),(np.pi/2,[.3,.25,.5])):
                target=np.array(relative_target(anchor,current,robot,.5,
                    mapping_mode='robot_base',operator_yaw_rad=yaw)).reshape(4,4)
                np.testing.assert_allclose(target[:3,3],expected,atol=1e-12)
                np.testing.assert_allclose(target[:3,:3],robot[:3,:3],atol=1e-12)
                np.testing.assert_allclose(relative_target(anchor,anchor,robot,
                    mapping_mode='robot_base',operator_yaw_rad=yaw),robot.ravel(),atol=1e-12)

    def test_base_mapping_rotation_and_calibration_use_the_same_axes(self):
        anchor=np.eye(4);anchor[:3,:3]=[[0,-1,0],[1,0,0],[0,0,1]]
        robot=np.eye(4);robot[:3,:3]=[[0,1,0],[-1,0,0],[0,0,1]]
        c,s=np.cos(.3),np.sin(.3)
        rotate_x=np.array([[1,0,0],[0,c,-s],[0,s,c]])
        rotate_y=np.array([[c,0,s],[0,1,0],[-s,0,c]])
        current=anchor.copy();current[:3,:3]=rotate_x @ anchor[:3,:3]
        for yaw,expected in ((0.,rotate_x),(np.pi/2,rotate_y)):
            target=np.array(relative_target(anchor,current,robot,
                mapping_mode='robot_base',operator_yaw_rad=yaw)).reshape(4,4)
            np.testing.assert_allclose(target[:3,:3],expected @ robot[:3,:3],atol=1e-12)
        legacy=np.array(relative_target(anchor,current,robot)).reshape(4,4)
        np.testing.assert_allclose(legacy[:3,:3],rotate_y.T @ robot[:3,:3],atol=1e-12)

    def test_mapping_config_preserves_old_defaults_and_rejects_ignored_yaw(self):
        profile=Profile.load(ROOT/'src/integration/bindu_runtime/config/robots/huawei_v34/huawei_v34_left_sim.json')
        cfg=json.loads((ROOT/'src/integration/bindu_runtime/config/robots/huawei_v34/teleop_v34.json').read_text())
        cfg.pop('mapping_mode');cfg.pop('operator_yaw_rad')
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'teleop.json'
            path.write_text(json.dumps(cfg))
            loaded=load_config(path,profile,ROOT/'src/hardware/bindu_description/urdf')
            self.assertEqual((loaded['mapping_mode'],loaded['operator_yaw_rad']),('v34_anchor',0.))
            for mode,yaw in (('unknown',0.),('robot_base',True),('robot_base','90'),
                             ('robot_base',float('nan')),('robot_base',float('inf')),('v34_anchor',.1)):
                with self.subTest(mode=mode,yaw=yaw):
                    path.write_text(json.dumps(dict(cfg,mapping_mode=mode,operator_yaw_rad=yaw)))
                    with self.assertRaises(ValueError):
                        load_config(path,profile,ROOT/'src/hardware/bindu_description/urdf')

    def test_base_mapping_reclutch_preserves_direction_and_anchors_at_feedback(self):
        self.cfg['mapping_mode']='robot_base'
        self.cfg['operator_yaw_rad']=0.
        seq=0
        def ingest(now,pose,grip):
            nonlocal seq
            seq+=1
            self.session.ingest(replace(self.frame,seq=seq,stamp=now,
                                        pose=tuple(pose.ravel()),grip=grip),now)
        previous=None
        for i in range(2):
            start=10.+i*3.
            pose=np.eye(4);pose[:3,3]=[i,0.,.2]
            if i:pose[:3,:3]=[[0,-1,0],[1,0,0],[0,0,1]]
            feedback=np.eye(4);feedback[:3,3]=[.3+i*.1,.2,.5]
            feedback[:3,:3]=[[0,0,1],[0,1,0],[-1,0,0]]
            positions=dict.fromkeys(self.names,.1*i)
            ingest(start,pose,0.)
            if previous:
                self.assertFalse(self.session.accept_result(previous,start))
            ingest(start+.01,pose,1.)
            ingest(start+1.02,pose,1.)
            req=self.session.request(positions,start+1.02,start+1.02)
            self.assertFalse(req.target)
            self.assertEqual(req.seed,tuple(positions.values()))
            self.session.accept_result(IKResult(req,True,'OK',req.seed,tuple(feedback.ravel())),start+1.03)
            ingest(start+1.04,pose,1.)
            req=self.session.request(positions,start+1.04,start+1.04)
            np.testing.assert_allclose(req.target,feedback.ravel(),atol=1e-12)
            moved=pose.copy();moved[0,3]+=.01
            ingest(start+1.05,moved,1.)
            req=self.session.request(positions,start+1.05,start+1.05)
            target=np.array(req.target).reshape(4,4)
            # Raw OpenXR +X maps to robot -Y regardless of starting wrist pose.
            np.testing.assert_allclose(target[:3,3]-feedback[:3,3],[0,-.01,0],atol=1e-12)
            previous=IKResult(req,True,'OK',req.seed,tuple(target.ravel()))

    def test_requires_release_then_timed_engagement(self):
        for i in range(20):
            self.session.ingest(replace(self.frame,seq=i,stamp=10.+i*.1,grip=1.),10.+i*.1)
        self.assertEqual(self.session.mode,'idle')
        self.session=TeleopSession(self.cfg,self.names)
        self.follow()
        self.assertEqual(self.session.mode,'follow')

    def test_anchor_from_feedback_and_generation_fences_late_ik(self):
        self.follow()
        req=self.session.request(dict.fromkeys(self.names,.1),11.2,11.2)
        self.assertFalse(req.target)
        self.assertEqual(req.seed,(.1,)*7)
        result=IKResult(req,True,'OK',req.seed,POSE)
        self.session.ingest(replace(self.frame,seq=13,stamp=11.21,grip=0.),11.21)
        self.assertEqual(self.session.mode,'idle')
        self.assertFalse(self.session.accept_result(result,11.22))
        self.assertIsNone(self.session.anchor_robot)

    def test_follow_result_and_input_are_not_restamped(self):
        self.follow()
        req=self.session.request(dict.fromkeys(self.names,0.),11.2,11.2)
        self.assertAlmostEqual(req.stamp,11.2)
        self.assertTrue(self.session.accept_result(IKResult(req,True,'OK',(0.,)*7,POSE),11.21))
        with self.assertRaisesRegex(ValueError,'INPUT_TIMEOUT'):self.session.check(11.6)

    def test_stale_reconnect_and_bad_tracking_fail(self):
        self.session.ingest(self.frame,10.)
        for frame,now,code in [(replace(self.frame,seq=1,stamp=9.),10.,'STALE'),
                               (replace(self.frame,seq=1,source_id='new'),10.,'CONNECTION'),
                               (replace(self.frame,seq=1,valid=False),10.,'TRACKING'),
                               (replace(self.frame,seq=1,stop=True),10.,'VR_STOP'),
                               (replace(self.frame,seq=1,init=True),10.,'INIT_NOT_CONFIGURED')]:
            self.session=TeleopSession(self.cfg,self.names)
            self.session.ingest(self.frame,10.)
            with self.subTest(code=code),self.assertRaisesRegex(ValueError,code):
                self.session.ingest(frame,now)

    def test_duplicate_input_does_not_refresh_age(self):
        self.session.ingest(self.frame,10.)
        self.assertFalse(self.session.ingest(replace(self.frame,stamp=10.2),10.2))
        with self.assertRaisesRegex(ValueError,'TIMEOUT'):self.session.check(10.3)

    def test_invalid_sequence_and_release_counter_are_rejected(self):
        for change in ({'seq': True}, {'seq': -1}, {'seq': .5},
                       {'clutch_seq': True}, {'clutch_seq': -1}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                TeleopSession(self.cfg, self.names).ingest(replace(self.frame, **change), 10.)

    def test_unfiltered_input_clock_cannot_go_backwards(self):
        first = replace(self.frame, stamp=10.1)
        self.session.ingest(first, 10.1)
        with self.assertRaisesRegex(ValueError, 'VR_INPUT_UNORDERED'):
            self.session.ingest(replace(first, seq=1, stamp=10.05), 10.15)
        self.assertEqual(self.session.frame, first)
        self.assertTrue(self.session.ingest(replace(first, seq=1, stamp=10.15), 10.15))

    def test_coalesced_release_still_invalidates_anchor(self):
        self.follow()
        req=self.session.request(dict.fromkeys(self.names,0.),11.2,11.2)
        # Release and squeeze happened between two subscriber polls.
        self.session.ingest(replace(self.frame,seq=14,stamp=11.22,grip=1.,clutch_seq=1),11.22)
        self.assertEqual(self.session.mode,'idle')
        self.assertFalse(self.session.accept_result(IKResult(req,True,'OK',(0.,)*7,POSE),11.23))

    def test_failed_expired_or_other_session_ik_cannot_command(self):
        self.follow()
        req=self.session.request(dict.fromkeys(self.names,0.),11.2,11.2)
        with self.assertRaisesRegex(ValueError,'IK_SOLVER_FAILED'):
            self.session.accept_result(IKResult(req,False,'IK_SOLVER_FAILED'),11.21)
        with self.assertRaisesRegex(ValueError,'EXPIRED'):
            self.session.accept_result(IKResult(req,True,'OK',(0.,)*7,POSE),11.7)
        other=replace(req,request_id='other_1')
        self.assertFalse(self.session.accept_result(IKResult(other,True,'OK',(0.,)*7,POSE),11.21))


class RealKinematics(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            import pinocchio
            import casadi
        except ImportError:
            raise unittest.SkipTest('Pinocchio/CasADi not installed; run on the IK validation environment')
        from bindu_kinematics.pinocchio_casadi import PinocchioCasadiIK
        cls.solver=PinocchioCasadiIK(configuration()['kinematics'])

    def test_fk_and_continuous_ik_reachable_path(self):
        arm=self.solver.arm
        q=np.zeros(7)
        worst=0.
        for i in range(1,11):
            target_q=np.zeros(7);target_q[0]=i*.004;target_q[3]=-i*.003
            target=arm.fk(target_q)
            req=IKRequest(str(i),1,10.,10.4,arm.names,tuple(q),tuple(target.ravel()))
            result=self.solver.solve(req)
            self.assertTrue(result.success,result)
            self.assertLess(result.position_error,.015)
            q=np.array(result.positions);worst=max(worst,result.elapsed)
        self.assertGreater(np.linalg.norm(q),.001)

    def test_unreachable_and_invalid_seed_have_no_command(self):
        arm=self.solver.arm
        target=arm.fk(np.zeros(7));target[0,3]+=10.
        req=IKRequest('bad',1,10.,10.4,arm.names,(0.,)*7,tuple(target.ravel()))
        result=self.solver.solve(req)
        self.assertFalse(result.success)
        self.assertFalse(result.positions)
        result=self.solver.solve(replace(req,seed=(float('nan'),)*7))
        self.assertEqual(result.code,'IK_INVALID_SEED')

    def test_worker_is_bounded_and_exit_is_detected(self):
        import time
        from bindu_kinematics.worker import KinematicsWorker
        worker=KinematicsWorker(configuration()['kinematics'])
        self.addCleanup(worker.close)
        deadline=time.monotonic()+30
        while not worker.ready and time.monotonic()<deadline:
            worker.poll();time.sleep(.01)
        self.assertTrue(worker.ready)
        req=IKRequest('fk',1,10.,10.4,self.solver.arm.names,(0.,)*7)
        self.assertTrue(worker.submit(req))
        self.assertFalse(worker.submit(req))
        result=None
        deadline=time.monotonic()+2
        while result is None and time.monotonic()<deadline:
            result=worker.poll();time.sleep(.005)
        self.assertTrue(result.success)
        worker.process.terminate();worker.process.join(timeout=2)
        with self.assertRaisesRegex(RuntimeError,'EXITED'):worker.poll()


if __name__=='__main__':unittest.main()
