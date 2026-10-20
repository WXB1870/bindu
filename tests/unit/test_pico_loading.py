"""Physical button identity, coalesced events and guarded operator transitions."""
from dataclasses import replace
import unittest
import json
from pathlib import Path
import numpy as np
from bindu_contracts.teleoperation import VRFrame
from bindu_teleoperation.vr.receiver import decode_controller, ButtonCounters
from bindu_teleoperation.pico import PicoControl, PicoFeedback, PICO_MAPPING
from bindu_teleoperation.bimanual_session import BimanualSession


from tests.support.pico import Rig


class GuidedLoadingTests(unittest.TestCase):
    def start_loading(self, rig):
        rig.click(0,2);rig.complete();rig.click(1,1)
        self.assertEqual(rig.core.phase,'loading')
        self.assertTrue(rig.core.supported)

    def open_close(self, rig):
        rig.triggers=[1.,1.];rig.step()
        rig.modes=('open','open');rig.complete();rig.step()
        rig.triggers=[0.,0.];rig.step()
        rig.modes=('closed','closed');rig.complete();rig.step()

    def test_complete_mount_and_unmount_with_only_controller_inputs(self):
        r=Rig('empty',guided=True)
        r.buttons[1]=1
        for _ in range(60):r.step()
        self.assertEqual(r.core.pending.operation,'prepare')
        r.complete();r.buttons[1]=0;r.step()
        self.assertEqual(r.core.phase,'standby')
        self.assertIsNone(r.core.pending)
        r.click(0,2);r.complete();r.step()
        self.assertEqual(r.core.phase,'load_check')
        self.assertEqual(r.core.snapshot(r.now)['operator_guide']['step'],'support')
        r.click(1,1)
        self.assertEqual(r.core.phase,'loading')
        r.click(1,1)
        self.assertEqual(r.core.reason,'OPEN_BOTH_GRIPPERS_BEFORE_CONFIRM')
        self.assertEqual(r.core.load,'empty')
        self.open_close(r)
        self.assertEqual(r.core.snapshot(r.now)['operator_guide']['step'],'confirm')
        r.click(1,1)
        self.assertEqual(r.core.load,'held')
        self.assertEqual(r.core.load_source,'operator_load_confirmation')
        self.assertEqual(r.core.phase,'standby')
        self.assertIsNone(r.core.pending)
        self.start_loading(r);self.assertEqual(r.core.loading_goal,'empty')
        self.open_close(r);r.click(1,1)
        self.assertEqual(r.core.load,'empty');self.assertEqual(r.core.phase,'standby')

    def test_prepare_accepts_held_or_unknown_without_empty_declaration(self):
        for load in ('held','unknown'):
            r=Rig(load,guided=True);r.buttons[1]=1
            for _ in range(60):r.step()
            self.assertEqual(r.core.pending.operation,'prepare')
            self.assertNotIn('load_empty',r.core.last_prepare_check['checks'])
            self.assertEqual(r.core.load,load)
            r.complete()
            self.assertEqual((r.core.phase,r.core.reason),('standby','PREPARED'))
            self.assertEqual(r.core.load,load)

    def test_prepare_release_fences_late_arrival_and_does_not_enter_loading(self):
        r=Rig(guided=True);r.buttons[1]=1
        for _ in range(60):r.step()
        token=r.core.pending.token
        r.buttons[1]=0;r.step()
        self.assertEqual(r.core.pending.operation,'stop')
        self.assertFalse(r.core.complete(token,True,now=r.now))
        r.complete();r.step()
        self.assertEqual(r.core.phase,'standby');self.assertIsNone(r.core.pending)

    def test_loading_completion_requires_released_squeezes(self):
        r=Rig(guided=True);self.start_loading(r);self.open_close(r)
        r.grips=[1.,1.];r.step();r.click(1,1)
        self.assertEqual(r.core.phase,'loading')
        self.assertEqual(r.core.reason,'RELEASE_SQUEEZES_BEFORE_CONFIRM')
        r.grips=[0.,0.];r.step();r.click(1,1)
        self.assertEqual(r.core.phase,'standby');self.assertIsNone(r.core.pending)

    def test_interrupted_loading_recovery_returns_standby_until_new_request(self):
        r=Rig('unknown',guided=True);self.start_loading(r)
        r.now+=.3;r.core.update_feedback(PicoFeedback(r.now,True),r.now);r.core.tick(r.now)
        self.assertEqual(r.core.phase,'locked');r.complete();r.step()
        r.click(1,1)
        self.assertEqual(r.core.phase,'standby');self.assertFalse(r.core.supported)
        self.assertIsNone(r.core.pending)
        r.click(0,2);r.complete();r.click(1,1)
        self.assertEqual(r.core.phase,'loading');self.assertEqual(r.core.load,'unknown')
        self.assertTrue(r.core.supported)

    def test_direct_engagement_does_not_require_a_b_or_load_declaration(self):
        for load in ('empty','held','unknown'):
            r=Rig(load,guided=True)
            self.assertEqual(r.core.phase,'standby');self.assertIsNone(r.core.pending)
            r.grips=[1.,1.];r.step()
            self.assertEqual(r.core.phase,'engaging')
            self.assertEqual(r.core.pending.operation,'follow')
            r.complete();self.assertEqual(r.core.phase,'following')
            r.grips=[0.,1.];r.step();r.complete()
            r.step();self.assertEqual(r.core.phase,'tool_standby')
            r.grips=[1.,1.];r.step();self.assertIsNone(r.core.pending)
            r.grips=[0.,0.];r.step();r.grips=[1.,1.];r.step()
            self.assertEqual(r.core.pending.operation,'follow')

    def test_read_only_rejection_requires_fresh_release_before_retry(self):
        r=Rig('unknown',guided=True);r.grips=[1.,1.];r.step()
        token=r.core.pending.token
        r.core.complete(token,False,'RuntimeError:BIMANUAL_GRASP_POSE',r.now,rejected=True)
        self.assertEqual(r.core.phase,'standby');self.assertIsNone(r.core.pending)
        r.step();self.assertIsNone(r.core.pending)
        self.assertEqual(r.core.snapshot(r.now)['operator_guide']['step'],'retry')
        r.grips=[0.,0.];r.step();r.grips=[1.,1.];r.step()
        self.assertEqual(r.core.pending.operation,'follow')

    def test_active_or_unconfirmed_failure_still_locks(self):
        for stopped,rejected in ((True,False),(False,True)):
            r=Rig(guided=True);r.grips=[1.,1.];r.step()
            r.core.update_feedback(PicoFeedback(r.now,stopped),r.now)
            r.core.complete(r.core.pending.token,False,'TEST_FAILURE',r.now,rejected=rejected)
            self.assertEqual(r.core.phase,'locked')

    def test_fault_recovery_never_consumes_held_squeeze_or_restarts(self):
        r=Rig(guided=True);r.grips=[1.,1.];r.step();r.complete()
        r.click(0,1);r.complete();r.click(1,1)
        self.assertEqual(r.core.phase,'locked')
        r.grips=[0.,0.];r.step();r.click(1,1)
        self.assertEqual(r.core.phase,'standby');self.assertIsNone(r.core.pending)
        r.step();self.assertIsNone(r.core.pending)

    def test_unknown_load_can_choose_mount_or_unmount(self):
        r=Rig('unknown',guided=True);r.core.loading_check(r.now);r.step()
        self.assertEqual(r.core.loading_goal,'held')
        r.click(0,0)
        self.assertEqual(r.core.loading_goal,'empty');self.assertEqual(r.core.load,'unknown')

    def test_idle_gap_keeps_step_and_held_key_cannot_acknowledge_support(self):
        r=Rig(guided=True);r.core.loading_check(r.now)
        r.now+=.4;r.core.update_feedback(PicoFeedback(r.now,True),r.now);r.core.tick(r.now)
        self.assertEqual(r.core.phase,'load_check');self.assertEqual(r.core.drain(),[])
        self.assertTrue(r.core.idle_input_lost)
        r.buttons[1]=2;r.step();r.step()
        self.assertEqual(r.core.phase,'load_check')
        r.buttons[1]=0;r.step();r.click(1,1)
        self.assertEqual(r.core.phase,'loading')

    def test_active_loss_still_locks_and_interrupted_loading_becomes_unknown(self):
        for phase in ('preparing','following','loading'):
            r=Rig(guided=True);r.core.phase=phase
            r.now+=.26;r.core.update_feedback(PicoFeedback(r.now,True),r.now);r.core.tick(r.now)
            self.assertEqual(r.core.phase,'locked')
            self.assertEqual(r.core.drain()[-1].operation,'stop')
            if phase=='loading':self.assertEqual(r.core.load,'unknown')

    def test_support_confirmation_requires_released_squeezes_and_triggers(self):
        r=Rig(guided=True);r.core.loading_check(r.now);r.triggers=[1.,0.]
        r.step();r.click(1,1)
        self.assertEqual(r.core.phase,'load_check');self.assertFalse(r.core.supported)
        r.triggers=[0.,0.];r.grips=[1.,0.];r.step();r.click(1,1)
        self.assertEqual(r.core.phase,'load_check');self.assertFalse(r.core.supported)

    def test_shared_guide_reaches_hud_and_never_claims_sensor_proof(self):
        from bindu_teleoperation.vr.hud import hud_spec
        r=Rig(guided=True);r.core.loading_check(r.now)
        view=r.core.snapshot(r.now);view['mode']=view['phase']
        spec=hud_spec(view)
        self.assertIn('步骤 1/4',spec['title']);self.assertIn('外部支撑',spec['detail'])
        r.click(1,1)
        view=r.core.snapshot(r.now);view['mode']=view['phase']
        self.assertIn('步骤 2/4',hud_spec(view)['title'])



class SimulationDemoTests(unittest.TestCase):
    def fixture(self):
        rig=Rig(load='unknown')
        rig.core.simulation_demo=True
        return rig

    def test_default_never_infers_a_tool(self):
        r=Rig(load='unknown')
        r.core.simulation_fixture(True,r.now)
        self.assertEqual(r.core.load,'unknown')
        self.assertEqual(r.core.drain(),[])

    def test_initial_tracking_acquisition_stays_checking_but_active_loss_stops(self):
        r=Rig()
        left,right=r.core.frames
        core=PicoControl()
        core.ingest(replace(left,valid=False,pose=()),replace(right,valid=False,pose=()),r.now)
        self.assertEqual(core.phase,'checking')
        self.assertEqual(core.drain(),[])
        r.core.ingest(replace(left,valid=False,pose=()),right,r.now)
        self.assertEqual(r.core.phase,'locked')
        self.assertEqual(r.core.drain()[-1].operation,'stop')

    def test_real_pico_near_rotation_remains_rigid_after_zero_motion_mapping(self):
        from bindu_teleoperation.vr.mapping import pose_matrix,robot_pose,relative_target
        # Observed live XR matrix: accepted at 1e-3; uncorrected composition
        # exceeded the QP's 1e-5 rotation check even with no operator motion.
        pose=np.array([[.5447476506233215,-.7995926141738892,-.25289684534072876,.066],
            [.83836829662323,.511938750743866,.1872648298740387,.774],
            [-.020273271948099136,-.31402915716171265,.9492216110229492,-.321],
            [0.,0.,0.,1.]])
        rigid=pose_matrix(pose)
        np.testing.assert_allclose(rigid[:3,:3].T@rigid[:3,:3],np.eye(3),atol=1e-12)
        np.testing.assert_array_equal(rigid[:3,3],pose[:3,3])
        target=np.array(relative_target(robot_pose(pose),robot_pose(pose),np.eye(4),mapping_mode='robot_base')).reshape(4,4)
        np.testing.assert_allclose(target,np.eye(4),atol=1e-12)
        for malformed in (np.diag([1.01,1.,1.,1.]),np.diag([-1.,1.,1.,1.])):
            with self.assertRaises(ValueError):pose_matrix(malformed)

    def test_demo_empty_start_stays_stationary_and_prepared_attaches_virtual_tool(self):
        r=self.fixture()
        r.core.simulation_fixture(False,r.now)
        self.assertEqual(r.core.load,'empty')
        self.assertEqual(r.core.drain(),[])
        r.core.reason='PREPARED'
        r.core.simulation_fixture(True,r.now)
        self.assertEqual(r.core.load,'held')
        self.assertEqual(r.core.drain()[-1].operation,'confirm_tool')

    def test_existing_prepared_fixture_only_requests_readiness(self):
        r=self.fixture()
        r.core.simulation_fixture(True,r.now)
        self.assertEqual(r.core.phase,'confirming_tool')
        self.assertEqual([i.operation for i in r.core.drain()],['confirm_tool'])

    def test_demo_cannot_clear_stop_or_use_stale_held_input(self):
        for condition in ('locked','stale','held_squeeze','moving'):
            r=self.fixture()
            if condition=='locked':r.core.phase='locked'
            elif condition=='stale':r.now+=1.
            elif condition=='held_squeeze':r.grips=[1.,1.];r.step()
            elif condition=='moving':r.core.feedback=PicoFeedback(r.now,False)
            r.core.simulation_fixture(True,r.now)
            self.assertEqual(r.core.load,'unknown',condition)
            self.assertEqual(r.core.drain(),[],condition)

    def test_stale_snapshot_never_advertises_old_buttons(self):
        r=self.fixture()
        view=r.core.snapshot(r.now+1.)
        self.assertFalse(view['input_fresh'])
        for value in view['controller_states'].values():
            self.assertIsNone(value['buttons'])
            self.assertIsNone(value['squeeze'])



if __name__ == "__main__":
    unittest.main()
