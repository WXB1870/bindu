"""Physical button identity, coalesced events and guarded operator transitions."""
from tests.support.paths import ROOT
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


class PicoGraspSwitchTests(unittest.TestCase):
    def test_new_clutch_after_paused_x_release_restores_rigid_before_follow(self):
        r=Rig('held',guided=True);r.core.phase='tool_standby';r.core.virtual=True
        r.modes=('closed','virtual');r.step()
        self.assertIsNone(r.core.pending)
        r.grips=[1.,1.];r.step()
        self.assertEqual((r.core.pending.operation,r.core.pending.value),('switch_grasp',False))
        r.modes=('closed','closed');r.complete()
        self.assertEqual(r.core.pending.operation,'follow')

    def test_x_release_during_entry_is_not_lost(self):
        r=Rig('held',guided=True);r.grips=[1.,1.];r.step();r.complete()
        r.buttons[0]=1;r.step()
        self.assertEqual((r.core.pending.operation,r.core.pending.value),('switch_grasp',True))
        r.buttons[0]=0;r.step();r.modes=('closed','virtual');r.complete()
        self.assertEqual((r.core.pending.operation,r.core.pending.value),('switch_grasp',False))
        r.modes=('closed','closed');r.complete()
        self.assertEqual((r.core.pending.operation,r.core.pending.value),('follow',False))

    def test_released_deadman_during_entry_never_queues_a_close_or_follow(self):
        r=Rig('held',guided=True);r.grips=[1.,1.];r.step();r.complete()
        r.buttons[0]=1;r.step();token=r.core.pending.token
        r.buttons[0]=0;r.grips=[0.,0.];r.step()
        self.assertFalse(r.core.complete(token,True,now=r.now))
        r.modes=('closed','virtual');r.complete()
        self.assertEqual(r.core.phase,'tool_standby');self.assertIsNone(r.core.pending)
        self.assertFalse(r.core.virtual)



class GraspContinuationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg=json.loads((ROOT/'src/integration/bindu_runtime/config/robots/snowbot/bimanual_snowbot_shovel.json').read_text())

    def feed(self,session,t,*,held=True,virtual=False,source='headset:coordinator',clutch=(0,0),mapping=PICO_MAPPING):
        frames=[]
        for i,side in enumerate(('left','right')):
            pose=np.eye(4);pose[0,3]=i*.34
            frames.append(VRFrame(source,round(t*1000),t,side,tuple(pose.ravel()),float(held),0.,
                clutch_seq=clutch[i],mapping=mapping,virtual_grip=virtual))
        session.ingest(*frames,t)

    def prior(self):
        session=BimanualSession(self.cfg,1,0.)
        self.feed(session,.01,held=False);self.feed(session,.02)
        self.feed(session,1.03);self.assertEqual(session.mode,'follow')
        session.capture(np.eye(4))
        # Coordinator projects a release while the physical counter stays put.
        self.feed(session,1.04,held=False)
        return session

    def test_first_engagement_still_requires_full_hold(self):
        session=BimanualSession(self.cfg,1,0.)
        self.feed(session,.01,held=False);self.feed(session,.02);self.feed(session,.12)
        self.assertEqual(session.mode,'idle');self.assertFalse(session.resumed_grasp)
        self.feed(session,1.03);self.assertEqual(session.mode,'follow')

    def test_continuous_grasp_switch_uses_short_hold_and_fresh_capture(self):
        proof=self.prior().stopped_continuation(1.05)
        self.assertIsNotNone(proof)
        session=BimanualSession(self.cfg,2,1.06,continuation=proof)
        self.feed(session,1.07,held=False,virtual=True);self.feed(session,1.08,virtual=True)
        self.feed(session,1.13,virtual=True);self.assertEqual(session.mode,'idle')
        self.feed(session,1.17,virtual=True)
        self.assertEqual(session.mode,'follow');self.assertTrue(session.resumed_grasp)
        self.assertIsNone(session.anchor_tool);self.assertIsNone(session.target)
        measured=np.eye(4);measured[0,3]=.2;session.capture(measured)
        np.testing.assert_allclose(session.target,measured)

    def test_source_release_expiry_same_grasp_and_non_pico_cannot_resume_fast(self):
        proof=self.prior().stopped_continuation(1.05)
        for start,changes in ((1.07,dict(source='other')),(1.07,dict(clutch=(1,0))),
                              (2.2,{}),(1.07,dict(virtual=False)),(1.07,dict(mapping='other'))):
            with self.subTest(start=start,changes=changes):
                session=BimanualSession(self.cfg,2,1.06,continuation=proof)
                kwargs=dict(virtual=True);kwargs.update(changes)
                self.feed(session,start,held=False,**kwargs)
                self.feed(session,start+.01,**kwargs);self.feed(session,start+.12,**kwargs)
                self.assertEqual(session.mode,'idle');self.assertFalse(session.resumed_grasp)

    def test_release_during_short_hold_discards_credit_and_old_grasp(self):
        session=BimanualSession(self.cfg,2,1.06,continuation=self.prior().stopped_continuation(1.05))
        self.feed(session,1.07,held=False,virtual=True);self.feed(session,1.08,virtual=True)
        self.feed(session,1.1,held=False,virtual=True,clutch=(1,0))
        self.feed(session,1.12,virtual=True,clutch=(1,0));self.feed(session,1.23,virtual=True,clutch=(1,0))
        self.assertEqual(session.mode,'idle');self.assertFalse(session.resumed_grasp)
        self.assertIsNone(session.stopped_continuation(1.24))

    def test_no_capture_stale_input_or_physical_release_cannot_issue_credit(self):
        self.assertIsNone(BimanualSession(self.cfg,1,0.).stopped_continuation(1.))
        session=self.prior();self.assertIsNone(session.stopped_continuation(1.5))
        self.feed(session,1.05,held=False,clutch=(1,0))
        self.assertIsNone(session.stopped_continuation(1.06))

    def test_clock_rollback_cannot_reuse_confirmation_from_the_future(self):
        proof=self.prior().stopped_continuation(1.05)
        session=BimanualSession(self.cfg,2,0.,continuation=proof)
        self.feed(session,.01,held=False,virtual=True);self.feed(session,.02,virtual=True)
        self.feed(session,.12,virtual=True)
        self.assertEqual(session.mode,'idle');self.assertFalse(session.resumed_grasp)



if __name__ == "__main__":
    unittest.main()
