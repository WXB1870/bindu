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


class DualModeTests(unittest.TestCase):
    def test_default_normal_accepts_one_hand_and_releases_independently(self):
        r=Rig('unknown',guided=True,dual=True)
        self.assertEqual(r.core.teleop_mode,'independent_arms')
        r.grips=[1.,0.];r.step();self.assertEqual(r.core.pending.operation,'follow')
        r.complete();r.stopped=False;r.grips=[1.,1.];r.step()
        r.click(0,0);self.assertIsNone(r.core.pending)
        r.grips=[0.,1.];self.assertEqual(r.step(),[])
        self.assertEqual(r.core.phase,'following')
        r.grips=[0.,0.];r.step();self.assertEqual(r.core.pending.operation,'stop')

    def test_b_switches_only_at_released_rest_and_does_not_follow(self):
        r=Rig(guided=True,dual=True);r.click(1,1)
        self.assertEqual((r.core.pending.operation,r.core.pending.value),('select_mode','bimanual_tool'))
        self.assertEqual(r.core.teleop_mode,'independent_arms')
        r.complete();self.assertEqual(r.core.teleop_mode,'bimanual_tool')
        self.assertEqual(r.core.phase,'standby');self.assertIsNone(r.core.pending)
        r.step();r.click(1,1);r.complete()
        self.assertEqual(r.core.teleop_mode,'independent_arms')

    def test_rejected_tool_mode_keeps_normal_mode_and_specific_reason(self):
        r=Rig(guided=True,dual=True);r.click(1,1)
        r.core.complete(r.core.pending.token,False,'RuntimeError:BIMANUAL_GRASP_POSE',r.now,rejected=True)
        self.assertEqual(r.core.teleop_mode,'independent_arms')
        self.assertEqual(r.core.phase,'standby');self.assertIsNone(r.core.pending)

    def test_known_coupled_load_cannot_enter_independent_arms(self):
        r=Rig('held',guided=True,dual=True);r.grips=[1.,0.];r.step()
        self.assertIsNone(r.core.pending)
        self.assertEqual(r.core.reason,'PICO_TOOL_HELD_REQUIRES_TOOL_MODE')
        r.core.teleop_mode='bimanual_tool';r.grips=[0.,0.];r.step();r.click(1,1)
        self.assertIsNone(r.core.pending);self.assertEqual(r.core.teleop_mode,'bimanual_tool')

    def test_mode_key_while_following_only_stops_then_requires_new_press(self):
        r=Rig(guided=True,dual=True);r.grips=[1.,0.];r.step();r.complete()
        r.click(1,1);self.assertEqual(r.core.pending.operation,'stop');r.complete()
        self.assertEqual(r.core.teleop_mode,'independent_arms')
        r.grips=[0.,0.];r.step();self.assertIsNone(r.core.pending)
        r.click(1,1);self.assertEqual(r.core.pending.operation,'select_mode')



class StateTests(unittest.TestCase):
    def test_pair_skew_compares_each_hand_with_its_own_previous_time(self):
        rig=Rig();core=rig.core;t=rig.now+.02
        pair=tuple(replace(f,seq=rig.seq+1,stamp=t+i*.0007) for i,f in enumerate(core.frames))
        core.ingest(*pair,t+.005)
        newer=tuple(replace(f,seq=f.seq+1,stamp=t+dt) for f,dt in zip(pair,(.00056,.001)))
        core.ingest(*newer,t+.006)
        self.assertEqual(core.seq,newer[0].seq)
        self.assertEqual(core.phase,'standby')

    def test_real_time_reversal_of_either_hand_still_locks_and_stops(self):
        for side in (0,1):
            with self.subTest(side=side):
                rig=Rig();core=rig.core;previous=core.frames
                pair=tuple(replace(f,seq=f.seq+1,stamp=f.stamp+(-.0001 if i==side else .001))
                           for i,f in enumerate(previous))
                core.ingest(*pair,rig.now+.005)
                self.assertEqual((core.phase,core.reason),('locked','PICO_TIME_REVERSED'))
                self.assertEqual(core.pending.operation,'stop')
                self.assertEqual(core.frames,previous)

    def test_stop_fault_keeps_its_cause_instead_of_claiming_feedback_loss(self):
        rig=Rig();core=rig.core
        code='PICO_STOP_UNCONFIRMED:STOP_UNCONFIRMED:vendor_hold'
        core.update_feedback(PicoFeedback(rig.now,False,healthy=False,fault=code),rig.now)
        self.assertEqual((core.phase,core.reason),('locked',code))
        self.assertFalse(core.rest(rig.now))

    def test_startup_input_before_feedback_does_not_latch_timeout_or_use_stale_pair(self):
        core=PicoControl(initial_load='unknown')
        rig=Rig()
        core.ingest(*rig.core.frames,rig.now)
        later=rig.now+1.
        core.tick(later)
        self.assertEqual(core.phase,'checking')
        core.update_feedback(PicoFeedback(later,True),later)
        self.assertEqual(core.phase,'checking')
        rig.step(1.01)
        core.ingest(*rig.core.frames,rig.now)
        core.update_feedback(PicoFeedback(rig.now,True),rig.now)
        self.assertEqual(core.phase,'standby')
        core.tick(rig.now+1.)
        self.assertEqual((core.phase,core.reason),('locked','PICO_INPUT_TIMEOUT'))

    def test_initial_feedback_discovery_waits_without_latching_or_moving(self):
        rig=Rig();core=PicoControl(initial_load='empty')
        core.update_feedback(PicoFeedback(0.,False,healthy=False),rig.now)
        core.ingest(*rig.core.frames,rig.now)
        core.update_feedback(PicoFeedback(0.,False,healthy=False),rig.now)
        core.tick(rig.now)
        self.assertEqual(core.phase,'checking');self.assertEqual(core.drain(),[])
        core.update_feedback(PicoFeedback(rig.now,True,('closed','closed')),rig.now)
        self.assertEqual(core.phase,'standby')

    def test_boot_has_no_effects_and_squeeze_is_not_gripper_open(self):
        rig=Rig();self.assertEqual(rig.core.phase,'standby')
        rig.grips=[1.,1.];self.assertEqual(rig.step(),[])
        self.assertEqual(rig.core.desired,['closed','closed'])

    def test_trigger_controls_only_its_gripper_and_release_closes(self):
        rig=Rig();rig.loading()
        rig.triggers[0]=1.;effects=rig.step()
        self.assertEqual(effects[-1].value,('open','closed'))
        rig.modes=('open','closed');rig.complete()
        rig.triggers[0]=0.;effects=rig.step()
        self.assertEqual(effects[-1].value,('closed','closed'))
        rig.modes=('closed','closed');rig.complete()
        rig.triggers[1]=1.;self.assertEqual(rig.step()[-1].value,('closed','open'))

    def test_trigger_hysteresis_does_not_close_at_half_press(self):
        rig=Rig();rig.loading();rig.triggers[0]=1.;rig.step();rig.modes=('open','closed');rig.complete()
        rig.triggers[0]=.5;self.assertEqual(rig.step(),[])
        self.assertEqual(rig.core.desired,['open','closed'])
        rig.triggers[0]=0.;self.assertEqual(rig.step()[-1].value,('closed','closed'))

    def test_prepare_held_during_loading_must_be_released_before_reuse(self):
        rig=Rig();rig.loading();rig.buttons[1]=1;rig.step()
        rig.core.phase='standby'
        for _ in range(70):self.assertEqual(rig.step(),[])

    def test_pose_origin_jump_locks_active_follow(self):
        rig=Rig('held');rig.grips=[1.,1.];rig.step();rig.core.phase='following';left,right=rig.core.frames
        pose=np.eye(4);pose[0,3]=1.
        rig.core.ingest(replace(left,pose=tuple(pose.ravel()),seq=left.seq+1,stamp=rig.now+.01),
                        replace(right,seq=right.seq+1,stamp=rig.now+.01),rig.now+.01)
        self.assertEqual(rig.core.phase,'locked')
        self.assertEqual(rig.core.reason,'PICO_POSE_ORIGIN_JUMP')
        self.assertEqual(rig.core.last_pose_jump['translation_limit_m'],.35)
        self.assertAlmostEqual(rig.core.last_pose_jump['translation_m'],1.)

    def test_moderate_displacement_and_wrist_rotation_survive_a_frame_gap(self):
        rig=Rig('held');rig.grips=[1.,1.];rig.step();rig.core.phase='following';left,right=rig.core.frames
        angle=.8;pose=np.eye(4);pose[:2,:2]=[[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]]
        pose[0,3]=.25
        rig.core.ingest(replace(left,pose=tuple(pose.ravel()),seq=left.seq+1,stamp=rig.now+.14),
                        replace(right,seq=right.seq+1,stamp=rig.now+.14),rig.now+.14)
        self.assertEqual(rig.core.phase,'following')
        self.assertIsNone(rig.core.last_pose_jump)

    def test_release_takes_priority_over_a_large_paused_hand_displacement(self):
        rig=Rig('held');rig.grips=[1.,1.];rig.step();rig.core.phase='following';left,right=rig.core.frames
        pose=np.eye(4);pose[0,3]=1.
        rig.core.ingest(replace(left,pose=tuple(pose.ravel()),grip=0.,seq=left.seq+1,stamp=rig.now+.01),
                        replace(right,seq=right.seq+1,stamp=rig.now+.01),rig.now+.01)
        self.assertNotEqual(rig.core.reason,'PICO_POSE_ORIGIN_JUMP')
        self.assertEqual(rig.core.pending.operation,'stop')

    def test_invalid_pose_jump_limits_are_rejected(self):
        for options in ({'pose_jump_translation_m':0.},{'pose_jump_rotation_rad':float('nan')},
                        {'pose_jump_rotation_rad':4.}):
            with self.assertRaisesRegex(ValueError,'POSE_JUMP_CONFIG'):PicoControl(**options)

    def test_invalid_pose_never_reaches_actuator_requests(self):
        rig=Rig();left,right=rig.core.frames
        rig.core.ingest(replace(left,pose=(float('nan'),)*16,seq=10),replace(right,seq=10),rig.now)
        self.assertEqual(rig.core.phase,'locked')
        self.assertEqual(rig.core.drain()[-1].operation,'stop')

    def test_two_triggers_and_cross_release_remain_independent(self):
        rig=Rig();rig.loading();rig.triggers=[1.,1.]
        self.assertEqual(rig.step()[-1].value,('open','open'))
        rig.modes=('open','open');rig.complete()
        rig.triggers[0]=0.
        self.assertEqual(rig.step()[-1].value,('closed','open'))

    def test_open_outside_loading_only_stops_and_is_not_deferred(self):
        rig=Rig();rig.triggers[1]=1.
        self.assertEqual(rig.step()[-1].operation,'stop')
        self.assertEqual(rig.core.desired,['closed','closed'])
        rig.complete();rig.context()
        rig.click(0,2);rig.complete();rig.click(1,1)
        self.assertEqual(rig.core.phase,'loading')
        self.assertEqual(rig.step(),[])

    def test_support_and_b_are_required_before_loading(self):
        rig=Rig('held');rig.click(0,2);rig.complete();rig.click(1,1)
        self.assertEqual(rig.core.phase,'load_check')
        rig.context(supported=True);rig.click(1,1)
        self.assertEqual(rig.core.phase,'loading')

    def test_prepare_requires_a_continuously_held_empty_and_released_squeeze(self):
        rig=Rig();rig.buttons[1]=1;rig.step()
        effects=[]
        for _ in range(60):effects+=rig.step()
        self.assertEqual([x.operation for x in effects],['prepare'])
        self.assertTrue(rig.core.last_prepare_check['accepted'])
        self.assertTrue(all(rig.core.last_prepare_check['checks'].values()))
        rig.buttons[1]=0
        self.assertEqual(rig.step()[-1].operation,'stop')

    def test_prepare_rejected_while_held_does_not_run_when_condition_improves(self):
        rig=Rig('held');rig.buttons[1]=1;rig.step()
        for _ in range(60):self.assertEqual(rig.step(),[])
        rig.core.load='empty'
        for _ in range(60):self.assertEqual(rig.step(),[])

    def test_prepare_failure_snapshot_retains_attempt_conditions(self):
        rig=Rig('unknown');rig.grips[0]=.8;rig.stopped=False;rig.buttons[1]=1
        for _ in range(60):self.assertEqual(rig.step(),[])
        check=rig.core.last_prepare_check
        self.assertFalse(check['accepted'])
        self.assertEqual({k for k,v in check['checks'].items() if not v},
                         {'load_empty','left_squeeze_released','motion_stopped'})
        self.assertEqual(check['load_source'],'unknown')
        rig.stopped=True;rig.grips[0]=0.;rig.step();rig.context('empty')
        self.assertEqual(rig.core.snapshot()['load_source'],'operator_declaration')
        self.assertTrue(all(rig.core.prepare_checks(rig.now).values()))
        self.assertIs(rig.core.last_prepare_check,check)
        self.assertEqual(check['load_state'],'unknown')
        from bindu_teleoperation.vr.hud import hud_spec
        hud=hud_spec(dict(mode='standby',reason=rig.core.reason,last_prepare_check=check))
        self.assertIn('空手状态未确认',hud['detail'])
        self.assertIn('左侧握未松开',hud['detail'])

    def test_startup_held_a_and_triggers_are_not_actions(self):
        rig=Rig();rig.core=PicoControl(initial_load='empty');rig.buttons[1]=1;rig.triggers=[1.,1.]
        for _ in range(80):self.assertEqual(rig.step(),[])

    def test_squeeze_release_stops_without_closing_virtual_grip(self):
        rig=Rig('held');rig.core.virtual=True;rig.core.phase='following';rig.grips=[1.,1.];rig.step()
        rig.grips[0]=0.;effects=rig.step()
        self.assertEqual(effects[-1].operation,'stop');self.assertTrue(rig.core.virtual)

    def test_trigger_during_preparation_or_x_switch_only_stops(self):
        for phase in ('preparing','following','switching','engaging'):
            rig=Rig('held');rig.core.phase=phase;rig.buttons[1]=1 if phase=='preparing' else 0
            rig.grips=[0.,0.] if phase=='preparing' else [1.,1.]
            rig.triggers[1]=1.;rig.buttons[0]=1
            effects=rig.step()
            self.assertEqual(effects[-1].operation,'stop')
            self.assertEqual(rig.core.desired,['closed','closed'])

    def test_y_has_priority_over_b_and_a(self):
        rig=Rig();rig.buttons=[2,3];effects=rig.step()
        self.assertEqual(rig.core.phase,'locked')
        self.assertEqual([x.operation for x in effects],['stop'])

    def test_timeout_and_late_completion_never_rearm(self):
        rig=Rig();rig.loading();rig.triggers[0]=1.;token=rig.step()[-1].token
        rig.now+=.3;rig.core.tick(rig.now)
        self.assertEqual(rig.core.phase,'locked')
        self.assertFalse(rig.core.complete(token,True,now=rig.now))
        self.assertEqual(rig.core.phase,'locked')

    def test_input_loss_during_open_does_not_synthesize_close(self):
        rig=Rig();rig.loading();rig.triggers[0]=1.;rig.step();rig.modes=('open','closed');rig.complete()
        rig.now+=.3;rig.core.tick(rig.now)
        self.assertEqual(rig.core.drain()[-1].operation,'stop')
        self.assertEqual(rig.core.desired,['open','closed'])

    def test_reconnect_locks_and_retired_source_cannot_take_over(self):
        rig=Rig();old=rig.core.frames;rig.source='new';rig.counters=[ButtonCounters(),ButtonCounters()]
        rig.step();self.assertEqual(rig.core.phase,'locked')
        rig.core.ingest(*old,rig.now)
        self.assertEqual(rig.core.source,'new')

    def test_coalesced_squeeze_release_stops_follow_or_preparation(self):
        for phase in ('following','preparing'):
            rig=Rig('held');rig.core.phase=phase
            left,right=rig.core.frames
            rig.core.ingest(replace(left,seq=left.seq+1,stamp=rig.now+.01,clutch_seq=left.clutch_seq+1),
                            replace(right,seq=right.seq+1,stamp=rig.now+.01),rig.now+.01)
            self.assertEqual(rig.core.drain()[-1].operation,'stop')
            self.assertNotEqual(rig.core.phase,phase)

    def test_source_change_invalidates_support_and_load_declarations(self):
        rig=Rig('held');rig.context('held',True)
        rig.source='new';rig.counters=[ButtonCounters(),ButtonCounters()];rig.step()
        self.assertFalse(rig.core.supported);self.assertEqual(rig.core.load,'unknown')

    def test_loading_completion_needs_fresh_load_declaration(self):
        rig=Rig();rig.loading();rig.click(1,1)
        self.assertEqual(rig.core.phase,'loading')
        self.assertEqual(rig.core.reason,'FRESH_LOAD_DECLARATION_REQUIRED')

    def test_context_is_fenced_and_never_recovers_a_fault(self):
        rig=Rig();rig.core.lock('fault',rig.now);rig.complete()
        with self.assertRaises(ValueError):rig.core.context(rig.source,0,'held',True,rig.now)
        rig.context('held');self.assertEqual(rig.core.phase,'locked')



if __name__ == "__main__":
    unittest.main()
