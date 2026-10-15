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


class Rig:
    def __init__(self,load='empty'):
        self.core=PicoControl(initial_load=load)
        self.now=1.;self.seq=0;self.source='headset-1'
        self.buttons=[0,0];self.triggers=[0.,0.];self.grips=[0.,0.]
        self.counters=[ButtonCounters(),ButtonCounters()]
        self.modes=('closed','closed');self.stopped=True
        self.step();self.step()

    def step(self,dt=.02):
        self.now+=dt;self.seq+=1
        self.core.update_feedback(PicoFeedback(self.now,self.stopped,self.modes),self.now)
        frames=[]
        for i,side in enumerate(('left','right')):
            pose=np.eye(4);pose[0,3]=i*.34
            frame=VRFrame(self.source,self.seq,self.now,side,tuple(pose.ravel()),self.grips[i],
                          self.triggers[i],mapping=PICO_MAPPING,buttons=self.buttons[i])
            frames.append(self.counters[i].update(frame))
        self.core.ingest(*frames,self.now)
        return self.core.drain()

    def click(self,side,bit):
        self.buttons[side]|=1<<bit;effects=self.step()
        self.buttons[side]&=~(1<<bit);effects+=self.step()
        return effects

    def complete(self,ok=True,reason=''):
        intent=self.core.pending
        self.now+=.01
        self.core.update_feedback(PicoFeedback(self.now,True,self.modes),self.now)
        self.core.complete(intent.token,ok,reason,self.now)
        return intent

    def context(self,load=None,supported=True):
        self.core.context(self.source,self.core.generation,load or self.core.load,supported,self.now)
        self.step()

    def loading(self):
        self.context()
        intents=self.click(0,2)
        assert intents[-1].operation=='stop'
        self.complete();self.click(1,1)
        assert self.core.phase=='loading'

    def held(self):
        self.context('held')
        self.click(1,1);self.complete();self.step()
        assert self.core.phase=='tool_standby'


class MappingTests(unittest.TestCase):
    def event(self):
        pose=np.eye(4).ravel(order='F').tolist()
        state=dict(squeezeValue=.8,triggerValue=.9,aButton=False,bButton=False,thumbstick=False)
        return dict(left=pose,right=pose,leftState=dict(state),rightState=dict(state))

    def test_pico_a_b_do_not_initialize_separate_arms(self):
        event=self.event();event['rightState']['aButton']=True;event['rightState']['bButton']=True
        for side in ('left','right'):
            frame=decode_controller(event,side,'id',1,1.,PICO_MAPPING)
            self.assertTrue(frame.valid);self.assertFalse(frame.init)
        self.assertTrue(decode_controller(event,'left','id',1,1.).init)

    def test_pico_status_panel_does_not_show_single_arm_or_legacy_trigger_mapping(self):
        from bindu_teleoperation.vr.display import status_lines
        lines=status_lines(dict(pico=True,mode='loading',reason='WAIT',load_state='held',
                                external_support=True,gripper_modes=['closed','open']))
        self.assertEqual(len(lines),7)
        text=' '.join(line for line,_ in lines)
        self.assertIn('PICO',text);self.assertIn('Triggers open',text);self.assertIn('X virtual',text)
        self.assertNotIn('SINGLE ARM',text)

    def test_real_position_panel_only_advertises_supported_controls(self):
        from bindu_teleoperation.vr.display import status_lines
        text=' '.join(line for line,_ in status_lines(dict(runtime_mode='real')))
        self.assertIn('REAL',text);self.assertIn('Squeeze',text)
        self.assertIn('stop only',text)
        self.assertNotIn('SIMULATION',text);self.assertNotIn('Triggers open',text)
        self.assertNotIn('X virtual',text)

    def test_unknown_tool_presence_cannot_disable_collision_checks(self):
        from bindu_kinematics.tool_geometry import make_collision_checker
        with self.assertRaisesRegex(ValueError,'INVALID_TOOL_PRESENCE'):
            make_collision_checker(Path('/not-a-model'),{},tool_present=None)

    def test_missing_physical_button_is_invalid_in_pico_profile(self):
        event=self.event();del event['leftState']['thumbstick']
        self.assertFalse(decode_controller(event,'left','id',1,1.,PICO_MAPPING).valid)

    def test_complete_stop_click_survives_latest_pose_coalescing(self):
        rig=Rig();left,right=rig.core.frames
        left=rig.counters[0].update(replace(left,seq=10,buttons=2))
        left=rig.counters[0].update(replace(left,seq=11,buttons=0,stamp=rig.now+.01))
        right=replace(right,seq=11,stamp=rig.now+.01)
        rig.core.ingest(left,right,rig.now+.01)
        self.assertEqual(rig.core.phase,'locked')
        self.assertEqual(rig.core.reason,'PICO_Y_STOP')

    def test_right_trigger_never_requests_pico_virtual_grasp(self):
        cfg=json.loads((Path(__file__).resolve().parents[1]/'src/integration/bindu_runtime/config/robots/snowbot/bimanual_snowbot_shovel.json').read_text())
        session=BimanualSession(cfg,1,0.)
        rig=Rig('held');a,b=rig.core.frames
        session.ingest(a,b,rig.now)
        rig.triggers[1]=1.;rig.grips=[1.,1.];rig.step()
        a,b=rig.core.frames;session.ingest(a,b,rig.now)
        self.assertEqual(session.grasp_mode,'rigid')
        session.ingest(replace(a,seq=a.seq+1,stamp=a.stamp+.01),
                       replace(b,seq=b.seq+1,stamp=b.stamp+.01,virtual_grip=True),rig.now+.01)
        self.assertEqual(session.grasp_mode,'right_axial')


class GraspContinuationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg=json.loads((Path(__file__).resolve().parents[1]/'src/integration/bindu_runtime/config/robots/snowbot/bimanual_snowbot_shovel.json').read_text())

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


class StateTests(unittest.TestCase):
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
        rig=Rig('held');rig.core.phase='following';left,right=rig.core.frames
        pose=np.eye(4);pose[0,3]=1.
        rig.core.ingest(replace(left,pose=tuple(pose.ravel()),seq=left.seq+1,stamp=rig.now+.01),
                        replace(right,seq=right.seq+1,stamp=rig.now+.01),rig.now+.01)
        self.assertEqual(rig.core.phase,'locked')
        self.assertEqual(rig.core.reason,'PICO_POSE_ORIGIN_JUMP')

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
        rig.buttons[1]=0
        self.assertEqual(rig.step()[-1].operation,'stop')

    def test_prepare_rejected_while_held_does_not_run_when_condition_improves(self):
        rig=Rig('held');rig.buttons[1]=1;rig.step()
        for _ in range(60):self.assertEqual(rig.step(),[])
        rig.core.load='empty'
        for _ in range(60):self.assertEqual(rig.step(),[])

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


if __name__=='__main__':unittest.main()
