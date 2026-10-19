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
    def __init__(self,load='empty',guided=False,dual=False):
        self.core=PicoControl(initial_load=load,guided_loading=guided,dual_modes=dual)
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
        self.assertEqual(len(lines),3)
        text=' '.join(line for line,_ in lines)
        self.assertIn('双手工具',text);self.assertIn('装卸',text);self.assertIn('既有流程',text)
        self.assertNotIn('SINGLE ARM',text)

    def test_position_coordinator_uses_same_hud_in_both_modes(self):
        from bindu_teleoperation.vr.display import status_lines
        from bindu_teleoperation.vr.hud import hud_spec
        view=dict(pico=True,mode='standby',load_state='empty',capabilities={'prepare':False})
        self.assertEqual(status_lines(dict(view,runtime_mode='simulation')),
                         status_lines(dict(view,runtime_mode='real')))
        self.assertEqual(hud_spec(view)['title'],'现场准备位尚未配置')

    def test_normal_hud_hides_raw_tracking_and_physical_keys(self):
        from bindu_teleoperation.vr.display import status_lines
        view=dict(pico=True,mode='following',reason='FOLLOWING',grasp_applied='rigid',input_fresh=True,controller_states={
            side:dict(valid=True,squeeze=.2,trigger=.4,buttons=1) for side in ('left','right')},
            raw_input_age_s=.02,raw_input_valid=True,simulation_demo=True)
        text=' '.join(line for line,_ in status_lines(view))
        for token in ('跟随中','刚握','Y 停止'):
            self.assertIn(token,text)
        for token in ('20 ms','X:1','A:1','0.2'):
            self.assertNotIn(token,text)
        self.assertEqual(len(status_lines(view)),1)
        view['raw_input_valid']=False
        self.assertIn('手柄输入不可用',' '.join(line for line,_ in status_lines(view)))

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


class PicoHUDTests(unittest.TestCase):
    def view(self,mode='following',**values):
        return dict(pico=True,mode=mode,reason='FOLLOWING',input_fresh=True,raw_input_valid=True,
            grasp_applied='rigid',grasp_requested='rigid',**values)

    def test_reference_six_states_use_feedback_and_hide_normal_notice(self):
        from bindu_teleoperation.vr.hud import hud_spec
        following=hud_spec(self.view())
        self.assertEqual((following['state'],following['title']),('following',''))
        virtual=hud_spec({**self.view(), 'grasp_applied':'right_axial'})
        self.assertEqual((virtual['state'],virtual['grip'],virtual['title']),('virtual','右手虚握',''))
        paused=hud_spec(self.view('tool_standby',stop_confirmed=True))
        self.assertIn('松开双侧握',paused['title'])
        switching=hud_spec({**self.view('switching'),'grasp_requested':'right_axial'})
        self.assertEqual((switching['state'],switching['grip']),('switching','刚握'))
        self.assertIn('尚未进入右手虚握',switching['detail'])
        lost=hud_spec({**self.view('locked'),'reason':'PICO_PAIR_INVALID','tracking_valid':{'left':True,'right':False}})
        self.assertEqual((lost['state'],lost['title']),('lost','右手柄输入失效'))
        self.assertIn('等待停稳确认',lost['detail'])
        stopped=hud_spec({**self.view('locked',stop_confirmed=True),'reason':'PICO_Y_STOP'})
        self.assertEqual((stopped['state'],stopped['title'],stopped['action']),('stopped','已确认停稳','B 恢复待命'))

    def test_y_request_and_stale_observer_never_claim_confirmed_stop(self):
        from bindu_teleoperation.vr.hud import hud_spec
        from bindu_teleoperation.feedback import display_snapshot
        view={**self.view('locked'),'reason':'PICO_Y_STOP'}
        self.assertEqual(hud_spec(view)['status'],'停止未确认')
        view.update(stop_confirmed=True,feedback_age_s=.02,input_age_s=.02)
        stale=display_snapshot(view,.3)
        self.assertFalse(stale['stop_confirmed'])
        self.assertFalse(stale['input_fresh'])
        self.assertEqual(hud_spec(stale)['status'],'停止未确认')
        self.assertEqual(hud_spec(display_snapshot(view,1.))['status'],'状态未知')

    def test_pico_render_has_one_small_image_hud_and_no_diagnostic_axes(self):
        from bindu_teleoperation.vr.display import render
        scene=render(self.view()).serialize()
        self.assertEqual(len(scene['children']),1)
        hud=scene['children'][0]
        self.assertEqual(hud['tag'],'ImageBackground')
        self.assertEqual(hud['geometry'],'plane')
        self.assertEqual(hud['userData']['hud_state'],'following')
        self.assertEqual(hud['userData']['title'],'')
        self.assertTrue(hud['src'].startswith('data:image/png;base64,'))

    def test_model_uses_fresh_measured_joints_and_local_urdf(self):
        from bindu_teleoperation.vr.display import render
        scene=render(self.view(model_url='/static/snowbot.urdf',model_joints={'joint':.2})).serialize()
        self.assertEqual(len(scene['children']),2)
        model=scene['children'][0]['children'][0]
        self.assertEqual(model['tag'],'Urdf')
        self.assertEqual(model['src'],'/static/snowbot.urdf')
        self.assertEqual(model['jointValues'],{'joint':.2})
        self.assertFalse(model['parseCollision'])


class ShortRodFixtureTests(unittest.TestCase):
    def test_midpoint_rebase_removes_shovel_lever_without_changing_grasp(self):
        from bindu_teleoperation.simulation_fixture import short_rod_config
        import copy
        path=Path(__file__).resolve().parents[1]/'src/integration/bindu_runtime/config/robots/snowbot/bimanual_snowbot_shovel.json'
        original=json.loads(path.read_text());before=copy.deepcopy(original)
        rod=short_rod_config(original)
        self.assertEqual(original,before)
        points=[]
        for side,key in (('left','grip_offset_m'),('right','right_grip_offset_m')):
            mount=np.array(rod[side+'_to_tool']).reshape(4,4)
            points.append((np.linalg.inv(mount) @ np.r_[rod['demo'][key],1.])[:3])
        np.testing.assert_allclose((points[0]+points[1])/2,0.,atol=1e-12)
        old_l=np.array(original['left_to_tool']).reshape(4,4);old_r=np.array(original['right_to_tool']).reshape(4,4)
        new_l=np.array(rod['left_to_tool']).reshape(4,4);new_r=np.array(rod['right_to_tool']).reshape(4,4)
        np.testing.assert_allclose(old_l @ np.linalg.inv(old_r),new_l @ np.linalg.inv(new_r),atol=1e-12)
        self.assertAlmostEqual(np.diff(rod['demo']['shaft_from_left_grip_m'])[0],.56)
        self.assertEqual(rod['position_scale'],.5)
        self.assertEqual(rod['demo']['geometry_kind'],'rod')
        self.assertNotIn('receiving_bin',rod['demo'])
        # Change only the old tool-origin/length convention. The short rod's
        # mount and operator leverage must remain exactly the same.
        shifted=copy.deepcopy(original);offset=np.eye(4);offset[2,3]=.6
        for side in ('left','right'):
            shifted[side+'_to_tool']=(np.array(original[side+'_to_tool']).reshape(4,4) @ offset).ravel().tolist()
        shifted_rod=short_rod_config(shifted)
        np.testing.assert_allclose(shifted_rod['left_to_tool'],rod['left_to_tool'],atol=1e-12)
        with self.assertRaisesRegex(ValueError,'REQUIRES_SIMULATION'):
            short_rod_config(dict(original,simulation_only=False))

    def test_display_has_only_the_short_rod_and_shares_the_solver_mount(self):
        from bindu_teleoperation.simulation_fixture import short_rod_config
        from bindu_teleoperation.vr.model import prepare_model
        import tempfile,xml.etree.ElementTree as ET
        root=Path(__file__).resolve().parents[1]
        cfg=root/'src/integration/bindu_runtime/config/robots/snowbot/bimanual_snowbot_shovel.json'
        urdf=root/'src/hardware/bindu_description/urdf/snowbot_display.urdf'
        rod=short_rod_config(json.loads(cfg.read_text()))
        with tempfile.TemporaryDirectory() as directory:
            prepare_model(urdf,directory,cfg,short_rod=True)
            tree=ET.parse(Path(directory)/'snowbot-tool.urdf')
            visuals=tree.findall("link[@name='bindu_virtual_tool']/visual")
            self.assertEqual([v.get('name') for v in visuals],['shaft'])
            self.assertAlmostEqual(float(visuals[0].find('geometry/cylinder').get('length')),.56)
            origin=tree.find("joint[@name='bindu_virtual_tool_mount']/origin")
            np.testing.assert_allclose(np.fromstring(origin.get('xyz'),sep=' '),np.array(rod['left_to_tool']).reshape(4,4)[:3,3])


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


if __name__=='__main__':unittest.main()
