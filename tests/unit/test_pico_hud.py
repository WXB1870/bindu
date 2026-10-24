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


class PicoHUDTests(unittest.TestCase):
    def test_device_barrier_or_stop_failure_never_offers_b_recovery(self):
        from bindu_teleoperation.vr.hud import hud_spec
        base=self.view('locked',stop_confirmed=True,fault_active=True)
        for details in (dict(recovery_required='device_inspection'),
                        dict(stop_failure=dict(code='PICO_STOP_UNCONFIRMED'))):
            spec=hud_spec({**base,**details})
            self.assertNotIn('B 恢复',spec['action'])

    def test_arrival_timeout_does_not_claim_arrival_or_unconfirmed_stop(self):
        from bindu_teleoperation.vr.hud import hud_spec
        for stopped in (False,True):
            view=dict(mode='locked',reason='RuntimeError:PICO_ACTUATOR_FEEDBACK_TIMEOUT:arm_r_joint_7:error_rad=0.14543',
                      stop_confirmed=stopped)
            spec=hud_spec(view)
            self.assertEqual(spec['title'],'未到目标位置')
            self.assertEqual('已停稳' in spec['detail'],stopped)
            self.assertEqual('B 恢复待命' in spec['action'],stopped)

    def view(self,mode='following',**values):
        return dict(pico=True,mode=mode,reason='FOLLOWING',input_fresh=True,raw_input_valid=True,
            grasp_applied='rigid',grasp_requested='rigid',**values)

    def test_direct_gripper_phase_shows_trigger_guidance(self):
        from bindu_teleoperation.vr.hud import hud_spec
        spec=hud_spec(self.view('gripper_control',teleop_mode='independent_arms'))
        self.assertEqual(spec['status'],'夹爪开合中')
        self.assertIn('松开闭合',spec['detail'])
        self.assertIn('双侧握松开',spec['detail'])

    def test_a_wait_during_gripper_close_uses_shared_operator_guidance(self):
        from bindu_teleoperation.vr.hud import hud_spec
        guide=dict(title='等待夹爪闭合停稳',detail='保持按住 A；停稳后重新计时',key='A',step='prepare_wait')
        spec=hud_spec(self.view('gripper_control',operator_guide=guide))
        self.assertEqual((spec['title'],spec['detail']),(guide['title'],guide['detail']))

    def test_mode_banner_uses_confirmed_mode_and_clears_stale_context(self):
        from bindu_teleoperation.vr.hud import hud_spec
        from bindu_teleoperation.vr.display import render
        for mode,name,hint in [('independent_arms','普通双臂','左右手柄各控一臂'),
                               ('bimanual_tool','扫雪耦合','双手柄共同控制工具')]:
            view=self.view(teleop_mode=mode)
            spec=hud_spec(view)
            self.assertEqual((spec['context'],spec['mode_hint']),(name,hint))
            hud=render(view).serialize()['children'][0]
            self.assertEqual(hud['userData']['control_mode'],name)
            self.assertEqual(hud['userData']['control_hint'],hint)
        view=self.view('selecting_mode',teleop_mode='independent_arms',mode_target='bimanual_tool')
        spec=hud_spec(view)
        self.assertEqual(spec['context'],'普通双臂')
        self.assertIn('扫雪耦合',spec['title'])
        view['mode']='DISPLAY_STALE'
        spec=hud_spec(view)
        self.assertEqual(spec['context'],'模式待确认')
        self.assertNotIn('共同控制',spec['mode_hint'])

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

    def test_current_limit_is_explained_without_claiming_a_fault_stop(self):
        from bindu_teleoperation.vr.hud import hud_spec
        limit=dict(scale=0.,first_rejection='SELF_COLLISION: arm / head',search_limited_by_budget=False)
        spec=hud_spec(self.view(motion_limit=limit))
        self.assertEqual(spec['status'],'跟随受限')
        # Exercise PNG rendering as well as the textual spec: the previous
        # 'warn' tone was absent from hud_image's palette and killed the HUD.
        from bindu_teleoperation.vr.display import render
        scene=render(self.view(motion_limit=limit)).serialize()
        self.assertTrue(scene['children'][0]['src'].startswith('data:image/png;base64,'))
        self.assertIn('本体',spec['title']);self.assertIn('反方向',spec['detail'])
        limit['search_limited_by_budget']=True
        self.assertEqual(hud_spec(self.view(motion_limit=limit))['title'],'控制更新暂时延迟')
        spec=hud_spec({**self.view(motion_limit=limit),'input_fresh':False})
        self.assertEqual(spec['status'],'输入失效')

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



if __name__ == "__main__":
    unittest.main()
