"""Physical button identity, coalesced events and guarded operator transitions."""
from bindu_contracts.resources import package_share
from tests.support.paths import ROOT
from dataclasses import replace
import unittest
import json
from pathlib import Path
import numpy as np
from bindu_contracts.teleoperation import VRFrame
from bindu_teleoperation.vr.receiver import decode_controller, ButtonCounters
from snowbot_teleop.control import PicoControl, PicoFeedback, PICO_MAPPING
from bindu_teleoperation.bimanual_session import BimanualSession


from tests.support.pico import Rig


class HeadsetMappingTests(unittest.TestCase):
    def frame(self, yaw=0.,pitch=0.,roll=0., **changes):
        from scipy.spatial.transform import Rotation
        from bindu_teleoperation.vr.mapping import OPENXR_TO_ROBOT
        head=np.eye(4);head[:3,:3]=Rotation.from_euler('ZYX',[yaw,pitch,roll]).as_matrix()
        raw=OPENXR_TO_ROBOT.T @ head @ OPENXR_TO_ROBOT
        return replace(VRFrame('headset',1,10.,'left',tuple(np.eye(4).ravel())),
            head_pose=tuple(raw.ravel()),head_source_id='headset',head_seq=1,head_stamp=9.99,
            head_age_s=.005,**changes)

    def test_forward_calibration_ignores_pitch_and_roll_and_preserves_trim(self):
        from bindu_teleoperation.vr.mapping import engagement_yaw
        cfg=dict(direction_reference='headset_on_engage',operator_yaw_rad=.2,input_max_age=.25)
        for yaw in (0.,np.pi/2,-np.pi/2,2.8):
            for pitch,roll in ((0.,0.),(.6,-.8),(-.7,.9)):
                self.assertAlmostEqual(engagement_yaw(self.frame(yaw,pitch,roll),cfg,10.),.2-yaw)

    def test_missing_stale_foreign_and_vertical_headings_are_not_fabricated(self):
        from bindu_teleoperation.vr.mapping import engagement_yaw
        cfg=dict(direction_reference='headset_on_engage',input_max_age=.25)
        for change in (dict(head_pose=()),dict(head_source_id='other'),dict(head_seq=0),
                       dict(head_stamp=9.7),dict(head_stamp=10.1),dict(head_age_s=.3),
                       dict(head_age_s=float('nan'))):
            with self.subTest(change=change),self.assertRaisesRegex(ValueError,'HEADSET_UNAVAILABLE'):
                engagement_yaw(replace(self.frame(),**change),cfg,10.)
        with self.assertRaisesRegex(ValueError,'HEADING_UNDEFINED'):
            engagement_yaw(self.frame(pitch=np.pi/2),cfg,10.)

    def test_repeated_head_sample_keeps_original_receipt_stamp_and_age(self):
        from bindu_teleoperation.vr.receiver import HeadsetSamples
        head=HeadsetSamples();matrix=np.eye(4);matrix[0,3]=.2
        controllers=dict(left=np.eye(4).ravel(order='F').tolist(),right=np.eye(4).ravel(order='F').tolist())
        sample=dict(seq=1.,frame_id='1:2:0',tracking_id='1:0',frame_time_ms=100.,controllers=controllers,
            mode='immersive-vr',age_ms=5.,pose=matrix.ravel(order='F').tolist())
        def ingest(sample,now):return head.ingest(dict(version=1,samples=[sample]),now,controllers)
        first=ingest(sample,10.)
        self.assertEqual(first['head_pose'],tuple(matrix.ravel()))
        self.assertEqual(ingest(dict(sample,age_ms=105.),10.1),first)
        self.assertEqual(ingest(dict(sample,age_ms=300.),10.3),{})
        self.assertEqual(ingest(dict(sample,mode='inline'),10.3),{})
        self.assertEqual(ingest(dict(sample,pose=[0.]*16),10.3),{})
        self.assertEqual(ingest(dict(sample,seq=0),10.3),{})
        self.assertEqual(ingest(dict(sample,seq=2.,frame_time_ms=99.),10.3),{})
        self.assertEqual(ingest(dict(sample,controllers=dict(controllers,left=[0.]*16)),10.3),{})
        conflicting=dict(sample,pose=np.diag([-1.,-1.,1.,1.]).ravel(order='F').tolist())
        self.assertEqual(head.ingest(dict(version=1,samples=[sample,conflicting]),10.3,controllers),{})
        ingest(dict(sample,seq=2.,frame_id='1:2:1',tracking_id='1:1'),10.4)
        self.assertTrue(head.changed)

    def test_ros_clock_conversion_preserves_head_sample_time_across_new_hand_frames(self):
        import ast
        from types import SimpleNamespace as S
        path=ROOT/'src/integration/bindu_runtime/bindu_runtime/vr_input_node.py'
        definition=next(n for n in ast.parse(path.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='VRInputNode')
        method=next(n for n in definition.body if getattr(n,'name','')=='poll_input')
        clock=S(wall=1000.01);published=[];packets=[]
        namespace=dict(time=S(time=lambda:clock.wall,monotonic=lambda:1.),stamp=lambda t:t,
                       VRInput=lambda **kw:S(**kw))
        exec(compile(ast.Module(body=[method],type_ignores=[]),str(path),'exec'),namespace)
        owner=S(receiver=S(poll=lambda:packets.pop(0),poll_health=lambda:[]),last_frame=None,input_clock_offset=None,
                last_poll=None,poll_delay_max=0.,
                now=lambda:50.,runtime_mode='simulation',publisher=S(publish=published.append))
        first=self.frame();first=replace(first,stamp=1000.,head_stamp=999.98)
        packets.append((first,replace(first,side='right')));namespace['poll_input'](owner)
        clock.wall+=.02
        second=replace(first,seq=2,stamp=1000.02)
        packets.append((second,replace(second,side='right')));namespace['poll_input'](owner)
        self.assertAlmostEqual(published[0].head_stamp,49.97)
        self.assertEqual(published[0].head_stamp,published[2].head_stamp)
        self.assertEqual(published[0].head_seq,published[2].head_seq)
        self.assertEqual(published[2].head_pose,list(first.head_pose))


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
        lines=status_lines(dict(pico=True,mode='loading',reason='WAIT',load_state='held',teleop_mode='bimanual_tool',
                                external_support=True,gripper_modes=['closed','open']))
        self.assertEqual(len(lines),3)
        text=' '.join(line for line,_ in lines)
        self.assertIn('扫雪耦合',text);self.assertIn('装卸',text);self.assertIn('既有流程',text)
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
        cfg=json.loads((package_share("snowbot_bringup")/'config/robots/snowbot/bimanual_snowbot_shovel.json').read_text())
        session=BimanualSession(cfg,1,0.)
        rig=Rig('held');a,b=rig.core.frames
        session.ingest(a,b,rig.now)
        rig.triggers[1]=1.;rig.grips=[1.,1.];rig.step()
        a,b=rig.core.frames;session.ingest(a,b,rig.now)
        self.assertEqual(session.grasp_mode,'rigid')
        session.ingest(replace(a,seq=a.seq+1,stamp=a.stamp+.01),
                       replace(b,seq=b.seq+1,stamp=b.stamp+.01,virtual_grip=True),rig.now+.01)
        self.assertEqual(session.grasp_mode,'right_axial')



class ShortRodFixtureTests(unittest.TestCase):
    def test_midpoint_rebase_removes_shovel_lever_without_changing_grasp(self):
        from bindu_teleoperation.simulation_fixture import short_rod_config
        import copy
        path=package_share("snowbot_bringup")/'config/robots/snowbot/bimanual_snowbot_shovel.json'
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
        root=ROOT
        cfg=package_share('snowbot_bringup')/'config/robots/snowbot/bimanual_snowbot_shovel.json'
        urdf=package_share('snowbot_description')/'urdf/snowbot_display.urdf'
        rod=short_rod_config(json.loads(cfg.read_text()))
        with tempfile.TemporaryDirectory() as directory:
            prepare_model(urdf,directory,cfg,short_rod=True)
            tree=ET.parse(Path(directory)/'snowbot-tool.urdf')
            visuals=tree.findall("link[@name='bindu_virtual_tool']/visual")
            self.assertEqual([v.get('name') for v in visuals],['shaft'])
            self.assertAlmostEqual(float(visuals[0].find('geometry/cylinder').get('length')),.56)
            origin=tree.find("joint[@name='bindu_virtual_tool_mount']/origin")
            np.testing.assert_allclose(np.fromstring(origin.get('xyz'),sep=' '),np.array(rod['left_to_tool']).reshape(4,4)[:3,3])



if __name__ == "__main__":
    unittest.main()
