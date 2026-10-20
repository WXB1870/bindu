"""Independent clutches and native dual-target movement from measured zero."""
from tests.support.paths import ROOT
import json
import unittest
from pathlib import Path
from dataclasses import replace
from unittest.mock import patch
import numpy as np
from ament_index_python.packages import get_package_share_directory
from bindu_contracts.profile import Profile
from bindu_contracts.teleoperation import VRFrame
from bindu_kinematics.bimanual import BimanualQP
from bindu_kinematics.tool_geometry import make_collision_checker
from bindu_teleoperation.independent_session import IndependentArmsSession

class IndependentSessionTests(unittest.TestCase):
    def setUp(self):
        self.session=IndependentArmsSession(dict(input_max_age=.25,feedback_max_age=.2,
            command_max_age=.4,engage_seconds=.1,position_scale=1.),2,0.)
        self.seq=0;self.now=1.;self.grips=[0.,0.];self.x=[0.,.3];self.clutch=[0,0]
        self.poses=np.array([np.eye(4),np.eye(4)]);self.poses[1,1,3]=.4
        self.step()

    def step(self,dt=.02):
        self.now+=dt;self.seq+=1;frames=[]
        for i,side in enumerate(('left','right')):
            p=np.eye(4);p[0,3]=self.x[i]
            frames.append(VRFrame('vr',self.seq,self.now,side,tuple(p.ravel()),self.grips[i],
                                  clutch_seq=self.clutch[i],mapping='pico_snowbot_v1'))
        self.session.ingest(*frames,self.now)

    def engage(self,grips):
        self.grips=grips;self.step();self.step(.12);self.session.capture(self.poses)

    def test_single_hand_engages_without_tool_geometry_or_opposite_clutch(self):
        self.engage([1.,0.]);np.testing.assert_allclose(self.session.target,self.poses)
        self.x[0]+=.03;self.step()
        self.assertGreater(np.linalg.norm(self.session.target[0]-self.poses[0]),.02)
        np.testing.assert_array_equal(self.session.target[1],self.poses[1])

    def test_paused_side_recaptures_while_other_side_keeps_its_baseline(self):
        self.engage([1.,1.]);right_anchor=self.session.anchors[1].copy()
        self.grips=[0.,1.];self.step();self.session.capture(self.poses)
        self.x[0]+=.15;self.x[1]+=.01;self.step()
        np.testing.assert_array_equal(self.session.target[0],self.poses[0])
        np.testing.assert_array_equal(self.session.anchors[1],right_anchor)
        self.grips=[1.,1.];self.step();self.step(.12);self.session.capture(self.poses)
        np.testing.assert_allclose(self.session.target[0],self.poses[0])
        self.assertGreater(np.linalg.norm(self.session.target[1]-self.poses[1]),.005)

    def test_coalesced_release_fences_pending_target_before_reengagement(self):
        self.engage([1.,1.]);generation=self.session.grasp_generation
        self.clutch[0]+=1;self.step()
        self.assertGreater(self.session.grasp_generation,generation)
        self.assertIsNone(self.session.anchor_tool)
        self.assertEqual(self.session.active_mask,2)

    def test_stale_tracking_and_source_change_reject(self):
        self.engage([1.,0.])
        f=self.session.hands[0].frame;right=self.session.hands[1].frame
        with self.assertRaisesRegex(ValueError,'PAIR_INVALID'):
            self.session.ingest(replace(f,seq=f.seq+1),replace(right,seq=f.seq+1),self.now+.3)
        with self.assertRaisesRegex(ValueError,'CONNECTION_CHANGED'):
            self.session.ingest(replace(f,seq=f.seq+1,source_id='new'),
                replace(right,seq=f.seq+1,source_id='new'),self.now)

    def test_invalid_or_stop_duplicate_is_never_ignored(self):
        self.engage([1.,0.]);left,right=(h.frame for h in self.session.hands)
        for change in (dict(valid=False),dict(stop=True),dict(init=True)):
            with self.subTest(change=change),self.assertRaisesRegex(ValueError,'PAIR_INVALID'):
                self.session.ingest(left,replace(right,**change),self.now)


class IndependentNativeTests(unittest.TestCase):
    def setUp(self):
        base=ROOT/'src/integration/bindu_runtime/config/robots/snowbot'
        self.cfg=json.loads((base/'bimanual_snowbot_real.json').read_text())
        self.cfg['urdf']=str(Path(get_package_share_directory('bindu_description'))/'urdf'/self.cfg['urdf'])
        self.solver=BimanualQP(self.cfg,Profile.load(base/'snowbot_real_position.json'))
        self.addCleanup(self.solver.close)
        recipe=dict(self.cfg['demo'])
        recipe['left_grip_in_tool_m']=(np.linalg.inv(self.solver.tool) @
                                     np.r_[recipe['grip_offset_m'],1.])[:3].tolist()
        self.solver.empty_collision_check=make_collision_checker(
            Path(self.cfg['urdf']).parent/recipe['display_urdf'],recipe,
            conservative_convex=True,tool_present=False)
        self.feedback={n:0. for n in self.solver.model_names}

    def test_recorded_collision_candidates_shrink_without_ending_control(self):
        data=json.loads((ROOT/'tests/config/teleoperation/snowbot_candidate_steps.json').read_text())
        cfg=dict(self.cfg,**data['recorded_config'])
        profile=replace(Profile.load(ROOT/'src/integration/bindu_runtime/config/robots/snowbot/snowbot_real_position.json'),
                        max_speed=data['recorded_joint_speed'],joint_dynamics={})
        solver=BimanualQP(cfg,profile);self.addCleanup(solver.close)
        solver.empty_collision_check=self.solver.empty_collision_check
        strict=BimanualQP(dict(cfg,geometry_step_backtracking=False),profile)
        self.addCleanup(strict.close);strict.empty_collision_check=self.solver.empty_collision_check
        for case in data['independent_cases']:
            with self.subTest(code=case['recorded_code']):
                target=np.array(case['target']).reshape(2,4,4)
                reference=np.array(case['reference'])
                strict.capture_independent(case['capture'])
                raw=[];check=strict.check_independent_path
                def record(feedback,*path):
                    raw.append(np.array(path[0]).copy())
                    return check(feedback,*path)
                with patch.object(strict,'check_independent_path',side_effect=record):
                    with self.assertRaisesRegex(ValueError,'SELF_COLLISION'):
                        strict.solve_independent(case['feedback'],target,active_mask=case['active_mask'],
                            reference=reference,dt=case['dt'])
                self.assertFalse(strict.captured)
                self.assertEqual(len(raw),1)

                solver.capture_independent(case['capture'])
                positions,_=solver.solve_independent(case['feedback'],target,active_mask=case['active_mask'],
                    reference=reference,dt=case['dt'])
                step=solver.last_geometry_step;scale=step['scale']
                # Native limit tapering can change the candidate direction.
                # Require the first safe common reduction, not a fixed fraction.
                self.assertGreater(scale,0.)
                self.assertLess(scale,1.)
                self.assertTrue(solver.captured)
                self.assertTrue(step['first_rejection'].startswith(case['recorded_code'].split(' t=')[0]))
                np.testing.assert_allclose(np.array(positions)-reference,scale*(raw[0]-reference),atol=1e-10)
                self.assertGreater(np.max(np.abs(np.array(positions)-reference)),1e-6)
                solver.check_independent_path(case['feedback'],positions,reference)
                larger=reference+2*scale*(raw[0]-reference)
                with self.assertRaisesRegex(ValueError,'SELF_COLLISION'):
                    solver.check_independent_path(case['feedback'],larger,reference)

    def test_zero_can_capture_normally_but_tool_grasp_still_rejects(self):
        with self.assertRaisesRegex(ValueError,'BIMANUAL_GRASP_POSE'):self.solver.capture(self.feedback)
        poses=self.solver.capture_independent(self.feedback)
        self.assertEqual(poses.shape,(2,4,4))
        out,_=self.solver.solve_independent(self.feedback,poses,active_mask=1,
                                          reference=[0.]*len(self.solver.names),dt=.01)
        np.testing.assert_allclose(out,0.,atol=1e-12)

    def test_only_requested_arm_moves_waist_and_other_arm_hold(self):
        for mask,side in ((1,0),(2,1),(3,0)):
            targets=self.solver.capture_independent(self.feedback)
            targets[side,2,3]+=.01
            out,_=self.solver.solve_independent(self.feedback,targets,active_mask=mask,
                                              reference=[0.]*len(self.solver.names),dt=.01)
            nshared=len(self.solver.shared_names);nleft=(len(out)-nshared)//2
            self.assertGreater(np.max(np.abs(out[nshared+side*nleft:nshared+(side+1)*nleft])),1e-7)
            np.testing.assert_allclose(out[:nshared],0.,atol=1e-12)
            if mask!=3:
                np.testing.assert_allclose(out[nshared+(1-side)*nleft:nshared+(2-side)*nleft],0.,atol=1e-12)

    def test_tracking_timestep_collision_and_limits_are_enforced(self):
        poses=self.solver.capture_independent(self.feedback)
        with self.assertRaisesRegex(ValueError,'INTEGRATION_INTERVAL'):
            self.solver.solve_independent(self.feedback,poses,active_mask=1,
                                         reference=[0.]*len(self.solver.names),dt=.11)
        poses=self.solver.capture_independent(self.feedback)
        with self.assertRaisesRegex(ValueError,'TRACKING_ERROR'):
            self.solver.solve_independent(self.feedback,poses,active_mask=1,
                                         reference=[.2]*len(self.solver.names),dt=.01)
        poses=self.solver.capture_independent(self.feedback)
        invalid=dict(self.feedback);invalid[self.solver.names[-1]]=100.
        with self.assertRaisesRegex(ValueError,'hard limits'):
            self.solver.capture_independent(invalid)
        poses=self.solver.capture_independent(self.feedback)
        self.solver.empty_collision_check=lambda _:(_ for _ in ()).throw(ValueError('SELF_COLLISION'))
        with self.assertRaisesRegex(ValueError,'SELF_COLLISION'):
            self.solver.solve_independent(self.feedback,poses,active_mask=1,
                                         reference=[0.]*len(self.solver.names),dt=.01)


if __name__=='__main__':unittest.main()
