"""Collision constraints and independent rejection, using a small analytic rig."""
import copy
import tempfile
import unittest
from pathlib import Path
import numpy as np
from bindu_contracts.teleoperation import IKRequest


class IKCollisionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            import pinocchio
            import casadi
        except ImportError:
            raise unittest.SkipTest('Pinocchio/CasADi required')
        from bindu_kinematics.pinocchio_casadi import PinocchioCasadiIK
        cls.temp = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.temp.cleanup)
        urdf = Path(cls.temp.name)/'rig.urdf'
        urdf.write_text('''<robot name="analytic"><link name="root"/><link name="tip"/><link name="body"/>
          <joint name="slide" type="prismatic"><parent link="root"/><child link="tip"/>
          <axis xyz="1 0 0"/><limit lower="-2" upper="2" effort="1" velocity="1"/></joint>
          <joint name="context" type="prismatic"><parent link="root"/><child link="body"/>
          <axis xyz="1 0 0"/><limit lower="-2" upper="2" effort="1" velocity="1"/></joint></robot>''')
        cls.config = {'urdf':str(urdf), 'joint_names':['slide'], 'ee_link':'tip',
            'tool_transform':np.eye(4).ravel().tolist(), 'locked_joints':{'context':1.},
            'measured_context':True, 'smooth_weight':.01, 'solve_timeout':.5,
            'position_tolerance':.15, 'rotation_tolerance':.1, 'max_joint_step':2.1,
            'collision':{'margin':.02, 'joint_sample_step':.025,
                'shapes':{'tip':{'type':'sphere','link':'tip','center':[0,0,0],'radius':.1},
                          'body':{'type':'sphere','link':'body','center':[0,0,0],'radius':.1}},
                'pairs':[['tip','body']]}}
        cls.solver = PinocchioCasadiIK(cls.config)

    def request(self, seed, target=None, context=0.):
        pose = () if target is None else tuple(self.solver.arm.fk([target],[context]).ravel())
        return IKRequest('collision',1,10.,10.5,('slide',),(seed,),pose,(context,))

    def test_hard_constraint_changes_otherwise_colliding_solution(self):
        from bindu_kinematics.pinocchio_casadi import PinocchioCasadiIK
        cfg=copy.deepcopy(self.config);cfg.pop('collision')
        unguarded=PinocchioCasadiIK(cfg).solve(self.request(-.5,-.15))
        self.assertTrue(unguarded.success,unguarded)
        self.assertFalse(self.solver.collision.safe(unguarded.positions,[0.]))
        guarded=self.solver.solve(self.request(-.5,-.15))
        self.assertTrue(guarded.success,guarded)
        self.assertLessEqual(guarded.positions[0],-.22)

    def test_safe_endpoints_with_colliding_middle_are_rejected(self):
        from unittest.mock import patch
        self.assertTrue(self.solver.collision.safe([-1.],[0.]))
        self.assertTrue(self.solver.collision.safe([1.],[0.]))
        # Force a different IK basin: the endpoint is feasible, the segment is not.
        with patch.object(self.solver,'_solve',return_value=np.array([1.])):
            result=self.solver.solve(self.request(-1.,1.))
        self.assertEqual(result.code,'IK_COLLISION_PATH',result)
        self.assertFalse(result.positions)

    def test_measured_opposite_body_and_no_target_are_checked(self):
        self.assertTrue(self.solver.solve(self.request(-.5)).success)
        result=self.solver.solve(self.request(-.5,context=-.5))
        self.assertEqual(result.code,'IK_COLLISION_SEED')
        self.assertFalse(result.positions)

    def test_independent_numeric_guard_rejects_bad_solver_output(self):
        from unittest.mock import patch
        with patch.object(self.solver,'_solve',return_value=np.array([-.15])):
            result=self.solver.solve(self.request(-.5,-.15))
        self.assertEqual(result.code,'IK_COLLISION_PATH')
        self.assertFalse(result.positions)

    def test_invalid_geometry_fails_closed(self):
        from bindu_kinematics.collision import CollisionModel
        variants=[]
        for key,value in [('radius',float('nan')),('radius',-.1),('link','missing'),('center',[0,0])]:
            cfg=copy.deepcopy(self.config['collision']);cfg['shapes']['tip'][key]=value;variants.append(cfg)
        for key,value in [('margin',-1),('joint_sample_step',float('nan')),('pairs',[]),('pairs',[['tip','body'],['body','tip']])]:
            cfg=copy.deepcopy(self.config['collision']);cfg[key]=value;variants.append(cfg)
        for cfg in variants:
            with self.subTest(cfg=cfg),self.assertRaisesRegex(ValueError,'IK_INVALID_COLLISION_CONFIG'):
                CollisionModel(self.solver.arm,cfg)

    def test_oriented_box_and_sphere_distance(self):
        from bindu_kinematics.collision import CollisionModel
        cfg=copy.deepcopy(self.config['collision'])
        cfg['shapes']['body']={'type':'box','link':'body','center':[0,0,0],'half_extents':[.1,.2,.3]}
        model=CollisionModel(self.solver.arm,cfg)
        self.assertAlmostEqual(next(iter(model.clearances([.5],[0.]).values())),.3)
        self.assertLess(next(iter(model.clearances([0.],[0.]).values())),0)
        # Rotate the box 90 degrees: world x now has its original y extent.
        original=self.solver.arm.frames
        def rotated(q,c,links):
            frames=original(q,c,links);frames['body'][:3,:3]=[[0,-1,0],[1,0,0],[0,0,1]];return frames
        from unittest.mock import patch
        with patch.object(self.solver.arm,'frames',side_effect=rotated):
            self.assertAlmostEqual(next(iter(model.clearances([.5],[0.]).values())),.2)


class G1CollisionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            import pinocchio
            import casadi
        except ImportError:
            raise unittest.SkipTest('Pinocchio/CasADi required')
        from bindu_contracts.profile import Profile
        from bindu_teleoperation.config import load_config
        from bindu_kinematics.pinocchio_casadi import PinocchioCasadiIK
        root=Path(__file__).resolve().parents[1]
        folder=root/'src/integration/bindu_runtime/config'
        profile=Profile.load(folder/'g1_provisional_sim.json')
        cls.solvers={side:PinocchioCasadiIK(load_config(folder/f'teleop_g1_{side}.json',profile,
            root/'src/hardware/bindu_description/urdf')['kinematics']) for side in ('left','right')}

    def test_symbolic_collision_agrees_with_independent_fk_for_both_arms(self):
        import casadi as ca
        rng=np.random.default_rng(23)
        for side,solver in self.solvers.items():
            arm=solver.arm;model=solver.collision
            self.assertEqual(len(model.pairs),32)
            q=ca.SX.sym('q',7);c=ca.SX.sym('c',len(arm.context_names))
            check=ca.Function('check_'+side,[q,c],[model.symbolic(q,c)])
            for i in range(80):
                values=rng.uniform(arm.lower,arm.upper)
                context=rng.uniform(arm.full.lowerPositionLimit[arm.context_indices],arm.full.upperPositionLimit[arm.context_indices])
                numeric=np.array(list(model.clearances(values,context).values()))
                symbolic=np.asarray(check(values,context)).ravel()
                np.testing.assert_array_equal(numeric>=model.margin,symbolic>=0)

    def test_colliding_g1_torso_poses_rejected_without_commands(self):
        poses={'left':[-2.0416677715,.3619414998,-2.6606169888,-2.4096499368,.0868605503,-.0962863657,1.2833772552],
               'right':[-1.372829137,1.4538157683,-.3239110404,2.4809886045,.0905583892,.0766113893,1.2199194703]}
        for side,solver in self.solvers.items():
            req=IKRequest(side,1,10.,10.2,solver.arm.names,tuple(poses[side]),context=solver.arm.context_default)
            result=solver.solve(req)
            self.assertEqual(result.code,'IK_COLLISION_SEED')
            self.assertFalse(result.positions)

    def test_measured_opposite_link7_collision_is_rejected(self):
        solver=self.solvers['left']
        q=(2.4803724595,1.5487593744,.8725818179,1.6338667652,-1.3640071698,.0661232515,-.3207135739)
        context=(-1.4410515695,.4031352377,.7814389628,.6146689157,.5475669094,-1.9782159672,
                 -.2130727599,2.1278334169,2.408514338,-1.4959198753,-.2191208669,-.4324825628)
        self.assertLess(solver.collision.clearances(q,context)['left_3 / right_1'],0)
        result=solver.solve(IKRequest('arms',1,10.,10.2,solver.arm.names,q,context=context))
        self.assertEqual(result.code,'IK_COLLISION_SEED')
        self.assertFalse(result.positions)


if __name__ == '__main__':
    unittest.main()
