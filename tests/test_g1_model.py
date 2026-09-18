import unittest
from pathlib import Path
import xml.etree.ElementTree as ET
from dataclasses import replace
from bindu_contracts.profile import Profile
from bindu_contracts.contracts import Motion
from bindu_execution.executor import Executor, Rejected
from bindu_hardware.drivers.simulated_joints import SimJointDriver
from bindu_hardware.drivers.simulated_base import SimBaseDriver
from bindu_hardware.robot_io.system import RobotIO

ROOT = Path(__file__).resolve().parents[1]
URDF = ROOT/'src/hardware/bindu_description/urdf/g1_provisional.urdf'
PROFILE = ROOT/'src/integration/bindu_runtime/config/g1_provisional_sim.json'


class G1ModelTests(unittest.TestCase):
    def test_profile_matches_all_19_urdf_axes_and_limits(self):
        p = Profile.load(PROFILE)
        root = ET.parse(URDF).getroot()
        joints = {j.get('name'): j for j in root.findall('joint') if j.get('type') != 'fixed'}
        self.assertEqual(len(joints), 19)
        self.assertEqual(set(joints), set(p.limits))
        self.assertEqual([len(p.groups[k]) for k in ('leg', 'waist', 'head', 'left_arm', 'right_arm')], [2,1,2,7,7])
        for name, joint in joints.items():
            limit = joint.find('limit')
            self.assertEqual(p.limits[name], [float(limit.get(k)) for k in ('lower', 'upper')])
            self.assertLessEqual(p.speed(name), float(limit.get('velocity')))
        for name in ('leg_joint4', 'leg_joint5'):
            self.assertEqual(root.find(f"joint[@name='{name}']").get('type'), 'fixed')
        self.assertFalse(any('gripper' in n or 'wheel' in n for n in joints))
        self.assertFalse(root.findall('.//mesh'))

    def test_each_group_routes_without_moving_other_axes_and_rejects_limits(self):
        p = Profile.load(PROFILE)
        for group, names in p.groups.items():
            with self.subTest(group=group):
                drivers = {g: SimJointDriver(ns, p.max_speed) for g,ns in p.groups.items()}
                io = RobotIO(p, drivers, SimBaseDriver())
                executor = Executor(p, io)
                lease, epoch = executor.acquire('g1', [group], 2., 10.)
                m = Motion('move', lease, epoch, p.digest, 'test', '', 'finite_trajectory', group,
                           10., 1.5, names=tuple(names), offsets=(1.,), points=((.04,)*len(names),))
                bad = replace(m, points=((p.limits[names[0]][1]+.01,)+(.04,)*(len(names)-1),))
                with self.assertRaises(Rejected): executor.submit(bad, 10.)
                executor.submit(m, 10.)
                for i in range(1, 130): executor.tick(10.+i*.01)
                self.assertEqual(executor.results['move'].state, 'SUCCEEDED')
                for name, value in executor.feedback.positions.items():
                    self.assertAlmostEqual(value, .04 if name in names else 0., places=5)


    def test_g1_pi_map_and_single_arm_scope(self):
        from bindu_vla.pi.adapter import load_config, CommandAdapter, observation_payload, IMAGE_KEYS
        from bindu_vla.pi.protocol import Message
        import numpy as np
        profile = Profile.load(PROFILE)
        cfg = load_config(ROOT/'src/integration/bindu_runtime/config/pi_g1.json', profile)
        self.assertEqual(cfg['joint_map'], {n:n for n in profile.limits})
        self.assertEqual(cfg['resource_groups'], ['left_arm', 'right_arm'])
        self.assertTrue(cfg['require_correlation'])
        positions = {n: .001*i for i,n in enumerate(profile.limits)}
        images = {n: np.zeros((224,224,3), dtype=np.uint8) for n in IMAGE_KEYS}
        payload = observation_payload(cfg, positions, images, 'obs', 'session', 'test')
        self.assertEqual(payload['关节状态字典'], positions)
        for group in ('leg', 'waist', 'head'):
            with self.assertRaisesRegex(ValueError, 'PI_RESOURCE_GROUPS'):
                CommandAdapter(cfg, profile, group, 'session', 10.)
        for group in cfg['resource_groups']:
            adapter = CommandAdapter(cfg, profile, group, 'session', 10.)
            adapter.observe('obs', 49.9)
            message = Message(cfg['command_topic'], 1, 10.1,
                              {'关节命令字典': positions, 'bindu': payload['bindu']}, {})
            names, values, stamp, obs = adapter.convert(message, 10.2, 50.)
            self.assertEqual(names, tuple(profile.groups[group]))
            self.assertEqual(values, tuple(positions[n] for n in names))
            self.assertAlmostEqual(stamp, 49.9)
            self.assertEqual(obs, 'obs')

    def test_g1_session_samples_body_feedback_in_model_order(self):
        import numpy as np
        from bindu_contracts.teleoperation import VRFrame
        from bindu_teleoperation.config import load_config
        from bindu_teleoperation.session import TeleopSession
        profile = Profile.load(PROFILE)
        for side in ('left', 'right'):
            cfg = load_config(ROOT/f'src/integration/bindu_runtime/config/teleop_g1_{side}.json', profile, URDF.parent)
            names = profile.groups[side+'_arm']
            session = TeleopSession(cfg, names)
            frame = VRFrame('vr', 0, 10., side, tuple(np.eye(4).ravel()))
            session.ingest(frame, 10.)
            for i in range(1,13):
                session.ingest(replace(frame, seq=i, stamp=10.+i*.1, grip=1.), 10.+i*.1)
            positions = {n: .001*i for i,n in enumerate(reversed(profile.limits))}
            missing = dict(positions); missing.pop('leg_joint1')
            with self.assertRaises(KeyError): session.request(missing, 11.2, 11.2)
            invalid = dict(positions, leg_joint1=float('nan'))
            with self.assertRaisesRegex(ValueError, 'TELEOP_INVALID_FEEDBACK'):
                session.request(invalid, 11.2, 11.2)
            request = session.request(positions, 11.2, 11.2)
            self.assertEqual(request.seed, tuple(positions[n] for n in names))
            self.assertEqual(request.context, tuple(positions[n] for n in sorted(set(profile.limits)-set(names))))


class G1KinematicsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            import pinocchio
            import casadi
        except ImportError:
            raise unittest.SkipTest('Pinocchio/CasADi required for model FK validation')

    def test_both_arm_fk_with_nonzero_leg_waist_and_opposite_arm(self):
        import numpy as np
        import casadi as ca
        import pinocchio as pin
        from bindu_kinematics.model import ArmModel
        p = Profile.load(PROFILE)
        full = pin.buildModelFromUrdf(str(URDF))
        self.assertEqual(full.nq, 19)
        values = {name: .2*lo+.8*hi for name,(lo,hi) in p.limits.items()}
        for side in ('left', 'right'):
            names = p.groups[side+'_arm']
            cfg = {'urdf': str(URDF), 'joint_names': names,
                   'ee_link': side+'_arm_end_effector_mount_link',
                   'tool_transform': np.eye(4).ravel().tolist(),
                   'locked_joints': {n:v for n,v in values.items() if n not in names}}
            arm = ArmModel(cfg)
            q, matrix = arm.symbolic_fk()
            symbolic = ca.Function('fk_'+side, [q], [matrix])
            q_full = pin.neutral(full)
            for n,v in values.items(): q_full[full.joints[full.getJointId(n)].idx_q] = v
            data = full.createData()
            pin.framesForwardKinematics(full, data, q_full)
            expected = data.oMf[full.getFrameId(cfg['ee_link'])].homogeneous
            joint_values = [values[n] for n in names]
            np.testing.assert_allclose(arm.fk(joint_values), expected, atol=1e-8)
            np.testing.assert_allclose(np.array(symbolic(joint_values)), expected, atol=1e-8)

    def test_g1_ik_requires_measured_body_pose_for_both_arms(self):
        import numpy as np
        from bindu_kinematics.pinocchio_casadi import PinocchioCasadiIK
        from bindu_teleoperation.config import load_config
        from bindu_contracts.teleoperation import IKRequest
        profile = Profile.load(PROFILE)
        for side in ('left', 'right'):
            cfg = load_config(ROOT/f'src/integration/bindu_runtime/config/teleop_g1_{side}.json', profile, URDF.parent)
            solver = PinocchioCasadiIK(cfg['kinematics'])
            body = dict(solver.arm.cfg['locked_joints'])
            body.update(leg_joint1=.1, leg_joint2=.2, leg_joint3=.1)
            context = tuple(body[n] for n in solver.arm.context_names)
            q = np.zeros(7)
            pose = solver.arm.fk(q, context)
            self.assertGreater(np.linalg.norm(pose-solver.arm.fk(q)), .01)
            np.testing.assert_allclose(np.array(solver.context_fk(q,context)),pose,atol=1e-8)
            request = IKRequest('g1',1,10.,10.4,solver.arm.names,tuple(q),context=context)
            actual = solver.solve(request)
            self.assertTrue(actual.success,actual)
            np.testing.assert_allclose(np.array(actual.pose).reshape(4,4),pose,atol=1e-8)
            target_q=q.copy();target_q[0]=.01
            solved=solver.solve(replace(request,target=tuple(solver.arm.fk(target_q,context).ravel())))
            self.assertTrue(solved.success,solved)
            self.assertEqual(solver.solve(replace(request,context=())).code,'IK_INVALID_CONTEXT')
            self.assertEqual(solver.solve(replace(request,context=(float('nan'),)*len(context))).code,'IK_INVALID_CONTEXT')



if __name__ == '__main__': unittest.main()
