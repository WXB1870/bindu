"""Cross-package contract tests; the shared core does not depend on plugins."""
import asyncio
import json
from pathlib import Path
import unittest
from bindu_core.profile import Profile
from bindu_core.contracts import Motion
from bindu_core.execution import Executor
from bindu_core.drivers.simulated_joints import SimJointDriver
from bindu_core.drivers.simulated_base import SimBaseDriver
from bindu_core.robot_io.system import RobotIO
from bindu_planning.simulated import PlannerStrategy
from bindu_vla.simulated import ChunkStrategy


class CapabilityContract(unittest.TestCase):
    def test_strategies_share_executor_across_profiles(self):
        root = Path(__file__).resolve().parents[1]
        for filename in ('wheel_sim', 'alternate_sim'):
            profile = Profile.load(root/'src/bindu_runtime/config'/f'{filename}.json')
            for strategy in (PlannerStrategy(), ChunkStrategy()):
                with self.subTest(profile=filename, strategy=type(strategy).__name__):
                    drivers = {g: SimJointDriver(names, profile.max_speed) for g, names in profile.groups.items()}
                    engine = Executor(profile, RobotIO(profile, drivers, SimBaseDriver()))
                    lease, epoch = engine.acquire('contract', ['arm'], 2., 10.)
                    names = tuple(profile.groups['arm'])
                    plan = asyncio.run(strategy.plan(profile, 'arm', (0.,)*len(names), (.3,)*len(names)))
                    motion = Motion('plan', lease, epoch, profile.digest, 'task', 'obs',
                                    'finite_trajectory', 'arm', 10., 1.5,
                                    names=plan.names, offsets=plan.offsets, points=plan.points)
                    engine.submit(motion, 10.)
                    for i in range(1,101): engine.tick(10.+i*.01)
                    self.assertEqual(engine.results['plan'].state, 'SUCCEEDED')
                    self.assertEqual(tuple(engine.feedback.positions[n] for n in names), (.3,)*len(names))


if __name__ == '__main__': unittest.main()
