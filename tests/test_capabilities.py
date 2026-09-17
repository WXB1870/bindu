"""Cross-package contract tests; shared contracts and control do not depend on implementations."""
import ast
import asyncio
import xml.etree.ElementTree as ET
import json
from pathlib import Path
import unittest
from bindu_contracts.profile import Profile
from bindu_contracts.contracts import Motion
from bindu_execution.executor import Executor
from bindu_hardware.drivers.simulated_joints import SimJointDriver
from bindu_hardware.drivers.simulated_base import SimBaseDriver
from bindu_hardware.robot_io.system import RobotIO
from bindu_planning.simulated import PlannerStrategy
from bindu_vla.simulated import ChunkStrategy


class CapabilityContract(unittest.TestCase):
    def test_strategies_share_executor_across_profiles(self):
        root = Path(__file__).resolve().parents[1]
        for filename in ('wheel_sim', 'alternate_sim'):
            profile = Profile.load(root/'src/integration/bindu_runtime/config'/f'{filename}.json')
            for strategy in (PlannerStrategy(), ChunkStrategy()):
                with self.subTest(profile=filename, strategy=type(strategy).__name__):
                    drivers = {g: SimJointDriver(names, profile.max_speed) for g, names in profile.groups.items()}
                    engine = Executor(profile, RobotIO(profile, drivers, SimBaseDriver()))
                    lease, epoch = engine.acquire('contract', ['arm'], 2., 10.)
                    names = tuple(profile.groups['arm'])
                    plan = asyncio.run(strategy.plan(profile, 'arm', (0.,)*len(names), (.3,)*len(names)))
                    motion = Motion('plan', lease, epoch, profile.digest, 'task', 'obs',
                                    'finite_trajectory', 'arm', 10., 1.5,
                                    names=plan.names, offsets=plan.offsets, points=plan.points,
                                    velocities=plan.velocities, accelerations=plan.accelerations)
                    engine.submit(motion, 10.)
                    for i in range(1,101): engine.tick(10.+i*.01)
                    self.assertEqual(engine.results['plan'].state, 'SUCCEEDED')
                    self.assertEqual(tuple(engine.feedback.positions[n] for n in names), (.3,)*len(names))


class DependencyBoundaries(unittest.TestCase):
    def test_packages_do_not_depend_on_higher_layers(self):
        """Check source imports and manifests so moving files cannot hide coupling."""
        source = Path(__file__).resolve().parents[1] / 'src'
        allowed = {
            'bindu_contracts': set(), 'bindu_interfaces': set(),
            'bindu_tasks': {'bindu_contracts'},
            'bindu_execution': {'bindu_contracts'},
            'bindu_hardware': {'bindu_contracts'},
            'bindu_recording': {'bindu_contracts'},
            'bindu_planning': {'bindu_contracts'},
            'bindu_vla': {'bindu_contracts'},
            'bindu_navigation': {'bindu_contracts'},
            'bindu_perception': {'bindu_contracts'},
            'bindu_teleoperation': {'bindu_contracts'},
            'bindu_kinematics': {'bindu_contracts'},
        }
        for manifest in source.rglob('package.xml'):
            xml = ET.parse(manifest).getroot()
            name = xml.findtext('name')
            declared = {e.text for e in xml if 'depend' in e.tag
                        and e.text and e.text.startswith('bindu_')}
            if name in allowed:
                self.assertLessEqual(declared, allowed[name], name)
            for file in manifest.parent.rglob('*.py'):
                tree = ast.parse(file.read_text())
                imports = set()
                for node in ast.walk(tree):
                    if isinstance(node, ast.Import):
                        imports.update(a.name.split('.')[0] for a in node.names)
                    elif isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
                        imports.add(node.module.split('.')[0])
                project_imports = {i for i in imports if i.startswith('bindu_') and i != name}
                self.assertLessEqual(project_imports, declared, str(file))
                if name in allowed:
                    self.assertLessEqual(project_imports, allowed[name], str(file))
                    self.assertNotIn('rclpy', imports, str(file))


if __name__ == '__main__': unittest.main()
