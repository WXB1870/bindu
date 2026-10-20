"""Cross-package contract tests; shared contracts and control do not depend on implementations."""
from tests.support.paths import ROOT
import ast
import asyncio
import xml.etree.ElementTree as ET
import json
from pathlib import Path
import unittest
import threading
from types import SimpleNamespace as S
from unittest.mock import Mock
from bindu_contracts.profile import Profile
from bindu_contracts.contracts import Motion
from bindu_execution.executor import Executor
from bindu_hardware.drivers.simulated_joints import SimJointDriver
from bindu_hardware.drivers.simulated_base import SimBaseDriver
from bindu_hardware.robot_io.system import RobotIO
from bindu_planning.simulated import PlannerStrategy
from bindu_vla.simulated import ChunkStrategy
from bindu_runtime.task_node import TaskNode
from rclpy.action import GoalResponse


class TaskReadiness(unittest.TestCase):
    def setUp(self):
        ready = lambda:S(service_is_ready=lambda:True)
        self.node = S(state=S(profile_hash='fixture',feedback_stamp=S(sec=10,nanosec=0)),
            health=S(ready=True,stamp=S(sec=10,nanosec=0)),
            profile=S(digest='fixture',capabilities=('base_velocity',)), now=lambda:10.1,
            lease_client=ready(),submit_client=ready(),control_client=ready(),locate_client=ready(),
            navigation=S(config=S(sites={'pickup':None})),busy=threading.Lock(),event=Mock())
        self.node.readiness_reason = lambda navigation_only=False:TaskNode.readiness_reason(self.node,navigation_only)

    def test_discovered_action_cannot_hide_missing_executor_service(self):
        self.node.submit_client.service_is_ready=lambda:False
        self.assertEqual(self.node.readiness_reason(),'EXECUTION_SERVICES_UNAVAILABLE')
        self.assertEqual(TaskNode.goal_site(self.node,S(task_id='test',site='pickup')),GoalResponse.REJECT)
        self.node.event.assert_called_once_with('REJECTED','EXECUTION_SERVICES_UNAVAILABLE','test')
        self.assertFalse(self.node.busy.locked())

    def test_cached_future_or_stale_feedback_is_not_ready(self):
        for sec,ns in ((9,0),(10,300000000)):
            with self.subTest(stamp=(sec,ns)):
                self.node.state.feedback_stamp=S(sec=sec,nanosec=ns)
                self.assertEqual(self.node.readiness_reason(),'FEEDBACK_STALE')
        self.node.state=None
        self.assertEqual(self.node.readiness_reason(),'EXECUTION_STATE_UNAVAILABLE')

    def test_navigation_does_not_wait_for_unrelated_perception(self):
        self.node.locate_client.service_is_ready=lambda:False
        self.assertEqual(self.node.readiness_reason(),'PERCEPTION_SERVICE_UNAVAILABLE')
        self.assertEqual(self.node.readiness_reason(navigation_only=True),'')
        self.node.health.ready=False
        self.assertEqual(self.node.readiness_reason(navigation_only=True),'RECORDER_UNAVAILABLE')

    def test_readiness_query_does_not_acquire_authority(self):
        reply=TaskNode.ready(self.node,None,S())
        self.assertTrue(reply.success)
        self.assertEqual(reply.message,'READY')
        self.assertFalse(self.node.busy.locked())
        self.node.event.assert_not_called()

    def test_busy_goal_reports_reason_and_preserves_owner(self):
        self.node.busy.acquire()
        self.assertEqual(TaskNode.goal_site(self.node,S(task_id='test',site='pickup')),GoalResponse.REJECT)
        self.node.event.assert_called_once_with('REJECTED','TASK_BUSY','test')
        self.assertTrue(self.node.busy.locked())


class CapabilityContract(unittest.TestCase):
    def test_strategies_share_executor_across_profiles(self):
        root = ROOT
        for filename in ('src/integration/bindu_runtime/config/simulation/wheel_sim.json',
                         'tests/config/profiles/alternate_sim.json'):
            profile = Profile.load(root/filename)
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
        source = ROOT / 'src'
        allowed = {
            'bindu_contracts': set(), 'bindu_interfaces': set(),
            'bindu_tasks': {'bindu_contracts'},
            'bindu_execution': {'bindu_contracts'},
            'bindu_hardware': {'bindu_contracts'},
            'bindu_recording': {'bindu_contracts'},
            'bindu_planning': {'bindu_contracts'},
            'bindu_vla': {'bindu_contracts'},
            # The separate native Nav2 process consumes shared wire types;
            # the Python navigation core remains ROS-independent.
            'bindu_navigation': {'bindu_contracts', 'bindu_interfaces'},
            'bindu_perception': {'bindu_contracts'},
            'bindu_teleoperation': {'bindu_contracts'},
            'bindu_kinematics': {'bindu_contracts', 'bindu_bimanual_qp'},
        }
        # This declared sensor transport is a ROS adapter, not navigation core.
        # Keep the exception at a specific path; all core files stay ROS-free.
        ros_adapters = {'bindu_navigation': {'bindu_navigation/airy_scan.py'}}
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
                    relative = file.relative_to(manifest.parent).as_posix()
                    if relative in ros_adapters.get(name, set()):
                        self.assertTrue(any(e.text == 'rclpy' and 'depend' in e.tag for e in xml), str(file))
                    else:
                        self.assertNotIn('rclpy', imports, str(file))


if __name__ == '__main__': unittest.main()
