import json
import math
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path
from unittest.mock import patch
from bindu_navigation.navigation import NavigationConfig, NavigationGate, planar_pose
from bindu_contracts.contracts import Motion
from bindu_contracts.profile import Profile
from bindu_execution.executor import Executor, Rejected
from bindu_hardware.robot_io.system import RobotIO
from bindu_hardware.drivers.simulated_base import SimBaseDriver
from bindu_hardware.drivers.simulated_joints import SimJointDriver

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT/'src/integration/bindu_runtime/config/navigation_sim.json'


class NavigationTests(unittest.TestCase):
    def setUp(self):
        self.cfg = NavigationConfig.load(CONFIG)
        self.g = NavigationGate(self.cfg, 'goal', 10.)
        self.g.accept_pose(self.cfg.map_id, 'map', 10., (.1, 0., 0.), 10.)

    def velocity(self, **kw):
        values = dict(goal_id='goal', source_id='instance', sequence=1, stamp=10., frame='base_link', velocity=(.1, .2), now=10.)
        values.update(kw)
        return self.g.accept_velocity(**values)

    def test_duplicate_sites_and_nonfinite_config_rejected(self):
        raw = json.loads(CONFIG.read_text())
        with tempfile.TemporaryDirectory() as td:
            p = Path(td)/'config.json'
            for change in ('duplicate', 'nan', 'negative_timeout'):
                d = json.loads(json.dumps(raw))
                if change == 'duplicate': d['sites'].append(d['sites'][0])
                elif change == 'nan': d['sites'][0]['x'] = float('nan')
                else: d['timeout'] = -1
                p.write_text(json.dumps(d))
                with self.subTest(change=change), self.assertRaises(ValueError): NavigationConfig.load(p)

    def test_other_goal_and_duplicate_do_not_refresh_watchdog(self):
        self.assertTrue(self.velocity())
        self.assertFalse(self.velocity(goal_id='old', sequence=2, stamp=10.2, now=10.2))
        self.assertFalse(self.velocity(stamp=10.2, now=10.2))
        self.g.accept_pose(self.cfg.map_id, 'map', 10.4, (0., 0., 0.), 10.4)
        with self.assertRaisesRegex(RuntimeError, 'NAV_INPUT_TIMEOUT'): self.g.check(10.4)

    def test_new_publisher_under_same_goal_fails_closed(self):
        self.velocity()
        self.assertFalse(self.velocity(source_id='restarted', sequence=2))
        self.assertTrue(self.g.closed)
        self.assertEqual(self.g.fault, 'NAV_SOURCE_CHANGED')

    def test_stale_future_wrong_frame_and_nonfinite_commands(self):
        for kw, code in [({'stamp':9.9},'NAV_STALE_VELOCITY'), ({'stamp':10.1},'NAV_STALE_VELOCITY'),
                         ({'frame':'map'},'NAV_INVALID_VELOCITY'), ({'velocity':(float('nan'),0.)},'NAV_INVALID_VELOCITY')]:
            self.setUp()
            with self.subTest(kw=kw):
                self.assertFalse(self.velocity(**kw)); self.assertEqual(self.g.fault,code)

    def test_close_fences_even_current_goal(self):
        self.g.close()
        self.assertFalse(self.velocity())
        self.assertIsNone(self.g.velocity)

    def test_global_pose_and_clock_are_required(self):
        self.velocity()
        with self.assertRaisesRegex(RuntimeError, 'NAV_POSE_STALE'): self.g.check(10.31)
        self.setUp()
        self.g.accept_pose('other_map','map',10.,(0.,0.,0.),10.)
        with self.assertRaisesRegex(RuntimeError, 'NAV_INVALID_POSE'): self.g.check(10.,False)
        self.setUp()
        with self.assertRaisesRegex(RuntimeError, 'NAV_CLOCK_RESET'): self.g.check(9.,False)

    def test_goal_tolerance_and_wrapped_yaw(self):
        site = replace(self.cfg.sites['pickup'], yaw=math.pi-.01)
        self.g.accept_pose(self.cfg.map_id,'map',10.01,(.1,0.,-math.pi+.01),10.01)
        self.assertTrue(self.g.at_site(site))
        self.g.accept_pose(self.cfg.map_id,'map',10.02,(.1,.5,site.yaw),10.02)
        self.assertFalse(self.g.at_site(site))
        with self.assertRaisesRegex(ValueError, 'QUATERNION'): planar_pose((0.,0.,0.),(0.,0.,0.,0.))

    def test_stop_needs_post_request_fresh_distinct_feedback(self):
        self.assertFalse(self.g.stopped(9.99,(0.,0.),10.,10.))
        self.assertFalse(self.g.stopped(10.,(0.,0.),10.,10.))
        self.assertFalse(self.g.stopped(10.,(0.,0.),10.2,10.))
        self.assertTrue(self.g.stopped(10.2,(0.,0.),10.2,10.))
        self.assertFalse(self.g.stopped(10.21,(.1,0.),10.21,10.))
        self.assertFalse(self.g.stopped(10.22,(0.,0.),10.22,10.))
        self.assertFalse(self.g.stopped(10.6,(0.,0.),10.6,10.))


class BaseStreamTests(unittest.TestCase):
    def setUp(self):
        self.p = Profile.load(ROOT/'src/integration/bindu_runtime/config/wheel_sim.json')
        self.base = SimBaseDriver()
        drivers = {g:SimJointDriver(names,self.p.max_speed) for g,names in self.p.groups.items()}
        self.io = RobotIO(self.p,drivers,self.base)
        self.e = Executor(self.p,self.io)
        lease, epoch = self.e.acquire('nav',['base'],2.,10.)
        self.m = Motion('first',lease,epoch,self.p.digest,'task','goal','base_velocity','base',10.,.4,velocity=(.2,.3),duration=.3)

    def test_continuous_updates_slew_and_expire_without_stop_gaps(self):
        with patch.object(self.io,'stop',wraps=self.io.stop) as stop:
            previous=(0.,0.)
            for i in range(1,81):
                now=10+i*.01
                if i%5==1:
                    self.e.submit(replace(self.m,command_id=str(i),stamp=now,expected_revision=self.e.revision),now)
                self.e.tick(now)
                current=self.base.velocity
                for a,b,axis in zip(previous,current,('linear','angular')):
                    self.assertLessEqual(abs(b-a),self.p.base_acceleration(axis)*.01+1e-9)
                if i>25: self.assertGreater(current[0], .19)
                previous=current
            stop.assert_not_called()
        for i in range(81,140): self.e.tick(10+i*.01)
        self.assertEqual(self.base.velocity,(0.,0.))
        self.assertIsNone(self.e.active)

    def test_revision_stop_and_new_lease_fence_late_commands(self):
        stale=replace(self.m,expected_revision=self.e.revision)
        self.e.halt(10.)
        with self.assertRaisesRegex(Rejected,'STALE_REVISION'): self.e.submit(stale,10.)
        self.e.halt(10.,revoke=True)
        self.e.acquire('new',['base'],2.,10.)
        with self.assertRaisesRegex(Rejected,'INVALID_LEASE'): self.e.submit(self.m,10.)

    def test_invalid_base_profile_limits(self):
        raw=json.loads((ROOT/'src/integration/bindu_runtime/config/wheel_sim.json').read_text())
        with tempfile.TemporaryDirectory() as td:
            path=Path(td)/'profile.json'
            for value in (-1.,float('nan'),True):
                raw['execution']['base_limits']['linear_acceleration']=value
                path.write_text(json.dumps(raw))
                with self.subTest(value=value), self.assertRaisesRegex(ValueError,'base limits'):
                    Profile.load(path)

    def test_axis_specific_limits_and_resource_ownership(self):
        self.e.profile=replace(self.p,base_limits={'linear_velocity':.1,'angular_velocity':.5})
        with self.assertRaisesRegex(Rejected,'BASE_LIMIT'): self.e.submit(self.m,10.)
        self.e.submit(replace(self.m,velocity=(.08,.45)),10.)
        self.e.halt(10.,revoke=True)
        lease,epoch=self.e.acquire('arm',['arm'],2.,10.)
        with self.assertRaisesRegex(Rejected,'RESOURCE_NOT_OWNED'):
            self.e.submit(replace(self.m,command_id='other',lease_id=lease,epoch=epoch),10.)

    def test_differential_arc_and_reverse(self):
        self.base.write_velocity((.2, .4))
        for i in range(100): reading = self.base.read(10+i*.01, .01)
        self.assertAlmostEqual(reading.x, .5*math.sin(.4))
        self.assertAlmostEqual(reading.y, .5*(1-math.cos(.4)))
        self.assertAlmostEqual(reading.yaw, .4)
        self.base.write_velocity((-.2, -.4))
        for i in range(100): reading = self.base.read(11+i*.01, .01)
        self.assertAlmostEqual(reading.x, 0.)
        self.assertAlmostEqual(reading.y, 0.)
        self.assertAlmostEqual(reading.yaw, 0.)

    def test_rotation_then_forward_uses_body_heading(self):
        self.base.write_velocity((0., math.pi/2))
        for i in range(100): reading = self.base.read(10+i*.01, .01)
        self.assertAlmostEqual(reading.x, 0.)
        self.assertAlmostEqual(reading.y, 0.)
        self.base.write_velocity((.2, 0.))
        for i in range(100): reading = self.base.read(11+i*.01, .01)
        self.assertAlmostEqual(reading.x, 0.)
        self.assertAlmostEqual(reading.y, .2)
        self.base.request_stop()
        stopped = self.base.read(12., .01)
        self.assertEqual((stopped.x, stopped.y), (reading.x, reading.y))

    def test_planar_feedback_propagates_and_frozen_stamp_stays_frozen(self):
        self.base.yaw = math.pi/2
        self.base.write_velocity((.2, 0.))
        reading = self.io.read_feedback(10., .01)
        self.assertAlmostEqual(reading.base_y, .002)
        self.base.inject_fault('feedback_loss')
        frozen = self.io.read_feedback(10.01, .01)
        self.assertEqual((frozen.stamp, frozen.base_y), (reading.stamp, reading.base_y))
        self.base.inject_fault('')
        self.base.y = float('nan')
        with self.assertRaisesRegex(RuntimeError, 'INVALID_DEVICE_FEEDBACK'):
            self.io.read_feedback(10.02, .01)


if __name__ == '__main__': unittest.main()
