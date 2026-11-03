"""Offline primitive replay checks; all I/O is an in-memory port."""
import copy
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace as S
import tempfile
import unittest

from snowbot_bringup.teleop.snowbot_primitive_replay import (
    NAMES, GROUPS, Replay, load_replay, run_replay, kinematic_digest,
)


class FakePort:
    def __init__(self, plan):
        self.plan, self.now, self.sent, self.released = plan, 0., [], False
        self.fail_after = None
        self.late = False
        self.clock_shift = 0.
        self.acquire_failure = False
    def steady(self): return self.now
    def ros(self): return 100.+self.now+self.clock_shift
    def wait_until(self, due):
        self.now = max(self.now, due)
        if self.late: self.now += .1
    def check(self, owned):
        if self.fail_after is not None and len(self.sent) >= self.fail_after:
            raise RuntimeError('STALE_FEEDBACK')
        q = self.plan.sample(self.now)
        return dict(zip(NAMES, q)), dict.fromkeys(NAMES, 0.)
    def acquire(self):
        if self.acquire_failure: raise RuntimeError('ACQUIRE_ACK_TIMEOUT')
    def renew(self): pass
    def send(self, q, source): self.sent.append((self.now, source, tuple(q)))
    def release_and_confirm_stop(self):
        self.released = True
        return True
    def arrival_tolerance(self, name): return .002


class PrimitiveReplayTests(unittest.TestCase):
    def plan(self, speed=1.):
        return Replay((0., .02, .04), ((0.,)*16, (.001,)*16, (.002,)*16),
                      NAMES, 'g099-curved', 'hash', speed)

    def test_completion_waits_for_measured_arrival_and_releases(self):
        p = self.plan()
        port = FakePort(p)
        r = run_replay(p, port)
        self.assertTrue(r['completed'] and r['measured_arrival_confirmed'] and r['stop_confirmed'])
        self.assertTrue(port.released)
        self.assertGreaterEqual(port.now, p.duration+.2)
        self.assertTrue(all(abs(source-(100.+t)) < 1e-8 for t, source, _ in port.sent))

    def test_start_mismatch_sends_nothing(self):
        p = self.plan()
        port = FakePort(p)
        port.now = .04
        with self.assertRaisesRegex(RuntimeError, 'START_POSE'):
            run_replay(p, port, start_tolerance=.001)
        self.assertFalse(port.sent)

    def test_stale_feedback_releases_with_separate_stop_confirmation(self):
        p = self.plan()
        port = FakePort(p)
        port.fail_after = 1
        with self.assertRaises(RuntimeError) as e:
            run_replay(p, port)
        result = json.loads(str(e.exception))
        self.assertFalse(result['completed'])
        self.assertTrue(result['stop_confirmed'] and port.released)
        self.assertEqual(len(port.sent), 1)

    def test_late_tick_and_changed_clock_cannot_refresh_a_target(self):
        for kind in ('late', 'clock'):
            p = self.plan()
            port = FakePort(p)
            if kind == 'late': port.late = True
            else:
                original = port.wait_until
                def shifted(due):
                    original(due)
                    port.clock_shift = .1
                port.wait_until = shifted
            with self.assertRaisesRegex(RuntimeError, 'STREAM_LATE|CLOCK_CHANGED'):
                run_replay(p, port)
            self.assertFalse(port.sent)
            self.assertTrue(port.released)

    def test_partial_acquire_still_cleans_up(self):
        p = self.plan()
        port = FakePort(p)
        port.acquire_failure = True
        with self.assertRaisesRegex(RuntimeError, 'ACQUIRE_ACK_TIMEOUT'):
            run_replay(p, port)
        self.assertTrue(port.released)

    def test_slower_sampling_keeps_the_path(self):
        p = self.plan(.5)
        self.assertAlmostEqual(p.duration, .08)
        self.assertEqual(p.sample(.04), self.plan().sample(.02))
        profile = S(groups=dict(zip(GROUPS, (list(NAMES[:2]), list(NAMES[2:9]), list(NAMES[9:])))),
                    limits=dict.fromkeys(NAMES, (-1., 1.)), speed=lambda n: .1)
        p.validate_profile(profile)
        profile.limits = dict(profile.limits, body_joint_1=(0., .001))
        with self.assertRaisesRegex(ValueError, 'POSITION_LIMIT'):
            p.validate_profile(profile)

    def test_waist_lead_is_real_time_when_slowing_playback(self):
        p = Replay((0., 1.), ((0.,)*16, (.1,)*16), NAMES, 'tool', 'kin', .5, .2)
        self.assertEqual(p.duration, 2.2)
        self.assertAlmostEqual(p.sample(.2)[0], .01)
        self.assertEqual(p.sample(.2)[2], 0.)
        self.assertAlmostEqual(p.sample(.4)[2], .01)
        self.assertEqual(p.sample(p.duration), (.1,)*16)

    def test_vendor_nominal_limits_match_shared_urdf_and_guard_stays_finite(self):
        import re
        import shutil
        import subprocess
        import xml.etree.ElementTree as ET
        import snowbot_bringup.teleop.snowbot_primitive_replay as replay_module
        app = Path(replay_module.__file__).resolve().parents[4]
        patch = (app/'tools/deployment/snowbot_vendor/position_session/takeover.patch').read_text()
        urdf = ET.parse(app/'src/snowbot_description/urdf/snowbot_control.urdf')
        wire_names = [f'arm_{s}_joint_{i}' for s in ('l','r') for i in range(1,8)] + ['body_joint_1','body_joint_2']
        for side in ('lower','upper'):
            actual = list(map(float, re.search(r'nominal_'+side+r'\{([^}]+)\}', patch).group(1).split(',')))
            expected = [float(urdf.find("./joint[@name='"+n+"']/limit").get(side)) for n in wire_names]
            self.assertEqual(actual, expected)
        compiler = shutil.which('c++')
        if not compiler:
            self.skipTest('C++17 compiler required for native guard check')
        a = patch.index('constexpr double joint_limit_allowance')
        b = patch.index('} // namespace position_session', a)
        guard = '\n'.join(line[1:] if line.startswith('+') else line for line in patch[a:b].splitlines())
        source = ('#include <array>\n#include <cmath>\n#include <cassert>\n'
                  '#include <limits>\nnamespace position_session { constexpr unsigned joint_count=16;\n'
                  + guard + '\n}\nint main(){using namespace position_session;'
                  'std::array<double,16> q{};q[2]=.9;q[9]=-1.;q[14]=2.;q[15]=1.;'
                  'assert(in_limits(q));q[2]=4.;assert(!in_limits(q));'
                  'q[2]=std::numeric_limits<double>::quiet_NaN();assert(!in_limits(q));}\n')
        with tempfile.TemporaryDirectory() as directory:
            binary = Path(directory)/'limits'
            subprocess.run([compiler,'-std=c++17','-x','c++','-','-o',str(binary)],
                           input=source,text=True,capture_output=True,check=True)
            subprocess.run([str(binary)],check=True,capture_output=True)

    def test_reference_margin_is_preserved_and_checked_separately(self):
        import snowbot_bringup.teleop.snowbot_primitive_replay as module
        app = Path(module.__file__).resolve().parents[4]
        cfg = json.loads((app/'src/snowbot_bringup/config/robots/snowbot/bimanual_snowbot_real.json').read_text())
        self.assertAlmostEqual(cfg['posture_reference_margin_rad'], .027453292519943294, places=14)
        p = Replay((0., .02), ((.98,)*16, (.98,)*16), NAMES, 'tool', 'kin', 1., reference_margin_rad=.03)
        profile = S(groups=dict(zip(GROUPS, (list(NAMES[:2]), list(NAMES[2:9]), list(NAMES[9:])))),
                    limits=dict.fromkeys(NAMES, (-1., 1.)), speed=lambda n: 1.)
        with self.assertRaisesRegex(ValueError, 'POSITION_LIMIT:arm_'):
            p.validate_profile(profile)

    def test_loader_checks_identity_and_contract_without_ros(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            payload = dict(schema_version=1, mode='joint_target', runtime_mode='real',
                           joint_names=list(NAMES), resource_groups=list(GROUPS),
                           period_s=.02, valid_for_s=.25, start_q_rad=[0.]*16,
                           samples=[dict(t=0., q=[0.]*16), dict(t=.02, q=[.001]*16)])
            target = root/'targets.json'
            target.write_text(json.dumps(payload))
            entry = dict(targets=target.name, sha256=hashlib.sha256(target.read_bytes()).hexdigest())
            manifest = root/'manifest.json'
            manifest.write_text(json.dumps(dict(schema_version=1, validation_passed=True,
                tool_id='g099-curved', kinematic_sha256='kin', actions={'Push': entry})))
            self.assertEqual(load_replay(manifest, 'Push').points[-1][0], .001)
            with self.assertRaisesRegex(ValueError, 'SPEED'):
                load_replay(manifest, 'Push', 2.)
            target.write_text('{}')
            with self.assertRaisesRegex(ValueError, 'IDENTITY'):
                load_replay(manifest, 'Push')

    def test_kinematics_digest_ignores_limits_but_detects_axis_changes(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d)/'r.urdf'
            xml = '<robot><joint name="j" type="revolute"><parent link="b"/><child link="l"/><axis xyz="0 0 1"/><limit lower="-1" upper="1"/></joint></robot>'
            p.write_text(xml)
            digest = kinematic_digest(p)
            p.write_text(xml.replace('upper="1"', 'upper="2"'))
            self.assertEqual(kinematic_digest(p), digest)
            p.write_text(xml.replace('0 0 1', '0 1 0'))
            self.assertNotEqual(kinematic_digest(p), digest)


if __name__ == '__main__':
    unittest.main()
