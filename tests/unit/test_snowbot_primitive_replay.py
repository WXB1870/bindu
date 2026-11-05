"""Offline primitive replay checks; all I/O is an in-memory port."""
import copy
import hashlib
import json
import math
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
        self.acquired_at = None
    def steady(self): return self.now
    def ros(self): return 100.+self.now+self.clock_shift
    def feedback_stamp(self): return 100.+self.now
    def elapsed(self): return 0. if self.acquired_at is None else self.now-self.acquired_at
    def wait_until(self, due):
        self.now = max(self.now, due)
        if self.late: self.now += .1
    def check(self, owned):
        if self.fail_after is not None and len(self.sent) >= self.fail_after:
            raise RuntimeError('STALE_FEEDBACK')
        q = self.plan.sample(self.elapsed())
        return dict(zip(NAMES, q)), dict.fromkeys(NAMES, 0.)
    def acquire(self):
        self.acquired_at = self.now
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
            if kind == 'late': port.wait_until = lambda due: setattr(port, 'now', due+(.3 if port.acquired_at is not None else 0.))
            else:
                original = port.wait_until
                def shifted(due):
                    original(due)
                    if port.acquired_at is not None: port.clock_shift = .3
                port.wait_until = shifted
            with self.assertRaisesRegex(RuntimeError, 'STREAM_LATE|CLOCK_CHANGED'):
                run_replay(p, port)
            self.assertFalse(port.sent)
            self.assertTrue(port.released)

    def test_scheduling_delay_inside_ttl_preserves_original_source_time(self):
        p = self.plan()
        port = FakePort(p)
        wait = port.wait_until
        def delayed(due):
            wait(due)
            if port.acquired_at is not None and not port.sent: port.now += .06
        port.wait_until = delayed
        self.assertTrue(run_replay(p, port)['completed'])
        self.assertEqual(port.sent[0][1], 100.+port.acquired_at)
        self.assertTrue(all(t-(source-100.) < .25 for t,source,_ in port.sent))

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

    def test_loader_reuses_published_asset_and_checks_playback_speed(self):
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
            self.assertEqual(load_replay(manifest, 'Push').points[-1][0], .001)

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


class PackagedLibraryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import snowbot_bringup.teleop.snowbot_primitive_replay as module
        cls.app = Path(module.__file__).resolve().parents[4]
        cls.config = cls.app/'src/snowbot_bringup/config/robots/snowbot'
        cls.library = cls.config/'primitive-library/manifest.json'
        cls.manifest = json.loads(cls.library.read_text())

    def test_fast_timing_has_no_unvalidated_waist_phase_shift(self):
        for action, duration in (('Push', 11.5), ('SideScoop', 8.9), ('Flip', 16.18)):
            with self.subTest(action=action):
                plan = load_replay(self.library, action)
                self.assertEqual(plan.waist_lead_s, 0.)
                self.assertAlmostEqual(plan.duration, duration, places=6)

    def test_final_fifty_hz_derivatives_and_independent_return_routes(self):
        from snowbot_bringup.teleop.snowbot_primitive_replay import plan_from_current
        for action in self.manifest['actions'] | self.manifest['transfers']:
            plan = load_replay(self.library, action)
            self.assertEqual(len(plan.times), len(plan.velocities))
            self.assertTrue(all(abs(b-a-.02)<1e-8 for a,b in zip(plan.times,plan.times[1:])))
            for k in range(0,len(plan.times),17):
                q,v,a = plan.state(plan.times[k])
                for actual,expected in ((q,plan.points[k]),(v,plan.velocities[k]),(a,plan.accelerations[k])):
                    self.assertLess(max(abs(x-y) for x,y in zip(actual,expected)),1e-7)
        for node,route in (('Dump:end',('DumpReturn',)),('Flip:start',('ScoopReturn',))):
            plan = plan_from_current(self.library,'Home',dict(zip(NAMES,self.manifest['nodes'][node])))
            self.assertEqual(plan.route,route)
            self.assertTrue(plan.trajectory_knots())
        for node in ('Push:start','Push:end','SideScoop:start','Dump:end'):
            with self.assertRaisesRegex(RuntimeError,'REQUIRES_COMPLETED_SIDE_SCOOP'):
                plan_from_current(self.library,'Flip',dict(zip(NAMES,self.manifest['nodes'][node])))

    def test_all_packaged_paths_keep_the_upright_shoulder_branch(self):
        for action in self.manifest['actions'] | self.manifest['transfers']:
            with self.subTest(action=action):
                plan = load_replay(self.library, action)
                shoulder = plan.names.index('arm_r_joint_1')
                self.assertLess(max(abs(q[shoulder]) for q in plan.points), math.pi/3)

    def test_prepare_configuration_uses_library_home_as_single_source(self):
        config = json.loads((self.config/'teleop.json').read_text())
        self.assertNotIn('prepare_positions_rad', config)
        for mode in ('real', 'simulation'):
            self.assertNotIn('prepare_positions_rad', config['modes'][mode])
        home = self.manifest['nodes'][self.manifest['home_node']]
        self.assertEqual(len(home), len(NAMES))
        self.assertLess(max(abs(a-b) for a,b in zip(home, load_replay(self.library, self.manifest['home_node'].split(':')[0]).points[0])),1e-12)

    def test_packaged_paths_fit_the_runtime_profile(self):
        from bindu_contracts.profile import Profile
        profile = Profile.load(self.config/'snowbot_real_position.json')
        for action in self.manifest['actions'] | self.manifest['transfers']:
            with self.subTest(action=action):
                load_replay(self.library, action).validate_profile(profile)

    def test_arm_speed_limit_is_uniform_and_matches_the_target_stream(self):
        from bindu_contracts.profile import Profile
        profile = Profile.load(self.config/'snowbot_real_position.json')
        parameters = (self.app/'tools/deployment/snowbot_vendor/position_session/target_stream.yaml').read_text()
        ceiling = float(next(line.split(':', 1)[1] for line in parameters.splitlines()
                             if line.strip().startswith('trajectory_max_velocity:')))
        self.assertEqual(ceiling, 1.25)
        arms = [n for side in ('left_arm', 'right_arm') for n in profile.groups[side]]
        self.assertEqual(len(arms), 14)
        self.assertEqual({profile.speed(n) for n in arms}, {ceiling})

    def test_packaged_successors_and_home_join_the_updated_nodes(self):
        from snowbot_bringup.teleop.snowbot_primitive_replay import plan_from_current
        for source in ('Push', 'SideScoop', 'Flip'):
            node = self.manifest['actions'][source]['end_node']
            current = dict(zip(NAMES, self.manifest['nodes'][node]))
            for action in (('Push', 'SideScoop', 'Flip', 'Home') if source == 'SideScoop'
                           else ('Push', 'SideScoop', 'Home')):
                with self.subTest(source=source, action=action):
                    plan = plan_from_current(self.library, action, current)
                    self.assertLess(max(abs(a-b) for a, b in zip(
                        plan.points[0], (current[n] for n in NAMES))), 1e-8)
                    end = (self.manifest['home_node'] if action == 'Home'
                           else self.manifest['actions'][action]['end_node'])
                    self.assertLess(max(abs(a-b) for a, b in zip(
                        plan.sample(plan.duration), self.manifest['nodes'][end])), 1e-8)
                    for before, after in zip(plan.segments, plan.segments[1:]):
                        self.assertLess(max(abs(a-b) for a, b in zip(
                            before.points[-1], after.points[0])), 1e-8)


class PrimitiveTransitionTests(unittest.TestCase):
    def fixture(self, root):
        import gzip
        nodes = {name: [v]*16 for name,v in [('Push:start',0.),('Push:end',.01),
                 ('SideScoop:start',.02),('Flip:start',.03),('Dump:end',.04)]}
        definitions = [('Push','Push:start','Push:end'),('PushReverse','Push:end','Push:start'),
            ('BridgeForward','Push:start','SideScoop:start'),('BridgeReverse','SideScoop:start','Push:start'),
            ('ScoopPrefix','SideScoop:start','Flip:start'),('ScoopPrefixReverse','Flip:start','SideScoop:start'),
            ('Flip','Flip:start','Dump:end'),('FlipReverse','Dump:end','Flip:start'),
            ('SideScoop','SideScoop:start','Flip:start')]
        m = dict(schema_version=1,validation_passed=True,transition_validation_passed=True,
            tool_id='g099-curved',kinematic_sha256='kin',waist_lead_s=.02,
            home_node='SideScoop:start',preparation={'goal_node':'Push:start'},
            nodes=nodes,actions={},transfers={},edges=[])
        for name,a,b in definitions:
            x=dict(schema_version=1,mode='joint_target',runtime_mode='real',joint_names=list(NAMES),
                   resource_groups=list(GROUPS),period_s=.02,valid_for_s=.25,start_q_rad=nodes[a],
                   samples=[dict(t=0.,q=nodes[a]),dict(t=.02,q=nodes[b])])
            payload=gzip.compress(json.dumps(x).encode(),mtime=0)
            file=root/(name+'.json.gz');file.write_bytes(payload)
            item=dict(targets=file.name,sha256=hashlib.sha256(payload).hexdigest(),start_node=a,end_node=b)
            m['actions' if name in ('Push','SideScoop','Flip') else 'transfers'][name]=item
            if name!='SideScoop':m['edges'].append({'from':a,'to':b,'segment':name})
        file=root/'manifest.json';file.write_text(json.dumps(m));return file,m

    def test_all_nine_successors_and_a_home_have_continuous_certified_joins(self):
        from snowbot_bringup.teleop.snowbot_primitive_replay import plan_from_current
        with tempfile.TemporaryDirectory() as d:
            path,m=self.fixture(Path(d))
            for source in ('Push','SideScoop','Flip'):
                current=dict(zip(NAMES,m['nodes'][m['actions'][source]['end_node']]))
                for action in (('Push','SideScoop','Flip','Home') if source == 'SideScoop'
                               else ('Push','SideScoop','Home')):
                    with self.subTest(source=source,action=action):
                        p=plan_from_current(path,action,current)
                        self.assertEqual(p.points[0],tuple(current[n] for n in NAMES))
                        end=m['nodes'][m['home_node'] if action=='Home' else m['actions'][action]['end_node']]
                        self.assertEqual(p.points[-1],tuple(end))
                        self.assertEqual(p.sample(p.duration),tuple(end))
                        t=0.
                        for a,b in zip(p.segments,p.segments[1:]):
                            t+=a.duration
                            self.assertEqual(p.sample(t+1.),a.points[-1])
                            self.assertEqual(a.points[-1],b.points[0])
                            t+=p.hold_s
                        self.assertTrue(run_replay(p,FakePort(p))['completed'])

    def test_unknown_start_is_rejected_before_execution(self):
        from snowbot_bringup.teleop.snowbot_primitive_replay import plan_from_current
        with tempfile.TemporaryDirectory() as d:
            path,m=self.fixture(Path(d))
            with self.assertRaisesRegex(RuntimeError,'UNKNOWN_START'):
                plan_from_current(path,'Push',dict.fromkeys(NAMES,.8))


class InteractiveReplayTests(unittest.TestCase):
    def test_dispatch_does_not_require_a_manual_backend_ready_flag(self):
        from contextlib import redirect_stderr
        from io import StringIO
        from unittest.mock import patch
        import snowbot_bringup.teleop.snowbot_primitive_replay as module
        for flag in (None, False):
            with self.subTest(flag=flag), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                library, manifest = PrimitiveTransitionTests().fixture(root)
                if flag is None:
                    manifest.pop('backend_ready', None)
                else:
                    manifest['backend_ready'] = flag
                library.write_text(json.dumps(manifest))
                with patch.object(module, 'execute') as execute, redirect_stderr(StringIO()):
                    module.main(['--library', str(library), '--action', 'Push',
                                 '--from-node', 'Push:start', '--execute',
                                 '--tool-id', 'g099-curved', '--output', str(root/'run.jsonl')])
                execute.assert_called_once()
                self.assertEqual(execute.call_args.args[1].route, ('Push',))

    def test_staged_menu_executes_actions_and_returns_with_separate_logs(self):
        from contextlib import redirect_stdout
        from io import StringIO
        from unittest.mock import patch
        import snowbot_bringup.teleop.snowbot_primitive_replay as module
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            library,_=PrimitiveTransitionTests().fixture(root)
            with patch('builtins.input',side_effect=['invalid','1','','2','3','','q']), \
                    patch.object(module,'execute') as execute,redirect_stdout(StringIO()):
                module.main(['--library',str(library),'--interactive','--execute',
                    '--execution-mode','position','--tool-id','g099-curved',
                    '--output-dir',str(root/'logs')])
            self.assertEqual([c.args[0].action for c in execute.call_args_list],['Push','PushReverse','SideScoop','Flip','Home'])
            outputs=[c.args[0].output for c in execute.call_args_list]
            self.assertEqual(len(set(outputs)),5)
            self.assertTrue(all(p.parent==root/'logs' for p in outputs))
            self.assertTrue(all(c.args[0].execution_mode=='position' for c in execute.call_args_list))

    def test_action_failure_exits_menu_before_another_selection(self):
        from contextlib import redirect_stdout
        from io import StringIO
        from unittest.mock import patch
        import snowbot_bringup.teleop.snowbot_primitive_replay as module
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            library,_=PrimitiveTransitionTests().fixture(root)
            with patch('builtins.input',side_effect=['1','2']) as choose, \
                    patch.object(module,'execute',side_effect=RuntimeError('STOP_UNCONFIRMED')) as execute, \
                    redirect_stdout(StringIO()):
                with self.assertRaisesRegex(RuntimeError,'STOP_UNCONFIRMED'):
                    module.main(['--library',str(library),'--interactive','--execute',
                        '--execution-mode','position','--tool-id','g099-curved',
                        '--output-dir',str(root/'logs')])
            self.assertEqual(choose.call_count,1)
            self.assertEqual(execute.call_count,1)


class EmptyPrepareTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from bindu_runtime.robot_setup import RobotSetup
        from bindu_contracts.resources import package_share
        cls.robot=RobotSetup.load(package_share('snowbot_bringup')/'config/robots/snowbot/robot_snowbot_real.json')
        cls.library=cls.robot.tool_config.parent/'primitive-library/manifest.json'

    def measured(self):
        return dict.fromkeys((n for group in self.robot.profile.groups.values() for n in group),0.)

    def test_empty_entry_reaches_configured_ready_and_retains_reference_margin(self):
        from dataclasses import replace
        from unittest.mock import Mock,patch
        from snowbot_bringup.teleop.snowbot_primitive_replay import plan_prepare
        checker=Mock()
        with patch('bindu_kinematics.tool_geometry.make_collision_checker',return_value=checker) as factory:
            plan=plan_prepare(self.library,self.robot,self.measured())
        self.assertEqual(plan.points[0],(0.,)*16)
        manifest=json.loads(self.library.read_text())
        expected=tuple(manifest['nodes'][manifest['preparation']['goal_node']])
        self.assertLess(max(abs(a-b) for a,b in zip(expected,plan.points[-1])),1e-12)
        self.assertEqual(plan.reference_margin_rad,.027453292519943294)
        self.assertTrue(plan.empty_prepare)
        self.assertGreaterEqual(plan.duration,2.)
        self.assertLess(plan.duration,30.)
        self.assertFalse(factory.call_args.kwargs['tool_present'])
        checked = checker.call_args.args[0]
        self.assertGreater(len(checked), 2)
        self.assertTrue(all(max(abs(b['q'][n]-a['q'][n]) for n in NAMES) <= .005+1e-12
                            for a, b in zip(checked, checked[1:])))
        plan.validate_profile(self.robot.profile)
        with self.assertRaisesRegex(ValueError,'POSITION_LIMIT'):
            replace(plan,empty_prepare=False).validate_profile(self.robot.profile)
        result=run_replay(plan,FakePort(plan))
        self.assertTrue(result['completed'] and result['stop_confirmed'])

    def test_collision_or_missing_measured_context_prevents_preparation(self):
        from unittest.mock import Mock,patch
        from snowbot_bringup.teleop.snowbot_primitive_replay import plan_prepare
        with patch('bindu_kinematics.tool_geometry.make_collision_checker',
                   return_value=Mock(side_effect=ValueError('SELF_COLLISION'))):
            with self.assertRaisesRegex(ValueError,'SELF_COLLISION'):
                plan_prepare(self.library,self.robot,self.measured())
        measured=self.measured();measured.pop('head_joint_1')
        with self.assertRaisesRegex(ValueError,'MEASUREMENT_REQUIRED:head_joint_1'):
            plan_prepare(self.library,self.robot,measured)

    def test_small_measured_limit_excursion_uses_confirmed_inward_reference(self):
        from unittest.mock import Mock,patch
        from snowbot_bringup.teleop.snowbot_primitive_replay import plan_prepare
        joint='arm_r_joint_2'
        for excursion in (9.587379463482648e-5, .03490658503988659):
            with self.subTest(excursion=excursion):
                measured=self.measured();measured[joint]=excursion
                original=dict(measured);reference=dict.fromkeys(NAMES,0.)
                checker=Mock()
                with patch('bindu_kinematics.tool_geometry.make_collision_checker',return_value=checker):
                    plan=plan_prepare(self.library,self.robot,measured,reference=reference if excursion>.001 else None)
                self.assertEqual(measured,original)
                self.assertEqual(plan.measured_start,tuple(measured[n] for n in NAMES))
                self.assertEqual(plan.points[0],tuple(reference[n] for n in NAMES))
                samples=checker.call_args.args[0]
                self.assertEqual(samples[0]['q'][joint],excursion)
                self.assertTrue(all(self.robot.profile.limits[n][0]<=q<=self.robot.profile.limits[n][1]
                                    for row in plan.points for n,q in zip(NAMES,row)))
                self.assertTrue(all(a[joint]>=b[joint] for a,b in
                    zip((x['q'] for x in samples),(x['q'] for x in samples[1:]))))
                port=FakePort(plan);check=port.check
                port.nominal_reference=lambda:dict(zip(NAMES,plan.sample(port.now)))
                def compliant(owned):
                    q,v=check(owned);q[joint]+=excursion
                    return q,v
                port.check=compliant
                result=run_replay(plan,port,execution_mode='impedance')
                self.assertTrue(result['completed'] and result['stop_confirmed'])
                self.assertTrue(all(self.robot.profile.limits[joint][0]<=q[NAMES.index(joint)]<=0.
                                    for _,_,q in port.sent))

    def test_limit_recovery_rejects_larger_excursion_or_unconfirmed_reference(self):
        from unittest.mock import Mock,patch
        from snowbot_bringup.teleop.snowbot_primitive_replay import plan_prepare
        measured=self.measured();measured['arm_r_joint_2']=.03490658503988659+1e-6
        with self.assertRaisesRegex(ValueError,'PREPARE_START_JOINT_LIMIT:arm_r_joint_2'):
            plan_prepare(self.library,self.robot,measured,reference=dict.fromkeys(NAMES,0.))
        measured['arm_r_joint_2']=.01
        with self.assertRaisesRegex(ValueError,'PREPARE_HELD_REFERENCE_REQUIRED'):
            plan_prepare(self.library,self.robot,measured)
        reference=dict.fromkeys(NAMES,0.);reference['arm_r_joint_2']=.001
        with self.assertRaisesRegex(ValueError,'PREPARE_LIMIT_RECOVERY_BOUND:arm_r_joint_2'):
            plan_prepare(self.library,self.robot,measured,reference=reference)
        reference['arm_r_joint_2']=0.
        with patch('bindu_kinematics.tool_geometry.make_collision_checker',
                   return_value=Mock(side_effect=ValueError('SELF_COLLISION'))):
            with self.assertRaisesRegex(ValueError,'SELF_COLLISION'):
                plan_prepare(self.library,self.robot,measured,reference=reference)

    def test_boundary_entry_cannot_move_outward_or_drop_reference_margin(self):
        profile=S(groups=dict(zip(GROUPS,(list(NAMES[:2]),list(NAMES[2:9]),list(NAMES[9:])))),
                  limits=dict.fromkeys(NAMES,(0.,1.)),speed=lambda n:1.)
        points=[[.1]*16 for _ in range(3)]
        points[0][2]=.01;points[1][2]=.005
        plan=Replay((0.,1.,2.),tuple(map(tuple,points)),NAMES,'tool','kin',1.,
                    reference_margin_rad=.03,empty_prepare=True)
        with self.assertRaisesRegex(ValueError,'REFERENCE_MARGIN_ENTRY'):
            plan.validate_profile(profile)

class PrepareGripperTests(unittest.TestCase):
    def test_each_enter_precedes_the_corresponding_gripper_command(self):
        from unittest.mock import Mock,patch
        from snowbot_bringup.teleop.snowbot_primitive_replay import confirm_prepare_grippers
        port=Mock()
        expected=iter((0,1))
        def enter(message):
            self.assertEqual(port.grippers.call_count,next(expected))
            return ''
        with patch('builtins.input',side_effect=enter):
            confirm_prepare_grippers(port,'open')
            confirm_prepare_grippers(port,'close')
        self.assertEqual([c.args for c in port.grippers.call_args_list],[('open',),('close',)])
        port.reset_mock()
        with patch('builtins.input',side_effect=KeyboardInterrupt):
            with self.assertRaises(KeyboardInterrupt):confirm_prepare_grippers(port,'open')
        port.grippers.assert_not_called()



class ImpedancePrimitiveReplayTests(unittest.TestCase):

    def test_impedance_uses_existing_named_tracking_limits(self):
        from snowbot_bringup.teleop.snowbot_primitive_replay import replay_follow_limits
        cfg=dict(reference_kinematics=True,max_reference_error=.15,
            impedance_reference_limits={'joint_reference_error_limits':{
                n:.25 for n in NAMES if n.startswith('arm_')}})
        self.assertEqual(set(replay_follow_limits(cfg,'position').values()),{.1})
        limits=replay_follow_limits(cfg,'impedance')
        self.assertEqual(limits['body_joint_1'],.15)
        self.assertEqual(limits['arm_l_joint_1'],.25)

    def test_compliant_displacement_never_moves_the_nominal_target(self):
        class CompliantPort(FakePort):
            def check(self,owned):
                q,v=super().check(owned)
                if owned and self.sent and self.elapsed()<.04:
                    for n in NAMES[2:]:q[n]+=.2
                return q,v
        p=Replay((0.,.02,.04),((0.,)*16,(.001,)*16,(.002,)*16),
                 NAMES,'tool','kin',1.)
        port=CompliantPort(p)
        self.assertTrue(run_replay(p,port)['completed'])
        self.assertTrue(all(q==p.sample(t-port.acquired_at) for t,_,q in port.sent))


    def test_steady_compliance_uses_nominal_start_and_arrival(self):
        class Port(FakePort):
            offset=.2
            def check(self,owned):
                q,v=super().check(owned)
                for n in NAMES[2:]:q[n]+=self.offset
                return q,v
            def nominal_reference(self):return dict(zip(NAMES,self.plan.sample(self.elapsed())))
        p=Replay((0.,.02,.04),((0.,)*16,(.001,)*16,(.002,)*16),NAMES,'t','k',1.)
        port=Port(p);result=run_replay(p,port,execution_mode='impedance')
        self.assertTrue(result['completed'] and result['nominal_arrival_confirmed'] and result['stop_confirmed'])
        self.assertFalse(result['measured_arrival_confirmed'])
        self.assertTrue(all(q==p.sample(t-port.acquired_at) for t,_,q in port.sent))

    def test_loss_of_nominal_reference_cancels_and_confirms_stop(self):
        class Port(FakePort):
            def nominal_reference(self):
                if self.elapsed()>=self.plan.duration:raise RuntimeError('NOMINAL_REFERENCE_STALE')
                return dict(zip(NAMES,self.plan.sample(self.elapsed())))
        p=Replay((0.,.02),((0.,)*16,(.001,)*16),NAMES,'t','k',1.)
        port=Port(p)
        with self.assertRaisesRegex(RuntimeError,'NOMINAL_REFERENCE_STALE'):
            run_replay(p,port,execution_mode='impedance')
        self.assertTrue(port.released)

    def test_held_and_sampled_reference_authentication_and_freshness(self):
        import ast,math
        import snowbot_bringup.teleop.snowbot_primitive_replay as module
        from bindu_runtime.stop_evidence import sampled_nominal_reference
        tree=ast.parse(Path(module.__file__).read_text())
        operation=next(n for n in tree.body if getattr(n,'name','')=='execute')
        cls=next(n for n in operation.body if isinstance(n,ast.ClassDef) and n.name=='Port')
        state=S(lease_active=False,lease_epoch=4,revision=7)
        held=dict(instance_id='exec',profile_hash='hash',lease_id='',epoch=4,revision=7,
                  stamp=100.,boot_id=55,session=2,sequence=3,positions=dict.fromkeys(NAMES,0.))
        future=S(done=lambda:True,result=lambda:S(success=True,message=json.dumps(held)))
        ns=dict(latest={'state':(state,0.)},NAMES=NAMES,math=math,json=json,
                robot=S(profile=S(digest='hash',limits=dict.fromkeys(NAMES,(-1,1)))),
                tool_config={'feedback_max_age':.2},Trigger=S(Request=lambda:None),
                holds=S(service_is_ready=lambda:True,call_async=lambda req:future),
                wait=lambda predicate,timeout:self.assertTrue(predicate()),
                sampled_nominal_reference=sampled_nominal_reference)
        exec(compile(ast.Module(body=[cls],type_ignores=[]),str(module.__file__),'exec'),ns)
        port=ns['Port']();port.instance='exec';port.boot=55;port.ros=lambda:100.;port.steady=lambda:0.
        self.assertEqual(port.nominal_reference(),held['positions'])
        held['boot_id']=99
        with self.assertRaisesRegex(RuntimeError,'HELD_REFERENCE_IDENTITY'):port.nominal_reference()
        held['boot_id']=55;held['positions']=dict.fromkeys(NAMES,.1)
        with self.assertRaisesRegex(RuntimeError,'HELD_REFERENCE_CHANGED'):port.nominal_reference()
        port.lease=S(lease_id='lease',epoch=5);port.commanded=True
        state.lease_active=True;state.lease_epoch=5
        state.reference=S(header=S(stamp=S(sec=100,nanosec=0)),name=list(NAMES),position=[.1]*16)
        self.assertEqual(port.nominal_reference(),dict.fromkeys(NAMES,.1))
        state.reference.header.stamp.sec=99
        with self.assertRaisesRegex(RuntimeError,'REFERENCE_UNAVAILABLE'):port.nominal_reference()


    def test_short_nominal_hold_waits_for_first_servo_sample_without_stalling_targets(self):
        class Port(FakePort):
            def nominal_reference(self):
                return None if self.sent and self.elapsed()<.08 else dict(zip(NAMES,self.plan.sample(self.elapsed())))
        p=Replay((0.,.02),((0.,)*16,(0.,)*16),NAMES,'t','k',1.)
        port=Port(p);result=run_replay(p,port,execution_mode='impedance')
        self.assertTrue(result['completed'] and result['nominal_arrival_confirmed'])
        self.assertGreaterEqual(port.now,.28)
        self.assertTrue(all(abs(b[0]-a[0]-.02)<1e-8 for a,b in zip(port.sent,port.sent[1:])))

if __name__ == '__main__':
    unittest.main()
