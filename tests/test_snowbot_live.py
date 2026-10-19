"""The live viewport must reject stale, mixed-mode or malformed feedback."""
import copy
import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest
import subprocess
import signal
from unittest.mock import patch

spec = importlib.util.spec_from_file_location('snowbot_live', Path(__file__).resolve().parents[1]/'tools/snowbot_live.py')
helper = importlib.util.module_from_spec(spec)
spec.loader.exec_module(helper)
webspec = importlib.util.spec_from_file_location('snowbot_web', Path(__file__).resolve().parents[1]/'tools/snowbot_web.py')
web = importlib.util.module_from_spec(webspec)
webspec.loader.exec_module(web)
entryspec = importlib.util.spec_from_file_location('run_vr_sim', Path(__file__).resolve().parents[1]/'tools/run_vr.py')
entry = importlib.util.module_from_spec(entryspec)
entryspec.loader.exec_module(entry)


class LiveFeedbackTests(unittest.TestCase):
    def setUp(self):
        self.data = dict(schema=1, run_id='run', profile_hash='profile', runtime_mode='simulation',
                         device_backend='kinematic', valid=True, written_at=10., feedback_stamp=10., q={'joint':.2})

    def validate(self, value, now=10.1):
        return helper.validate_state(value, now, 'run', 'profile', ('joint',))

    def test_fresh_complete_feedback_is_displayable(self):
        self.assertEqual(self.validate(self.data), {'joint':.2})

    def test_real_feedback_requires_explicit_matching_backend(self):
        real=dict(self.data,runtime_mode='real',device_backend='snowbot_vendor')
        self.assertEqual(helper.validate_state(real,10.1,'run','profile',('joint',),'real','snowbot_vendor'),{'joint':.2})
        with self.assertRaises(ValueError):self.validate(real)
        with self.assertRaises(ValueError):
            helper.validate_state(self.data,10.1,'run','profile',('joint',),'real','snowbot_vendor')

    def test_recent_file_does_not_refresh_old_device_feedback(self):
        for field in ('feedback_stamp', 'written_at'):
            for value in (9., 11., float('nan')):
                changed = dict(self.data, **{field:value})
                with self.subTest(field=field,value=value), self.assertRaisesRegex(ValueError,'STALE'):
                    self.validate(changed)

    def test_identity_and_real_or_external_feedback_are_rejected(self):
        for field,value in (('schema',2),('run_id','old'),('profile_hash','other'),
                            ('runtime_mode','real'),('device_backend','external_simulation'),('valid',False)):
            with self.subTest(field=field), self.assertRaisesRegex(ValueError,'IDENTITY'):
                self.validate(dict(self.data, **{field:value}))

    def test_missing_extra_or_nonfinite_joints_are_rejected(self):
        for q in ({}, {'joint':.2,'extra':0.}, {'joint':float('nan')}, {'joint':True}):
            with self.subTest(q=q), self.assertRaisesRegex(ValueError,'LAYOUT'):
                self.validate(dict(self.data,q=q))


class WebFeedbackTests(unittest.TestCase):
    def test_error_samples_require_fresh_matching_feedback_and_pico_identity(self):
        metric=dict(valid=True,stamp=10.,coupled=[2.,.1])
        data=dict(self.data,end_effector_errors=metric)
        self.assertEqual(self.project(data)['end_effector_errors'],metric)
        self.assertNotIn('end_effector_errors',self.project(data,10.3))
        self.assertNotIn('end_effector_errors',self.project(dict(data,valid=False)))
        self.assertNotIn('end_effector_errors',self.project(dict(data,pico=dict(data['pico'],run_id='old'))))
        self.assertNotIn('end_effector_errors',self.project(dict(data,end_effector_errors=dict(metric,stamp=9.7))))

    def setUp(self):
        LiveFeedbackTests.setUp(self)
        self.data.update(joint_names=['joint'],instance_id='executor',pico=dict(
            run_id='run',profile_hash='profile',runtime_mode='simulation',stamp=10.,
            input_age_s=.02,raw_input_age_s=.02,input_fresh=True,
            controller_states={'left':dict(valid=True,buttons=1,squeeze=1.,trigger=0.)}))

    def project(self,data,now=10.1):
        with patch.dict(sys.modules,{'snowbot_live':helper}):
            return web.project_state(data,now,self.data)

    def test_executor_restart_is_not_silently_displayed(self):
        view=self.project(dict(self.data,instance_id='new-executor'))
        self.assertFalse(view['valid'])
        self.assertEqual(view['code'],'LIVE_EXECUTOR_CHANGED')

    def test_browser_refresh_does_not_refresh_input_values(self):
        view=self.project(self.data,10.3)
        self.assertTrue(view['valid'])
        self.assertFalse(view['pico']['input_fresh'])
        self.assertIsNone(view['pico']['controller_states']['left']['buttons'])
        self.assertEqual(self.data['pico']['controller_states']['left']['buttons'],1)

    def test_stale_or_mixed_session_pico_is_hidden(self):
        for value in (dict(self.data['pico'],stamp=9.),dict(self.data['pico'],run_id='another')):
            view=self.project(dict(self.data,pico=value))
            self.assertTrue(view['valid'])
            self.assertFalse(view['pico_fresh'])
            self.assertEqual(view['pico'],{})

    def test_coalesced_button_click_is_logged_once(self):
        with tempfile.TemporaryDirectory() as directory:
            log=web.EventLog(Path(directory)/'events.jsonl')
            s=self.project(self.data)
            p=s['pico'];p.update(phase='tool_standby',reason='READY',source_id='controller',raw_input_valid=True)
            p['controller_states']['left']['button_edges']=[0]*8
            log.sample(s)
            before=len(log.events)
            s=copy.deepcopy(s);s['pico']['controller_states']['left']['button_edges']=[1,1,0,0,0,0,0,0]
            log.sample(s);log.sample(s)
            self.assertEqual(len(log.events)-before,2)
            self.assertEqual([e['message'] for e in log.events[-2:]],['左 X 按下','左 X 松开'])

    def test_empty_cad_mesh_is_omitted_without_removing_its_link(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);(root/'urdf').mkdir()
            (root/'empty.STL').write_bytes(b'0'*80+b'\x00'*4)
            source=root/'urdf'/'model.urdf'
            source.write_text(f'<robot name="r"><link name="head"><visual><geometry><mesh filename="{root}/empty.STL"/></geometry></visual></link></robot>')
            target=web.meshcat_urdf(source,root)
            self.assertIn('name="head"',target.read_text())
            self.assertNotIn('<visual>',target.read_text())

    def test_memory_events_are_bounded_and_exportable_without_disk(self):
        log = web.EventLog()
        for i in range(400):
            state = self.project(self.data)
            state['pico']['reason'] = str(i)
            log.sample(state)
        self.assertIsNone(log.path)
        self.assertEqual(len(log.events), 300)
        self.assertEqual(log.events[-1]['reason'], '399')

    def test_shared_snapshot_is_detached_and_never_refreshes_feedback(self):
        store = helper.LatestState()
        store.publish(self.data)
        self.data['q']['joint'] = 9.
        first = store.read()
        self.assertEqual(first['q']['joint'], .2)
        first['q']['joint'] = 8.
        self.assertEqual(store.read()['q']['joint'], .2)
        self.assertFalse(self.project(store.read(), now=11.)['valid'])


class SimulationEntryTests(unittest.TestCase):
    def test_physics_selects_external_feedback_and_keeps_browser(self):
        args=entry.parse_args(['--physics','--run-id','physics_test'])
        physics,control,display=entry.commands(args)
        self.assertIn('--vr',physics)
        self.assertIn('device_backend:=external_simulation',control)
        self.assertEqual(display[display.index('--device-backend')+1],'external_simulation')
        self.assertIn('--dashboard',display)

    def test_real_recipe_cannot_enable_isaac(self):
        root=Path(__file__).resolve().parents[1]
        with self.assertRaises(SystemExit):
            entry.parse_args(['--physics','--config',str(root/'src/integration/bindu_runtime/config/deployments/snowbot_vr_real.json')])

    def test_device_selection_propagates_to_control_and_display(self):
        root=Path(__file__).resolve().parents[1]
        args=entry.parse_args(['--config',str(root/'src/integration/bindu_runtime/config/deployments/snowbot_vr_real.json')])
        control,display=entry.commands(args)
        self.assertIn('runtime_mode:=real',control)
        self.assertIn('device_backend:=snowbot_vendor',control)
        self.assertIn('command_enabled:=false',control)
        self.assertIn('position_contract:=true',control)
        self.assertNotIn('pico_simulation_demo:=true',control)
        self.assertEqual(display[display.index('--runtime-mode')+1],'real')
        self.assertEqual(display[display.index('--device-backend')+1],'snowbot_vendor')

    def test_partial_start_failure_stops_already_started_control(self):
        with tempfile.TemporaryDirectory() as directory:
            args = entry.parse_args(['--output', str(Path(directory)/'run'), '--no-diagnostics',
                                     '--namespace', 'test_'+Path(directory).name])
            control = object()
            with patch.object(entry, 'check'), patch.object(entry.subprocess, 'Popen',
                    side_effect=[control, OSError('display could not start')]), \
                    patch.object(entry, 'stop_group', return_value=True) as stop:
                with self.assertRaisesRegex(OSError, 'display could not start'):
                    entry.run(args)
                stop.assert_called_once_with(control)

    def test_shared_identity_and_fixed_simulation_flags(self):
        args = entry.parse_args(['--run-id', 'test', '--namespace', '/test_ns/'])
        control, display = entry.commands(args)
        self.assertIn('runtime_mode:=simulation', control)
        self.assertIn('device_backend:=kinematic', control)
        self.assertIn('run_id:=test', control)
        self.assertIn('namespace:=test_ns', control)
        self.assertEqual(display[display.index('--run-id')+1], 'test')
        self.assertEqual(display[display.index('--namespace')+1], 'test_ns')
        self.assertIn('--dashboard', display)
        self.assertFalse(any(value.endswith(':=') for value in control))
        self.assertNotIn('--state-file', display)
        self.assertIn('--trace-file', display)
        args.diagnostics = False
        self.assertNotIn('--trace-file', entry.commands(args)[1])

    def test_bad_parameters_fail_before_processes_start(self):
        for argv in (['--run-id', '../escape'], ['--namespace', 'bad:ns'],
                     ['--port', '8012'], ['--duration', 'nan'], ['--domain-id', '233'],
                     ['--check','--declare-load','empty'], ['--external-support']):
            with self.subTest(argv=argv), patch('sys.stderr'), self.assertRaises(SystemExit):
                entry.parse_args(argv)

    def test_process_group_stops_gracefully(self):
        child = subprocess.Popen([sys.executable, '-u', '-c',
            'import signal,time; signal.signal(signal.SIGINT, lambda *_: exit(0)); print("ready",flush=True); time.sleep(30)'],
            stdout=subprocess.PIPE, text=True, start_new_session=True)
        try:
            self.assertEqual(child.stdout.readline().strip(), 'ready')
            self.assertTrue(entry.stop_group(child, grace=1.))
            self.assertEqual(child.returncode, 0)
        finally:
            if child.poll() is None:
                child.kill(); child.wait()
            child.stdout.close()

    def test_forced_shutdown_is_not_reported_as_graceful(self):
        child = subprocess.Popen([sys.executable, '-u', '-c',
            'import signal,time; signal.signal(signal.SIGINT, signal.SIG_IGN); print("ready",flush=True); time.sleep(30)'],
            stdout=subprocess.PIPE, text=True, start_new_session=True)
        try:
            self.assertEqual(child.stdout.readline().strip(), 'ready')
            self.assertFalse(entry.stop_group(child, grace=.05))
            self.assertEqual(child.returncode, -signal.SIGTERM)
        finally:
            if child.poll() is None:
                child.kill(); child.wait()
            child.stdout.close()
