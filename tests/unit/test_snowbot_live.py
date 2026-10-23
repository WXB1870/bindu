"""The live viewport must reject stale, mixed-mode or malformed feedback."""
from tests.support.paths import ROOT
import copy
import importlib.util
from io import StringIO
import json
from pathlib import Path
import sys
import tempfile
import unittest
import subprocess
import signal
import shutil
from types import SimpleNamespace as S
from unittest.mock import patch

spec = importlib.util.spec_from_file_location('snowbot_live', ROOT/'tools/teleop/snowbot_live.py')
helper = importlib.util.module_from_spec(spec)
spec.loader.exec_module(helper)
webspec = importlib.util.spec_from_file_location('snowbot_web', ROOT/'tools/teleop/snowbot_web.py')
web = importlib.util.module_from_spec(webspec)
webspec.loader.exec_module(web)
entryspec = importlib.util.spec_from_file_location('run_vr_sim', ROOT/'tools/teleop/run_vr.py')
entry = importlib.util.module_from_spec(entryspec)
entryspec.loader.exec_module(entry)


class BrowserControlFailureTests(unittest.TestCase):
    def test_startup_failure_feedback_loss_and_web_disconnect_have_separate_status(self):
        node=shutil.which('node')
        self.assertIsNotNone(node, 'Node is required for this browser logic check')
        html=(ROOT/'tools/web/snowbot.html').read_text()
        js=html[html.index('function operatorState('):html.index('for(const [side,name,keys]')]
        code="const phases={};const explainReason=r=>r;\n"+js+"""
const assert=require('node:assert/strict');
const p={phase:'locked',reason:'PICO_STOP_UNCONFIRMED',stop_confirmed:false};
const e={execution:'FAILED',task_id:'startup_hold'};
let s=operatorState(p,true,true,true,true,e);
assert.equal(s.status,'启动接管失败');assert.match(s.reason,/关节反馈正常/);assert.equal(s.key,'');
assert.match(s.next,/不能进入准备/);
assert.equal(operatorState(p,true,false,true,true,e).status,'关节反馈不可用');
assert.equal(operatorState(p,false,true,true,true,e).status,'网页连接中断');
assert.equal(operatorState(p,true,true,false,true,e).status,'遥操作状态未知');
assert.equal(operatorState(p,true,true,true,true,{}).status,'停止未确认');
"""
        result=subprocess.run([node,'-e',code],capture_output=True,text=True)
        self.assertEqual(result.returncode,0,result.stderr)


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
    def test_joint_targets_stay_separate_from_measured_feedback_and_stale_views(self):
        data=dict(self.data,joint_targets={'joint':.3})
        view=self.project(data)
        self.assertEqual(view['joint_comparison'][0]['measured'],.2)
        self.assertEqual(view['joint_comparison'][0]['target'],.3)
        self.assertNotIn('joint_comparison',self.project(data,10.7))
        self.assertFalse(self.project(dict(data,joint_targets={'joint':float('nan')}))['joint_comparison'])

    def test_pdo_faults_are_visible_independently_and_old_views_are_marked_stale(self):
        pdo=dict(available=True,pdo_valid=True,checked_at=10.,fault_mask=1,fault_axes=['L1'],
                 fault_mask_complete=True,reasons=['DRIVE_FAULT'])
        data=dict(self.data,runtime_mode='real',device_backend='snowbot_vendor',pdo=pdo)
        view=web.project_state(data,10.1,dict(data))
        self.assertTrue(view['pdo']['pdo_valid'])
        self.assertEqual(view['pdo']['fault_axes'],['L1'])
        stale=web.project_state(data,10.3,dict(data))
        self.assertFalse(stale['pdo']['pdo_valid'])
        self.assertIn('PDO_VIEW_STALE',stale['pdo']['reasons'])
        self.assertEqual(data['pdo']['reasons'],['DRIVE_FAULT'])
        self.assertNotIn('pdo',self.project(dict(self.data,pdo=pdo)))

    def test_fresh_measured_pose_keeps_execution_fault_visible(self):
        p=dict(self.data['pico'],phase='locked',reason='PICO_STOP_UNCONFIRMED:vendor_hold',
               motion_stopped=False,stop_confirmed=False)
        data=dict(self.data,execution='FAILED',task_id='startup_hold',stop_failures=['vendor_hold'],code='STOP_UNCONFIRMED:vendor_hold',pico=p)
        view=self.project(data)
        self.assertTrue(view['valid'])
        self.assertEqual(view['code'],'STOP_UNCONFIRMED:vendor_hold')
        self.assertEqual(view['task_id'],'startup_hold')
        self.assertEqual(view['stop_failures'],['vendor_hold'])
        self.assertEqual(view['pico']['reason'],p['reason'])
        self.assertFalse(view['pico']['stop_confirmed'])
        self.assertFalse(self.project(data,10.7)['valid'])

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
        with patch.dict(sys.modules,{'tools.teleop.snowbot_live':helper}):
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
            log.close()
            saved=[json.loads(line) for line in log.path.read_text().splitlines()]
            self.assertEqual(saved,log.events)

    def test_web_events_queue_without_disk_wait_and_keep_the_existing_jsonl_format(self):
        from tools.teleop import snowbot_diagnostics as diagnostics
        import threading
        gate=threading.Event();original=diagnostics.DiagnosticWriter.run
        def delayed(writer):gate.wait(2.);original(writer)
        with tempfile.TemporaryDirectory() as directory,patch.object(diagnostics.DiagnosticWriter,'run',delayed):
            log=web.EventLog(Path(directory)/'events.jsonl')
            try:
                for i in range(10):
                    s=self.project(self.data);s['pico']['reason']=str(i);log.sample(s)
                self.assertEqual(log.writer.status()['written'],0)
                self.assertGreater(log.writer.status()['queued'],0)
            finally:gate.set();log.close()
            saved=[json.loads(line) for line in log.path.read_text().splitlines()]
            self.assertEqual(saved,log.events)
            self.assertEqual(log.writer.status()['dropped'],0)
            self.assertFalse(json.loads((log.path.parent/'web-log-health.json').read_text())['active'])

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


class ToolDisplayPoseTests(unittest.TestCase):
    def setUp(self):
        import numpy as np
        self.np = np
        self.left = np.eye(4)
        self.left[:3, :3] = [[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]]
        self.left[:3, 3] = [.2, .3, .4]
        self.mount = np.eye(4)
        self.mount[0, 3] = .1
        self.data = dict(device_backend='external_simulation', run_id='run', profile_hash='profile',
                         pico=dict(load_state='empty', teleop_mode='bimanual_tool'),
                         physics=dict(stamp=10., run_id='run', profile_hash='profile', tool_present=False))

    def test_tool_is_hidden_outside_coupled_mode_even_when_physically_present(self):
        for backend in ('external_simulation', 'kinematic', 'snowbot_vendor'):
            for mode in ('independent_arms', None):
                data = dict(self.data, device_backend=backend, pico=dict(teleop_mode=mode),
                            physics=dict(self.data['physics'], tool_present=True))
                with self.subTest(backend=backend, mode=mode):
                    self.assertEqual(web.tool_display_pose(data, self.left, self.mount, 10.1),
                                     (None, 'inactive'))

    def test_unloaded_tool_has_measured_left_frame_preview_in_all_backends(self):
        before = copy.deepcopy(self.data)
        for backend in ('external_simulation', 'kinematic', 'snowbot_vendor'):
            with self.subTest(backend=backend):
                pose, source = web.tool_display_pose(dict(self.data, device_backend=backend),
                                                    self.left, self.mount, 10.1)
                self.np.testing.assert_allclose(pose[:3, 3], [.2, .4, .4])
                self.np.testing.assert_allclose(pose[:3, :3], self.left[:3, :3])
                self.assertEqual(source, 'preview')
        self.assertEqual(self.data, before)

    def test_installed_physical_tool_uses_its_measured_pose_instead_of_fk(self):
        actual = self.np.eye(4)
        actual[:3, 3] = [.9, .8, .7]
        self.data['physics'].update(tool_present=True, tool_pose=actual.ravel().tolist())
        pose, source = web.tool_display_pose(self.data, self.left, self.mount, 10.1)
        self.np.testing.assert_allclose(pose, actual)
        self.assertEqual(source, 'measured')

    def test_stale_missing_or_other_session_physics_does_not_become_a_preview(self):
        for change in (dict(stamp=9.), dict(stamp=11.), dict(run_id='other'),
                       dict(profile_hash='other')):
            data = dict(self.data, physics=dict(self.data['physics'], **change))
            self.assertEqual(web.tool_display_pose(data, self.left, self.mount, 10.1),
                             (None, 'unavailable'))
        self.assertEqual(web.tool_display_pose(dict(self.data, physics={}),
                                              self.left, self.mount, 10.1),
                         (None, 'unavailable'))

    def test_invalid_measured_tool_pose_cannot_fall_back_to_fk(self):
        self.data['physics'].update(tool_present=True, tool_pose=[0.] * 16)
        with self.assertRaisesRegex(ValueError, 'DISPLAY_ERROR_TRANSFORM'):
            web.tool_display_pose(self.data, self.left, self.mount, 10.1)


class SimulationEntryTests(unittest.TestCase):
    def setUp(self):
        environment = patch.dict(entry.os.environ)
        environment.start()
        self.addCleanup(environment.stop)
        entry.os.environ.pop('BINDU_VR_CONFIG', None)

    def test_local_default_recipe_and_explicit_override(self):
        fallback=ROOT/'src/integration/bindu_runtime/config/deployments/snowbot_vr_sim.json'
        with tempfile.TemporaryDirectory() as directory:
            base=Path(directory)
            recipe=json.loads(fallback.read_text())
            recipe['arguments'].update(host='0.0.0.0',cert_file='server.crt',key_file='server.key')
            for name in ('server.crt','server.key'):
                (base/name).touch()
            config=base/'site.json'
            config.write_text(json.dumps(recipe))
            with patch.dict(entry.os.environ,{'BINDU_VR_CONFIG':str(config)}):
                args=entry.parse_args([])
                control,_=entry.commands(args)
                self.assertEqual(args.config,config)
                self.assertEqual(args.host,'0.0.0.0')
                self.assertEqual(Path(args.cert_file),(base/'server.crt').resolve())
                self.assertEqual(Path(args.key_file),(base/'server.key').resolve())
                self.assertIn('host:=0.0.0.0',control)
                explicit=entry.parse_args(['--config',str(fallback)])
                self.assertEqual(explicit.host,'127.0.0.1')
                self.assertEqual(explicit.cert_file,'')

    def test_physics_selects_external_feedback_and_keeps_browser(self):
        args=entry.parse_args(['--physics','--run-id','physics_test'])
        physics,control,display=entry.commands(args)
        self.assertIn('--vr',physics)
        self.assertIn('device_backend:=external_simulation',control)
        self.assertEqual(display[display.index('--device-backend')+1],'external_simulation')
        self.assertIn('--dashboard',display)

    def test_real_recipe_cannot_enable_isaac(self):
        root=ROOT
        with self.assertRaises(SystemExit):
            entry.parse_args(['--physics','--config',str(root/'src/integration/bindu_runtime/config/deployments/snowbot_vr_real.json')])

    def test_device_selection_propagates_to_control_and_display(self):
        root=ROOT
        args=entry.parse_args(['--config',str(root/'src/integration/bindu_runtime/config/deployments/snowbot_vr_real.json')])
        control,display=entry.commands(args)
        self.assertIn('runtime_mode:=real',control)
        self.assertIn('device_backend:=snowbot_vendor',control)
        self.assertIn('command_enabled:=false',control)
        self.assertIn('position_contract:=true',control)
        self.assertNotIn('pico_simulation_demo:=true',control)
        self.assertEqual(display[display.index('--runtime-mode')+1],'real')
        self.assertEqual(display[display.index('--device-backend')+1],'snowbot_vendor')

    def test_explicit_real_write_enable_keeps_preflight_and_recipe(self):
        recipe=ROOT/'src/integration/bindu_runtime/config/deployments/snowbot_vr_real.json'
        before=recipe.read_bytes()
        args=entry.parse_args(['--config',str(recipe),'--enable-commands'])
        self.assertEqual(args.deployment['command_enabled'],'true')
        self.assertIn('command_enabled:=true',entry.commands(args)[0])
        with patch.object(entry,'check',side_effect=RuntimeError('online check rejected')), \
                patch.object(entry.subprocess,'Popen') as start:
            with self.assertRaisesRegex(RuntimeError,'online check rejected'):entry.run(args)
            start.assert_not_called()
        self.assertEqual(recipe.read_bytes(),before)

    def test_write_enable_rejects_simulation_physics_and_load_declaration(self):
        recipe=ROOT/'src/integration/bindu_runtime/config/deployments/snowbot_vr_real.json'
        for argv in (['--enable-commands'],
                     ['--config',str(recipe),'--enable-commands','--physics'],
                     ['--config',str(recipe),'--enable-commands','--declare-load','empty']):
            with self.subTest(argv=argv),patch('sys.stderr'),self.assertRaises(SystemExit):
                entry.parse_args(argv)

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

    def test_partial_start_failure_drains_and_closes_the_shared_process_log_writer(self):
        with tempfile.TemporaryDirectory() as directory:
            args=entry.parse_args(['--output',str(Path(directory)/'run'),
                                   '--namespace','test_'+Path(directory).name])
            child=S(stdout=StringIO('[WARN] [10.0] [servo_node]: observed warning\n'))
            with patch.object(entry,'check'),patch.object(entry.subprocess,'Popen',
                    side_effect=[child,OSError('display could not start')]), \
                    patch.object(entry,'stop_group',return_value=True) as stop:
                with self.assertRaisesRegex(OSError,'display could not start'):entry.run(args)
                stop.assert_called_once_with(child)
            health=json.loads((args.output/'process-log-health.json').read_text())
            self.assertFalse(health['active'])
            self.assertEqual(health['error'],'')
            self.assertTrue(child.stdout.closed)
            self.assertIn('observed warning',(args.output/'control.log').read_text())

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
