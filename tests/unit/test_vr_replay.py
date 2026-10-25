"""Full observer capture and offline replay boundaries, without ROS or hardware."""
from tests.support.paths import ROOT
import contextlib
import copy
from dataclasses import asdict
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
from types import SimpleNamespace as S
import unittest
from unittest.mock import patch

from bindu_contracts.teleoperation import VRFrame
from tests.replay.replay_vr import dataset, frame, input_audit, records
from tools.teleop.snowbot_diagnostics import DiagnosticWriter, VR_INPUT_STREAMS


POSE=(1.,0.,0.,0.,0.,1.,0.,0.,0.,0.,1.,0.,0.,0.,0.,1.)


def packet(seq, side, at=100., source='headset'):
    value=asdict(VRFrame(source,seq,at,side,POSE,.8,.9,False,False,True,2,
        'pico_snowbot_v1',5,(1,0,0,1,1,0,2,1),True,(.4,-.2),POSE,at-.02,.01,source,3))
    value['stamp']={'sec':int(at),'nanosec':round((at-int(at))*1e9)}
    value['head_stamp']={'sec':int(at-.02),'nanosec':round(((at-.02)%1)*1e9)}
    return value


class FullInputCaptureTests(unittest.TestCase):
    def capture(self, root, sequences=(1,2,3)):
        writer=DiagnosticWriter(root/'diagnostics.jsonl',full_streams=VR_INPUT_STREAMS,chunk_bytes=600)
        for seq in sequences:
            for side in ('left','right'):writer.record('vr-raw',packet(seq,side,100.+seq*.01))
            writer.record('vr-projected',packet(seq,'left',100.+seq*.01))
        writer.close()
        return writer

    def test_full_capture_rotates_drains_and_preserves_every_field(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);writer=self.capture(root,range(1,61))
            audit=input_audit(root,require_full=True)
            self.assertEqual(audit['messages'],120)
            self.assertEqual(audit['paired_events'],60)
            self.assertEqual(audit['sequence_gaps'],[])
            rows=list(records(root/'vr-raw.jsonl',strict=True))
            self.assertEqual(rows[0],json.loads(json.dumps(packet(1,'left',100.01))))
            self.assertEqual(rows[-1]['seq'],60)
            self.assertEqual(writer.status()['input_recording']['written']['vr-projected'],60)
            self.assertFalse(audit['upstream_complete'])

    def test_opt_in_leaves_other_diagnostics_sampled_and_default_unchanged(self):
        for full in ((),VR_INPUT_STREAMS):
            with self.subTest(full=bool(full)),tempfile.TemporaryDirectory() as directory:
                root=Path(directory);writer=DiagnosticWriter(root/'diagnostics.jsonl',full_streams=full)
                with patch('tools.teleop.snowbot_diagnostics.time.monotonic',return_value=1.):
                    for seq in range(50):
                        writer.record('vr-raw',packet(seq,'left'))
                        writer.record('execution-state',dict(instance_id='same',state='IDLE'))
                writer.close()
                self.assertEqual(len(list(records(root/'vr-raw.jsonl'))),50 if full else 1)
                self.assertEqual(len(list(records(root/'execution-state.jsonl'))),1)

    def test_sequence_gaps_are_exposed_without_inventing_frames(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);self.capture(root,(1,5))
            audit=input_audit(root,require_full=True)
            self.assertEqual(audit['paired_events'],2)
            self.assertEqual([g['missing'] for g in audit['sequence_gaps']],[3,3])

    def test_full_capture_rejects_loss_running_writer_corruption_and_missing_side(self):
        for fault in ('drop','open','count','truncated','side'):
            with self.subTest(fault=fault),tempfile.TemporaryDirectory() as directory:
                root=Path(directory);self.capture(root)
                path=root/'diagnostics-health.json';health=json.loads(path.read_text())
                if fault=='drop':health['input_recording']['dropped']['vr-projected']=1
                if fault=='open':health['input_recording']['closed']=False
                if fault=='count':health['input_recording']['received']['vr-raw']+=1
                if fault=='truncated':
                    with (root/'vr-raw.jsonl').open('a') as output:output.write('{truncated')
                if fault=='side':
                    chunks=sorted(root.glob('vr-raw*.jsonl'));chunks[-1].unlink()
                path.write_text(json.dumps(health))
                with self.assertRaises(ValueError):input_audit(root,require_full=True)

    def test_actual_queue_overflow_is_reported_and_rejected(self):
        gate=threading.Event();run=DiagnosticWriter.run
        def delayed(writer):gate.wait(2.);run(writer)
        with tempfile.TemporaryDirectory() as directory,patch.object(DiagnosticWriter,'run',delayed):
            root=Path(directory);writer=DiagnosticWriter(root/'diagnostics.jsonl',capacity=1,full_streams=VR_INPUT_STREAMS)
            try:
                writer.record('vr-raw',packet(1,'left'))
                writer.record('vr-raw',packet(1,'right'))
            finally:gate.set();writer.close()
            self.assertEqual(writer.status()['input_recording']['dropped']['vr-raw'],1)
            with self.assertRaisesRegex(ValueError,'INCOMPLETE'):input_audit(root,require_full=True)

    def test_offline_cli_inspection_works_without_ros_solver_or_network(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);self.capture(root)
            result=subprocess.run([sys.executable,'-m','tests.replay.replay_vr','--log',str(root),
                '--output',str(root/'inspection.json'),'--require-full-inputs','--inspect-inputs'],
                cwd=ROOT,capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)
            self.assertEqual(json.loads((root/'inspection.json').read_text())['paired_events'],3)


class ReplayPayloadTests(unittest.TestCase):
    def test_frame_keeps_identity_buttons_and_relative_head_time(self):
        raw=packet(88,'right',100.);saved=copy.deepcopy(raw);value=frame(raw,2.)
        self.assertEqual(raw,saved)
        self.assertEqual((value.source_id,value.seq,value.trigger,value.buttons),('headset',88,.9,5))
        self.assertEqual(value.button_edges,(1,0,0,1,1,0,2,1))
        self.assertEqual(value.stick,(.4,-.2));self.assertTrue(value.virtual_grip)
        self.assertEqual(value.head_pose,POSE);self.assertEqual(value.head_seq,3)
        self.assertAlmostEqual(value.head_stamp,1.98)
        self.assertEqual(value.head_age_s,.01)

    def test_legacy_capture_is_labelled_and_strict_full_mode_refuses_it(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);writer=DiagnosticWriter(root/'diagnostics.jsonl',sampling=False)
            for side in ('left','right'):writer.record('vr-raw',packet(1,side))
            writer.close()
            self.assertEqual(input_audit(root)['mode'],'legacy_sampled_or_unknown')
            with self.assertRaisesRegex(ValueError,'FULL_INPUTS_REQUIRED'):input_audit(root,require_full=True)

    def test_reconnect_order_duplicates_and_same_time_button_frames(self):
        for sequences,error in (((1,1),'DUPLICATE'),((2,1),'ORDER'),((1,2),None)):
            with self.subTest(sequences=sequences),tempfile.TemporaryDirectory() as directory:
                root=Path(directory);writer=DiagnosticWriter(root/'diagnostics.jsonl',full_streams=VR_INPUT_STREAMS)
                for seq in sequences:
                    for side in ('left','right'):writer.record('vr-raw',packet(seq,side))
                writer.close()
                if error:
                    with self.assertRaisesRegex(ValueError,error):input_audit(root)
                else:self.assertEqual(input_audit(root)['paired_events'],2)
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);writer=DiagnosticWriter(root/'diagnostics.jsonl',full_streams=VR_INPUT_STREAMS)
            for source,at in (('first',100.),('reconnect',99.)):
                for side in ('left','right'):writer.record('vr-raw',packet(1,side,at,source))
            writer.close()
            self.assertEqual(input_audit(root)['sources'],['first','reconnect'])

    def test_capture_source_is_exact_and_legacy_ambiguity_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            with (root/'vr-raw.jsonl').open('w') as output:
                for source in ('first','other'):
                    for side in ('left','right'):
                        output.write(json.dumps(dict(data=packet(1,side,100.,source)))+'\n')
            (root/'vr-health.jsonl').write_text('')
            capture=dict(input_seq=1,input_stamp=100.,input_source_id='first')
            def event():
                (root/'runtime-events.jsonl').write_text(json.dumps(dict(data=dict(stamp=100.,
                    state='BIMANUAL_CAPTURED',code=json.dumps(capture))))+'\n')
            event();segments,_=dataset(root,101.)
            self.assertEqual(segments[0]['pairs'][0][0]['source_id'],'first')
            del capture['input_source_id'];event()
            with self.assertRaisesRegex(ValueError,'SOURCE_AMBIGUOUS'):dataset(root,101.)


class CaptureCliTests(unittest.TestCase):
    def test_replay_cli_reports_a_failed_segment_with_nonzero_exit_status(self):
        from tests.replay import replay_vr
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);FullInputCaptureTests().capture(root)
            capture=dict(input_seq=1,input_stamp=100.01,input_source_id='headset')
            (root/'runtime-events.jsonl').write_text(json.dumps(dict(data=dict(stamp=100.01,
                state='BIMANUAL_CAPTURED',code=json.dumps(capture))))+'\n')
            (root/'vr-health.jsonl').write_text('')
            config=root/'tool.json';config.write_text('{}')
            robot=S(tool_config=config,urdf=root/'model.urdf',profile=S(max_speed=1.,joint_dynamics={}))
            modules={'ament_index_python.packages':S(get_package_prefix=lambda name:'/offline'),
                     'bindu_runtime.robot_setup':S(RobotSetup=S(load=lambda path:robot))}
            arguments=['replay','--log',str(root),'--output',str(root/'result.json'),'--require-full-inputs']
            with patch.dict(sys.modules,modules),patch.object(sys,'argv',arguments),\
                    patch.object(replay_vr,'replay',return_value=dict(failure='RECORDED_INPUT_TIMEOUT')),\
                    contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(replay_vr.main(),1)
            self.assertEqual(json.loads((root/'result.json').read_text())['clock'],'received')

    def test_actual_observer_routes_raw_projected_and_timing_without_sampling(self):
        from tools.teleop.snowbot_live import observe
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);model=root/'model.urdf';model.write_text('offline fixture')
            config=root/'tool.json';config.write_text('{}')
            robot=S(tool_config=config,urdf=model,display_urdf=model,
                    profile=S(digest='profile',limits={'joint':(-1.,1.)}))
            subscriptions={};alive=iter((True,False));snapshots=[]
            node=S(create_subscription=lambda cls,topic,callback,qos:subscriptions.setdefault(topic,callback),
                   destroy_node=lambda:None)
            def spin_once(*args,**kwargs):
                for seq in range(1,51):
                    for side in ('left','right'):
                        for topic in ('raw','input'):
                            subscriptions['/fixture/teleop/vr/'+topic](packet(seq,side,100.+seq*.01))
                    subscriptions['/fixture/teleop/vr/health'](S(data=json.dumps(dict(
                        run_id='run',stage='controller_timing',source_id='headset',seq=seq))))
            modules={'rclpy':S(init=lambda **kw:None,create_node=lambda name:node,ok=lambda:next(alive),
                    spin_once=spin_once,try_shutdown=lambda:None,executors=S(ExternalShutdownException=RuntimeError)),
                'rclpy.signals':S(SignalHandlerOptions=S(NO=0,ALL=1)),
                'std_msgs.msg':S(String=object),
                'bindu_interfaces.msg':S(**{k:object for k in ('ExecutionState','RuntimeEvent','VRInput',
                    'MotionCommand','GripperState','TeleopModeState')}),
                'bindu_runtime.common':S(EVENT_QOS=object()),
                'bindu_runtime.robot_setup':S(RobotSetup=S(load=lambda path:robot)),
                'tools.teleop.snowbot_errors':S(EndEffectorErrors=lambda *a:S(sample=lambda *a:{}),pinocchio_fk=lambda *a:None),
                'rosidl_runtime_py.convert':S(message_to_ordereddict=lambda message:message)}
            args=S(robot_config=config,run_id='run',runtime_mode='simulation',device_backend='kinematic',
                state_file=None,trace_file=root/'diagnostics.jsonl',namespace='fixture',duration=0.,record_inputs=True)
            with patch.dict(sys.modules,modules),contextlib.redirect_stdout(io.StringIO()):
                observe(args,snapshots.append)
            self.assertEqual(input_audit(root,require_full=True)['messages'],100)
            self.assertEqual(len(list(records(root/'vr-projected.jsonl'))),100)
            self.assertEqual(len(list(records(root/'vr-health.jsonl'))),50)

    def test_record_flag_only_routes_to_observer_and_keeps_command_authority(self):
        from tools.teleop.run_vr import commands
        args=S(deployment=dict(robot_config='fixture',runtime_mode='real',device_backend='snowbot_vendor',
                command_enabled='false'),launch='robot_teleop.launch.py',run_id='offline',namespace='fixture',
                port=8013,output=Path('/tmp/not-created'),startup_timeout=30.,diagnostics=True)
        control,display=commands(args)
        self.assertNotIn('--record-inputs',display)
        args.record_inputs=True
        full_control,display=commands(args)
        self.assertEqual(full_control,control)
        self.assertIn('command_enabled:=false',full_control)
        self.assertIn('--record-inputs',display)

    def test_recording_cannot_silently_run_without_diagnostics(self):
        from tools.teleop.run_vr import parse_args
        with contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit) as exc:
            parse_args(['--record-inputs','--no-diagnostics'])
        self.assertEqual(exc.exception.code,2)


class ReplayMappingTests(unittest.TestCase):
    def test_actual_sessions_keep_targets_heading_and_freshness_after_clock_rebase(self):
        import numpy as np
        from dataclasses import replace
        from bindu_teleoperation.bimanual_session import BimanualSession
        from bindu_teleoperation.independent_session import IndependentArmsSession
        cfg=json.loads((ROOT/'src/integration/bindu_runtime/config/robots/snowbot/bimanual_snowbot_real.json').read_text())
        for cls in (BimanualSession,IndependentArmsSession):
            with self.subTest(session=cls.__name__):
                targets=[];headings=[]
                for offset in (0.,-98.):
                    session=cls(cfg,1,-10.)
                    for seq,at,grip,move in ((10,100.,0.,0.),(11,100.02,.8,0.),
                                             (12,101.1,.8,0.),(13,101.12,.8,.01)):
                        pair=[]
                        for side in ('left','right'):
                            raw=packet(seq,side,at)
                            pose=list(POSE);pose[3]=-.17 if side=='left' else .17;pose[11]=-move
                            raw.update(pose=pose,grip=grip,virtual_grip=False)
                            pair.append(frame(raw,at+offset))
                        session.ingest(*pair,at+offset)
                        if seq==12:
                            self.assertEqual(session.mode,'follow')
                            session.capture(np.array([np.eye(4),np.eye(4)]) if cls is IndependentArmsSession else np.eye(4))
                    targets.append(session.target.copy())
                    headings.append([h.mapping_yaw for h in session.hands] if cls is IndependentArmsSession else session.mapping_yaw)
                    with self.assertRaises(ValueError):
                        session.ingest(*[replace(f,seq=14) for f in pair],101.12+offset+.26)
                np.testing.assert_allclose(targets[0],targets[1],atol=1e-12)
                np.testing.assert_allclose(headings[0],headings[1],atol=1e-12)


if __name__=='__main__':unittest.main()
