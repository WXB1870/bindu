"""Actual Vuer handler/receiver and ROS poll methods, without sockets or ROS.

Vuer 0.0.60 is used to decode/dispatch recorded-timing synthetic controller
events. Only clock, websocket transport, and ROS message/publisher boundaries
are supplied. These tests do not identify the cause of the historical gap.
"""
import ast
import asyncio
import json
from pathlib import Path
import queue
from types import SimpleNamespace as S
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
from bindu_teleoperation.vr.timing import InputPipelineTiming, ControllerTiming
from bindu_teleoperation.vr.receiver import VuerReceiver, _serve
from tools.teleop.snowbot_diagnostics import DiagnosticSampler

ROOT=Path(__file__).resolve().parents[2]
RECORDED_GAP=json.loads((ROOT/'tests/config/teleoperation/snowbot_input_gap.json').read_text())['coordinator_callback_gap']['interval_s']


class TimingSummaries(unittest.TestCase):
    def test_actual_writer_preserves_independent_health_stage_evidence(self):
        from tools.teleop.snowbot_diagnostics import DiagnosticWriter
        stages=('controller_timing','websocket_handler','ros_input_publish',
                'ros_input_publish_duration','input_pipeline')
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);writer=DiagnosticWriter(root/'diagnostics.jsonl')
            for stage in stages:writer.record('vr-health',dict(stage=stage,source_id='source'))
            writer.close()
            rows=[json.loads(line)['data']['stage'] for line in (root/'vr-health.jsonl').read_text().splitlines()]
            self.assertEqual(rows,list(stages))
            self.assertEqual(writer.status()['dropped'],0)

    def test_cli_timing_is_opt_in_and_does_not_change_write_authority_or_recipe(self):
        from tools.teleop.run_vr import commands
        args=S(deployment=dict(robot_config='fixture',runtime_mode='simulation',
                device_backend='kinematic',command_enabled='false'),launch='robot_teleop.launch.py',
                run_id='offline',namespace='fixture',port=8013,output=Path('/tmp/not-created'),
                startup_timeout=30.,diagnostics=True,input_timing=False)
        self.assertNotIn('input_timing_enabled:=true',commands(args)[0])
        args.input_timing=True
        control,_=commands(args)
        self.assertIn('input_timing_enabled:=true',control)
        self.assertIn('command_enabled:=false',control)
        self.assertNotIn('input_timing_enabled',args.deployment)

    def test_coordinator_gap_labels_ros_clock_separately_from_monotonic(self):
        from bindu_contracts.teleoperation import VRFrame
        from bindu_teleoperation.pico import PicoControl,PicoFeedback,PICO_MAPPING
        path=ROOT/'src/integration/bindu_runtime/bindu_runtime/pico_node.py'
        node=next(n for n in ast.parse(path.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='PicoNode')
        method=next(n for n in node.body if getattr(n,'name','')=='on_frame')
        clock=S(ros=100.,mono=10.)
        namespace=dict(time=S(monotonic=lambda:clock.mono),json=json,seconds=lambda x:x,
                       VRFrame=VRFrame,VRInput=lambda **kw:S(**kw),PICO_MAPPING=PICO_MAPPING)
        exec(compile(ast.Module(body=[method],type_ignores=[]),str(path),'exec'),namespace)
        events=[]
        owner=S(matches_mode=lambda m:True,frames={},last_pair_at=None,
                now=lambda:clock.ros,event=lambda *v:events.append(v),
                control=PicoControl(),feedback=lambda:PicoFeedback(clock.ros,True,('closed','closed')),
                last_prepare_check=None,release_until=0.,projected=S(publish=lambda m:None),
                instance_id='offline',position_contract=True,handoff_complete=True)
        def pair(seq):
            for side in ('left','right'):
                message=S(source_id='source',seq=seq,stamp=clock.ros,side=side,
                    pose=tuple(np.eye(4).ravel()),grip=0.,trigger=0.,init=False,stop=False,
                    valid=True,clutch_seq=0,mapping=PICO_MAPPING,buttons=0,
                    button_edges=(0,)*8,virtual_grip=False,runtime_mode='simulation')
                namespace['on_frame'](owner,message)
        pair(1);clock.ros+=RECORDED_GAP;clock.mono+=.02;pair(2)
        data=json.loads(next(v[1] for v in events if v[0]=='PICO_INPUT_DELIVERY_GAP'))
        self.assertEqual(data['interval_clock'],'ros_callback')
        self.assertAlmostEqual(data['interval_s'],RECORDED_GAP)
        self.assertAlmostEqual(data['monotonic_interval_s'],.02)
        clock.mono+=RECORDED_GAP;pair(3)
        data=json.loads([v[1] for v in events if v[0]=='PICO_INPUT_DELIVERY_GAP'][-1])
        self.assertEqual(data['interval_s'],0.)
        self.assertAlmostEqual(data['monotonic_interval_s'],RECORDED_GAP)

    def test_diagnostic_stages_do_not_suppress_each_other_or_clock_anomaly(self):
        sampler=DiagnosticSampler()
        for stage in ('controller_timing','websocket_handler','ros_input_publish',
                      'ros_input_publish_duration','input_pipeline'):
            self.assertTrue(sampler.keep('vr-health',dict(stage=stage),10.))
        self.assertFalse(sampler.keep('vr-health',dict(stage='websocket_handler'),10.01))
        gap=dict(stage='websocket_handler',event_interval_s=RECORDED_GAP)
        self.assertTrue(sampler.keep('vr-health',gap,10.02))
        self.assertFalse(sampler.keep('vr-health',gap,10.03))
        self.assertTrue(sampler.keep('vr-health',dict(stage='websocket_handler'),10.04))
        for i in range(1000):sampler.keep('vr-health',dict(stage=str(i),source_id=str(i)),11.)
        self.assertEqual(len(sampler.last),6)

    def test_no_frames_and_stalled_poll_have_different_evidence(self):
        quiet=InputPipelineTiming()
        for i in range(202):
            quiet.begin(i*.005);report=quiet.finish(i*.005,None)
            if report:quiet_report=report
        self.assertEqual(quiet_report['frame_count'],0)
        self.assertLess(quiet_report['maxima']['ros_poll_interval']['seconds'],.006)
        stalled=InputPipelineTiming();stalled.begin(0.);stalled.finish(0.,None)
        stalled.begin(RECORDED_GAP)
        report=stalled.finish(RECORDED_GAP,None)
        self.assertAlmostEqual(report['maxima']['ros_poll_interval']['seconds'],RECORDED_GAP)

    def test_queue_handler_dispatch_and_publish_are_separate_bounded_metrics(self):
        timing=InputPipelineTiming();timing.begin(11.)
        sample=dict(source_id='source',seq=7,dispatched=10.,handler_started=10.8,
                    frame_ready=10.81,dequeued=11.,shared_monotonic=True)
        report=timing.finish(12.236,sample)
        values=report['maxima']
        self.assertAlmostEqual(values['server_task_dispatch']['seconds'],.8)
        self.assertAlmostEqual(values['controller_handler']['seconds'],.01)
        self.assertAlmostEqual(values['frame_queue_wait']['seconds'],.19)
        self.assertAlmostEqual(values['ros_after_dequeue']['seconds'],1.236)
        self.assertEqual(values['frame_queue_wait']['seq'],7)
        timing.begin(13.3);report=timing.finish(13.3,dict(sample,shared_monotonic=False))
        self.assertNotIn('frame_queue_wait',report['maxima'])
        self.assertNotIn('handler_to_ros_poll_end',report['maxima'])


def actual_poller(clock):
    source=ROOT/'src/integration/bindu_runtime/bindu_runtime/vr_input_node.py'
    node=next(n for n in ast.parse(source.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='VRInputNode')
    methods=[n for n in node.body if getattr(n,'name','') in ('poll','poll_input')]
    namespace=dict(time=clock,stamp=lambda x:x,VRInput=lambda **kw:S(**kw))
    module=ast.Module(body=[ast.ClassDef(name='VRInputNode',bases=[],keywords=[],
                         body=methods,decorator_list=[])],type_ignores=[])
    exec(compile(ast.fix_missing_locations(module),str(source),'exec'),namespace)
    return namespace['VRInputNode']()


class VuerPipeline(unittest.IsolatedAsyncioTestCase):
    async def test_real_vuer_dispatch_to_receiver_and_poll_exposes_publish_stall(self):
        # Patch run before constructing _serve: no listener, browser, ROS, or
        # hardware endpoint is started. Vuer's actual generator dispatches.
        from vuer import Vuer
        from vuer.events import ClientEvent
        import msgpack
        apps=[];output=queue.Queue(2);feedback=queue.Queue(1);health=queue.Queue(128)
        clock=S(value=10.,time=lambda:100.,monotonic=lambda:clock.value)
        config=dict(host='127.0.0.1',port=0,cert_file=None,key_file=None,
                    side='both',mapping='pico_snowbot_v1',input_timing_enabled=True)
        with patch.object(Vuer,'run',lambda app:apps.append(app)),patch(
                'bindu_teleoperation.vr.receiver.time',clock):
            _serve(config,output,feedback,health)
            self.assertTrue(apps,list(output.queue))
            app=apps[0];app.ws['offline']=S()
            session=app._proxy('offline')
            generator=app.bound_fn(session)
            await generator.__anext__()
            state=dict(squeezeValue=0.,triggerValue=0.,aButton=False,bButton=False,thumbstick=False)
            value=dict(left=np.eye(4).ravel().tolist(),right=np.eye(4).ravel().tolist(),
                       leftState=state,rightState=state)
            wire=msgpack.packb(dict(etype='CONTROLLER_MOVE',value=value),use_bin_type=True)
            await generator.asend(ClientEvent(**msgpack.unpackb(wire,raw=False)))
            await asyncio.sleep(0)
            receiver=VuerReceiver.__new__(VuerReceiver)
            receiver.output=output;receiver.process=S(is_alive=lambda:True)
            receiver.poll_health=lambda:[]
            node=actual_poller(clock)
            node.receiver=receiver;node.last_poll=None;node.poll_delay_max=0.
            node.last_frame=None;node.input_clock_offset=None
            node.run_id,node.runtime_mode='offline','simulation'
            node.now=lambda:100.
            node.input_timing=InputPipelineTiming()
            published=[];reports=[]
            def publish(msg):published.append(msg);clock.value+=RECORDED_GAP/2
            node.publisher=S(publish=publish);node.emit_health=reports.append
            clock.value=10.2
            node.poll()
            summary=next(r for r in reports if r['stage']=='input_pipeline')
            self.assertAlmostEqual(summary['maxima']['ros_poll_callback']['seconds'],RECORDED_GAP)
            self.assertAlmostEqual(summary['maxima']['frame_queue_wait']['seconds'],.2)
            self.assertEqual([m.stamp for m in published],[100.,100.])
            self.assertEqual(summary['published_messages'],2)
            self.assertIn('server_task_dispatch',summary['maxima'])
            await generator.aclose()
            await app.app.cleanup()

    async def test_disabled_timing_keeps_two_field_frame_envelope(self):
        # ParamsProto registers class CLI names globally. Production _serve is
        # one spawned process per receiver; use the same isolation here.
        import sys,os
        program='''import asyncio,queue,numpy as np
from types import SimpleNamespace as S
from unittest.mock import patch
from vuer import Vuer
from vuer.events import ClientEvent
from bindu_teleoperation.vr.receiver import _serve
async def run():
    apps=[];output=queue.Queue(2)
    with patch.object(Vuer,'run',lambda app:apps.append(app)):
        _serve(dict(host='127.0.0.1',port=0,cert_file=None,key_file=None,side='left',mapping='v34'),output,queue.Queue(1),queue.Queue(128))
    app=apps[0];app.ws['offline']=S();session=app._proxy('offline')
    handler=next(iter(app.handlers['CONTROLLER_MOVE'].values()))
    await handler(ClientEvent(etype='CONTROLLER_MOVE',value=dict(left=np.eye(4).ravel().tolist(),leftState=dict(squeezeValue=0.,triggerValue=0.))),session)
    assert len(output.get_nowait())==2
    await app.app.cleanup()
asyncio.run(run())
'''
        env=dict(os.environ);env['PYTHONPATH']=str(ROOT)+os.pathsep+os.pathsep.join(
            str(p) for p in (ROOT/'src').glob('*/*') if p.is_dir())
        process=await asyncio.create_subprocess_exec(sys.executable,'-c',program,
            env=env,stdout=asyncio.subprocess.PIPE,stderr=asyncio.subprocess.STDOUT)
        output,_=await process.communicate()
        self.assertEqual(process.returncode,0,output.decode())


if __name__=='__main__':unittest.main()
