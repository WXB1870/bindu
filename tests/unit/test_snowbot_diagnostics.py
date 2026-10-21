"""Diagnostic capture must preserve payloads, drain on exit and expose loss."""
from tests.support.paths import ROOT
import importlib.util
import json
from pathlib import Path
import tempfile
import threading
from types import SimpleNamespace as S
import unittest
from unittest.mock import patch

spec=importlib.util.spec_from_file_location('snowbot_diagnostics',ROOT/'tools/teleop/snowbot_diagnostics.py')
diagnostics=importlib.util.module_from_spec(spec)
spec.loader.exec_module(diagnostics)


class DiagnosticTests(unittest.TestCase):
    def test_rotation_retains_order_and_shutdown_drains_all_streams(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);writer=diagnostics.DiagnosticWriter(root/'diagnostics.jsonl',chunk_bytes=180,sampling=False)
            for i in range(50):
                writer.record('vr-raw',dict(seq=i,pose=[1.,float('nan')]))
                writer.record('execution-state',dict(seq=i))
            writer.close()
            for stream in ('vr-raw','execution-state'):
                files=[root/(stream+'.jsonl'),*sorted(root.glob(stream+'-*.jsonl'))]
                rows=[json.loads(line) for path in files for line in path.read_text().splitlines()]
                self.assertEqual([r['data']['seq'] for r in rows],list(range(50)))
                self.assertTrue(all(r['received_monotonic_ns']>0 for r in rows))
            self.assertEqual(json.loads((root/'vr-raw.jsonl').read_text().splitlines()[0])['data']['pose'][1],'nan')
            self.assertEqual(writer.status()['written'],100)
            self.assertFalse(json.loads((root/'diagnostics-health.json').read_text())['active'])

    def test_full_queue_does_not_block_producer_and_reports_loss(self):
        gate=threading.Event();original=diagnostics.DiagnosticWriter.run
        def delayed(writer):
            gate.wait(2.);original(writer)
        with tempfile.TemporaryDirectory() as directory, patch.object(diagnostics.DiagnosticWriter,'run',delayed):
            writer=diagnostics.DiagnosticWriter(Path(directory)/'diagnostics.jsonl',capacity=1)
            try:
                writer.record('test',1);writer.record('test',2)
                self.assertEqual(writer.status()['dropped'],1)
            finally:gate.set();writer.close()
            self.assertEqual(writer.status()['written'],1)

    def test_write_failure_is_visible_and_never_reported_as_recording(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);(root/'bad.jsonl').mkdir()
            writer=diagnostics.DiagnosticWriter(root/'diagnostics.jsonl')
            writer.record('bad',{})
            writer.close()
            self.assertIn('IsADirectoryError',writer.status()['error'])
            self.assertFalse(writer.status()['active'])

    def test_ros_fixed_arrays_and_nan_are_serializable(self):
        from bindu_interfaces.msg import VRInput
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'diagnostics.jsonl';writer=diagnostics.DiagnosticWriter(path)
            writer.record('diagnostics',VRInput(seq=12,pose=[float('nan')]+[0.]*15),ros=True)
            writer.close()
            self.assertEqual(writer.status()['error'],'')
            data=json.loads(path.read_text())['data']
            self.assertEqual(data['seq'],12);self.assertEqual(data['pose'][0],'nan')

    def test_real_pdo_fixed_arrays_keep_integer_types_and_write_successfully(self):
        from snowbot_ctl_msgs.msg import PositionObservation
        from tools.teleop.snowbot_pdo import observation_payload, PdoMonitor
        msg=PositionObservation(boot_id=2**64-1,pdo_sample=1,pdo_age=0.,
            wkc=67,expected_wkc=67,valid_mask=(1<<20)-1,status=[0x40]*20)
        msg.header.stamp.sec=10
        payload=observation_payload(msg)
        self.assertTrue(all(type(value) is int for value in payload['status']))
        health=PdoMonitor().receive(payload,10.,1.)
        self.assertNotIn('PDO_STATUSWORD_INVALID',health['reasons'])
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'pdo-observation.jsonl'
            writer=diagnostics.DiagnosticWriter(path)
            writer.record('diagnostics',dict(observation=payload));writer.close()
            self.assertEqual(writer.status()['error'],'')
            self.assertEqual(writer.status()['written'],1)
            saved=json.loads(path.read_text())['data']['observation']
            self.assertEqual(saved['boot_id'],2**64-1)
            self.assertEqual(saved['status'],[0x40]*20)


class SamplingTests(unittest.TestCase):
    def test_regular_input_is_capped_per_hand_before_ros_serialization(self):
        sampler=diagnostics.DiagnosticSampler()
        messages={side:S(side=side,source_id='vr',valid=True,button_edges=[0]*8,
                         grip=.8,trigger=0.) for side in ('left','right')}
        kept={side:0 for side in messages}
        for tick in range(1000):
            for side,msg in messages.items():
                msg.seq=tick
                kept[side]+=sampler.keep('vr-raw',msg,tick*.001)
        self.assertTrue(all(9 <= count <= 10 for count in kept.values()),kept)
        self.assertEqual(sampler.sampled_out,2000-sum(kept.values()))

    def test_edges_release_reconnect_invalid_and_clock_reversal_are_kept(self):
        sampler=diagnostics.DiagnosticSampler()
        msg=S(side='left',source_id='vr',valid=True,button_edges=[0]*8,grip=.8,trigger=0.)
        self.assertTrue(sampler.keep('vr-raw',msg,1.))
        self.assertFalse(sampler.keep('vr-raw',msg,1.001))
        for tick,change in enumerate((dict(button_edges=[1]+[0]*7),dict(grip=.2),
                dict(valid=False),dict(source_id='reconnected'),dict(trigger=.9))):
            msg.__dict__.update(change)
            self.assertTrue(sampler.keep('vr-raw',msg,1.002+tick*.001))
        self.assertTrue(sampler.keep('vr-raw',msg,.1))
        for i in range(1000):
            msg.source_id=str(i);sampler.keep('vr-raw',msg,2.+i*.001)
        self.assertEqual(len(sampler.last),1)

    def test_health_is_one_hz_with_immediate_fault_and_recovery(self):
        sampler=diagnostics.DiagnosticSampler()
        value=dict(valid=True,reasons=[],fault_mask=0)
        self.assertTrue(sampler.keep('pdo-health',value,0.))
        self.assertFalse(sampler.keep('pdo-health',value,.5))
        self.assertTrue(sampler.keep('pdo-health',dict(value,fault_mask=1),.501))
        self.assertTrue(sampler.keep('pdo-health',value,.502))
        self.assertFalse(sampler.keep('pdo-health',value,1.))
        self.assertTrue(sampler.keep('pdo-health',value,1.503))

    def test_feedback_command_ids_do_not_bypass_rate_and_stop_changes_are_kept(self):
        sampler=diagnostics.DiagnosticSampler()
        value=dict(instance_id='execution',state='RUNNING',code='OK',stop_failures=[])
        self.assertTrue(sampler.keep('execution-state',value,1.))
        self.assertFalse(sampler.keep('execution-state',dict(value,command_id='new',revision=20),1.001))
        self.assertTrue(sampler.keep('execution-state',dict(value,stop_failures=['STOP_UNCONFIRMED']),1.002))

    def test_continuous_qp_diagnostics_are_sampled_but_command_mapping_is_full(self):
        sampler=diagnostics.DiagnosticSampler()
        value=S(state='BIMANUAL_QP_RESULT',code='{"code":"OK","elapsed":0.01}')
        self.assertTrue(sampler.keep('runtime-events',value,1.))
        value.code='{"code":"OK","elapsed":0.02}'
        self.assertFalse(sampler.keep('runtime-events',value,1.001))
        value.code='{"code":"FAILED"}'
        self.assertTrue(sampler.keep('runtime-events',value,1.002))
        value.state='SNOWBOT_SEGMENT'
        self.assertTrue(all(sampler.keep('runtime-events',value,1.003) for _ in range(100)))
        self.assertTrue(all(sampler.keep('accepted-commands',{},1.003) for _ in range(100)))

    def test_writer_exposes_intentional_sampling_separately_from_queue_loss(self):
        with tempfile.TemporaryDirectory() as directory:
            writer=diagnostics.DiagnosticWriter(Path(directory)/'diagnostics.jsonl')
            with patch.object(diagnostics.time,'monotonic',return_value=1.):
                for i in range(100):writer.record('vr-raw',dict(side='left',seq=i))
            writer.close()
            self.assertEqual(writer.status()['sampled_out'],99)
            self.assertEqual(writer.status()['written'],1)
            self.assertEqual(writer.status()['dropped'],0)
            health=json.loads((Path(directory)/'diagnostics-health.json').read_text())
            self.assertEqual(health['sampled_out'],99)
