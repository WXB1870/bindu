import json
import shutil
import subprocess
import unittest
from pathlib import Path

from bindu_teleoperation.vr.timing import ControllerTiming

SCRIPT = Path(__file__).resolve().parents[1]/'src/capabilities/bindu_teleoperation/bindu_teleoperation/vr/client_timing.js'


def sample(perf, seq=1):
    return dict(version=1, seq=seq, perf_ms=perf, wall_ms=123456., buffered_bytes=0,
                loop_max_ms=0., raf_max_ms=0., xr_max_ms=0., long_task_max_ms=0.,
                long_task_count=0, page_visibility='visible', xr_visibility='visible')


class ControllerTimingTests(unittest.TestCase):
    def test_blocked_diagnostic_dds_publish_does_not_block_input_report_enqueue(self):
        import queue
        import threading
        from types import SimpleNamespace as S
        from unittest.mock import Mock,patch
        from bindu_runtime.vr_input_node import VRInputNode,_publish_health
        from bindu_teleoperation.vr.receiver import _latest_put
        entered=threading.Event();release=threading.Event();delivered=[]
        def publish(msg):
            if not delivered:entered.set();release.wait(2.)
            delivered.append(json.loads(msg.data))
        node=Mock();node.create_publisher.return_value=S(publish=publish)
        fake_ros=S(init=Mock(),create_node=Mock(return_value=node),try_shutdown=Mock(),
                   executors=S(ExternalShutdownException=RuntimeError))
        pending=queue.Queue(maxsize=2)
        owner=S(health_reports=pending,health_dropped=0,health_failed=False,
                health_process=S(is_alive=lambda:True))
        with patch.dict('sys.modules',rclpy=fake_ros):
            worker=threading.Thread(target=_publish_health,args=('/test/health',pending))
            worker.start()
            try:
                VRInputNode.emit_health(owner,dict(seq=0))
                self.assertTrue(entered.wait(2.))
                for seq in range(1,101):VRInputNode.emit_health(owner,dict(seq=seq))
                self.assertEqual(owner.health_dropped,98)
                self.assertEqual(pending.qsize(),2)
            finally:
                release.set();_latest_put(pending,None);worker.join(3.)
        self.assertFalse(worker.is_alive())
        self.assertEqual(delivered[-1]['seq'],100)
        node.destroy_node.assert_called_once()

    def test_browser_stall_and_transport_stall_have_different_evidence(self):
        probe = ControllerTiming()
        probe.observe(sample(1000.), 50., 10., 'source', 1)
        browser = probe.observe(sample(1180., 2), 50.18, 10.18, 'source', 2)
        self.assertAlmostEqual(browser['client_controller_interval_s'], .18)
        self.assertAlmostEqual(browser['transport_interval_excess_s'], 0.)
        network = probe.observe(sample(1210., 3), 50.36, 10.36, 'source', 3)
        self.assertAlmostEqual(network['client_controller_interval_s'], .03)
        self.assertAlmostEqual(network['transport_interval_excess_s'], .15)

    def test_missing_malformed_clock_reset_and_reconnect_do_not_fake_latency(self):
        probe = ControllerTiming()
        for i, value in enumerate((None, {}, dict(sample(1.), perf_ms=float('nan')), sample(1000.))):
            report = probe.observe(value, 50.+i, 10.+i, 'source', i)
            self.assertEqual(report['client_timing_available'], i == 3)
            self.assertNotIn('transport_interval_excess_s', report)
        reset = probe.observe(sample(10.), 55., 15., 'source', 5)
        self.assertNotIn('transport_interval_excess_s', reset)
        reconnect = probe.observe(sample(20.), 56., 16., 'new-source', 6)
        self.assertNotIn('server_controller_interval_s', reconnect)

    @unittest.skipUnless(shutil.which('node'), 'Node required for browser wire test')
    def test_browser_hook_preserves_actual_msgpack_payload_and_records_gap(self):
        import msgpack
        event = dict(etype='CONTROLLER_MOVE', value={'right': [0.]*16}, key='motionControllers')
        inputs = [msgpack.packb(event, use_bin_type=True)]
        # Exercise all legal root-map headers, and the fixmap-to-map16 boundary.
        inputs += [b'\xde\x00\x03'+inputs[0][1:], b'\xdf\x00\x00\x00\x03'+inputs[0][1:],
                   msgpack.packb({str(i): i for i in range(15)})]
        harness = r'''
const fs=require('fs'), vm=require('vm'); let now=1000, packets=[];
class Socket { constructor(){this.bufferedAmount=42;} send(b){packets.push(Buffer.from(b).toString('base64'));} }
const ctx={TextEncoder,Uint8Array,DataView,ArrayBuffer,WeakMap,WebSocket:Socket,
  performance:{now:()=>now}, Date, document:{visibilityState:'visible'},setInterval:()=>{},
  window:{requestAnimationFrame:()=>{}}};
vm.createContext(ctx);vm.runInContext(fs.readFileSync(process.argv[1],'utf8'),ctx);
const socket=new Socket();
for(const data of JSON.parse(process.argv[2])){socket.send(Uint8Array.from(Buffer.from(data,'base64')));now+=180;}
process.stdout.write(JSON.stringify(packets));
'''
        import base64
        result = subprocess.run(['node', '-e', harness, str(SCRIPT),
            json.dumps([base64.b64encode(b).decode() for b in inputs])], check=True, capture_output=True, text=True)
        for i, packed in enumerate(json.loads(result.stdout)):
            actual = msgpack.unpackb(base64.b64decode(packed), raw=False)
            timing = actual.pop('bindu_timing')
            self.assertEqual(actual, msgpack.unpackb(inputs[i], raw=False))
            self.assertEqual(timing['seq'], i+1)
            self.assertEqual(timing['buffered_bytes'], 42)
            self.assertEqual(timing['send_interval_ms'], 180 if i else None)


if __name__ == '__main__':
    unittest.main()
