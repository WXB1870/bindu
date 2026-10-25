from tests.support.paths import ROOT
import json
import shutil
import subprocess
import unittest
from pathlib import Path

from bindu_teleoperation.vr.timing import ControllerTiming

SCRIPT = ROOT/'src/capabilities/bindu_teleoperation/bindu_teleoperation/vr/client_timing.js'


def sample(perf, seq=1):
    return dict(version=1, seq=seq, perf_ms=perf, wall_ms=123456., buffered_bytes=0,
                loop_max_ms=0., raf_max_ms=0., xr_max_ms=0., long_task_max_ms=0.,
                long_task_count=0, page_visibility='visible', xr_visibility='visible')


class VRInputClockTests(unittest.TestCase):
    def owner(self, wall, delays):
        from types import SimpleNamespace as S
        from unittest.mock import Mock
        from bindu_contracts.teleoperation import VRFrame
        from bindu_teleoperation.pico import PICO_MAPPING
        pose=(1.,0.,0.,0.,0.,1.,0.,0.,0.,0.,1.,0.,0.,0.,0.,1.)
        packets=[];published=[]
        owner=S(last_poll=None,poll_delay_max=0.,last_frame=None,input_clock_offset=None,
                run_id='clock-test',runtime_mode='simulation',emit_health=Mock(),
                receiver=S(poll_health=lambda:[],poll=lambda:packets.pop(0)),
                publisher=S(publish=published.append),now=Mock(side_effect=lambda:wall[0]+next(delays)))
        def pair(seq,at):
            return tuple(VRFrame('headset',seq,at,side,pose,0.,0.,mapping=PICO_MAPPING)
                         for side in ('left','right'))
        return owner,packets,published,pair

    def test_burst_events_keep_one_pair_stamp_and_original_intervals_despite_clock_read_jitter(self):
        from unittest.mock import patch
        from bindu_runtime.vr_input_node import VRInputNode
        from bindu_runtime.common import seconds
        wall=[1000.01]
        owner,packets,published,pair=self.owner(wall,iter((.000630,.000005,.000005,.000005)))
        with patch('bindu_runtime.vr_input_node.time.time',side_effect=lambda:wall[0]):
            packets.append(pair(6719,1000.));VRInputNode.poll(owner)
            wall[0]=1000.015
            packets.append(pair(6720,1000.000560));VRInputNode.poll(owner)
        stamps=[seconds(m.stamp) for m in published]
        self.assertEqual(stamps[0],stamps[1])
        self.assertEqual(stamps[2],stamps[3])
        self.assertAlmostEqual(stamps[2]-stamps[0],.000560,places=6)
        owner.now.assert_called_once()

    def test_original_stale_future_and_reversed_event_times_are_not_refreshed_or_clamped(self):
        from unittest.mock import patch
        from bindu_runtime.vr_input_node import VRInputNode
        from bindu_runtime.common import seconds
        wall=[1000.01]
        owner,packets,published,pair=self.owner(wall,iter((-950.,)))
        with patch('bindu_runtime.vr_input_node.time.time',side_effect=lambda:wall[0]):
            packets.append(pair(1,1000.));VRInputNode.poll(owner)
            packets.append(pair(2,999.9999));VRInputNode.poll(owner)
            self.assertLess(seconds(published[-1].stamp),seconds(published[0].stamp))
            wall[0]=1000.4
            packets.append(pair(3,1000.));VRInputNode.poll(owner)
            self.assertAlmostEqual(wall[0]-950.-seconds(published[-1].stamp),.4)
            packets.extend((pair(4,1001.),pair(5,998.)))
            VRInputNode.poll(owner);VRInputNode.poll(owner)
        self.assertEqual(len(published),6)
        owner.now.assert_called_once()


class ControllerSocketTests(unittest.TestCase):
    def test_immediate_ack_is_rearmed_and_unsupported_sockets_are_nonfatal(self):
        import socket
        from unittest.mock import Mock
        from bindu_teleoperation.vr.receiver import quick_ack
        sock=Mock()
        if hasattr(socket,'TCP_QUICKACK'):
            self.assertTrue(quick_ack(sock));self.assertTrue(quick_ack(sock))
            self.assertEqual(sock.setsockopt.call_count,2)
            sock.setsockopt.side_effect=OSError('unsupported')
        self.assertFalse(quick_ack(None));self.assertFalse(quick_ack(sock))

    def test_kernel_tcp_metrics_are_not_confused_with_one_way_latency(self):
        import struct,socket
        from unittest.mock import Mock
        from bindu_teleoperation.vr.receiver import controller_socket_status
        info=bytearray(104)
        for offset,value in ((68,14000),(72,3000),(100,9)):struct.pack_into('=I',info,offset,value)
        sock=Mock();sock.getsockopt.side_effect=lambda _level,option,*_args: bytes(info) if option==socket.TCP_INFO else 1
        actual=controller_socket_status(sock)
        self.assertTrue(actual['tcp_nodelay'])
        self.assertAlmostEqual(actual['rtt_s'],.014)
        self.assertEqual(actual['total_retransmissions'],9)
        sock.getsockopt.side_effect=OSError('closed')
        self.assertEqual(controller_socket_status(sock),{})
        self.assertEqual(controller_socket_status(None),{})


class ControllerWebSocketTests(unittest.IsolatedAsyncioTestCase):
    async def test_controller_transport_reply_is_bounded_and_does_not_emit_input(self):
        import asyncio, struct
        from types import SimpleNamespace as S
        from unittest.mock import AsyncMock
        from bindu_teleoperation.vr.receiver import acknowledge_controller
        ws=S(ping=AsyncMock())
        await acknowledge_controller(ws,1976)
        ws.ping.assert_awaited_once_with(struct.pack('!Q',1976))
        async def stalled(_):await asyncio.Future()
        ws.ping=stalled
        await asyncio.wait_for(acknowledge_controller(ws,1977),.1)
        ws.ping=AsyncMock(side_effect=ConnectionResetError('disconnected'))
        await acknowledge_controller(ws,1978)

    async def test_actual_handler_delivers_input_before_reply_and_bounds_pending_replies(self):
        import asyncio,queue,numpy as np
        from types import SimpleNamespace as S,ModuleType
        from unittest.mock import AsyncMock,patch
        from bindu_teleoperation.vr.receiver import _serve
        apps=[]
        async def stalled(_):await asyncio.Future()
        ws=S(ping=AsyncMock(side_effect=stalled))
        class Vuer:
            def __init__(self,**kw):self.ws={'offline':ws};self.handlers={};apps.append(self)
            def add_handler(self,name,fn):self.handlers[name]=fn
            def spawn(self,*a,**kw):pass
            def run(self):pass
        vuer=ModuleType('vuer');vuer.Vuer=Vuer
        schemas=ModuleType('vuer.schemas')
        schemas.MotionControllers=schemas.Scene=schemas.Grid=object
        font=ModuleType('PIL.ImageFont');font.load_default=lambda **kw:S(font_bytes=b'font')
        pil=ModuleType('PIL');pil.ImageFont=font
        output=queue.Queue(2);health=queue.Queue(128)
        with patch.dict('sys.modules',{'vuer':vuer,'vuer.schemas':schemas,'PIL':pil,'PIL.ImageFont':font}):
            _serve(dict(host='127.0.0.1',port=0,cert_file=None,key_file=None,side='both',
                mapping='pico_snowbot_v1'),output,queue.Queue(1),health)
            self.assertTrue(apps,list(output.queue))
            app=apps[0];handler=app.handlers['CONTROLLER_MOVE'];session=S(CURRENT_WS_ID='offline')
            state=dict(squeezeValue=0.,triggerValue=0.,aButton=False,bButton=False,thumbstick=False)
            event=S(value=dict(left=np.eye(4).ravel().tolist(),right=np.eye(4).ravel().tolist(),
                leftState=state,rightState=state))
            await handler(event,session)
            first=output.get_nowait()[1]
            self.assertEqual({f.seq for f in first},{1})
            await asyncio.sleep(0)
            await handler(event,session)
            second=output.get_nowait()[1]
            self.assertEqual({f.seq for f in second},{2})
            await asyncio.sleep(0)
            self.assertEqual(ws.ping.await_count,1)
            self.assertTrue(all(f.valid for f in first+second))
            self.assertTrue(all(f.stamp==first[0].stamp for f in first))
            await asyncio.sleep(.02)
            self.assertTrue(output.empty())  # Timeout/pong never mints input.

    async def test_vuer_none_return_preserves_the_prepared_response_on_disconnect(self):
        from types import ModuleType
        from unittest.mock import AsyncMock, patch
        from bindu_teleoperation.vr.receiver import _controller_websocket
        request, response = object(), object()
        server = ModuleType('vuer.server')

        async def wrapper(req, handler, **options):
            self.assertIs(req, request);self.assertEqual(options, {'compress': False})
            await handler(req, response)
            return None

        server.websocket_handler = wrapper
        handler = AsyncMock()
        with patch.dict('sys.modules', {'vuer': ModuleType('vuer'), 'vuer.server': server}):
            self.assertIs(await _controller_websocket(request, handler), response)
        handler.assert_awaited_once_with(request, response)

    async def test_failed_handshake_propagates_without_returning_none(self):
        from types import ModuleType
        from unittest.mock import AsyncMock, patch
        from bindu_teleoperation.vr.receiver import _controller_websocket
        server = ModuleType('vuer.server')
        server.websocket_handler = AsyncMock(side_effect=ConnectionResetError('handshake failed'))
        handler = AsyncMock()
        with patch.dict('sys.modules', {'vuer': ModuleType('vuer'), 'vuer.server': server}):
            with self.assertRaisesRegex(ConnectionResetError, 'handshake failed'):
                await _controller_websocket(object(), handler)
        handler.assert_not_called()


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

    @unittest.skipUnless(shutil.which('node'), 'Node required for browser wire test')
    def test_actual_viewer_uses_controller_space_and_cannot_be_a_desktop_camera(self):
        import base64,msgpack
        from bindu_teleoperation.vr.receiver import HeadsetSamples
        matrix=[1.,0.,0.,0.,0.,1.,0.,0.,0.,0.,1.,0.,0.,0.,0.,1.]
        event=dict(etype='CONTROLLER_MOVE',value=dict(left=matrix,right=matrix),key='motionControllers')
        payload=base64.b64encode(msgpack.packb(event,use_bin_type=True)).decode()
        harness=r'''
const fs=require('fs'),vm=require('vm');let now=1000,packets=[];
const original=JSON.parse(process.argv[3]);
const left={handedness:'left',targetRayMode:'tracked-pointer',targetRaySpace:{side:'left'}};
const right={handedness:'right',targetRayMode:'tracked-pointer',targetRaySpace:{side:'right'}};
class Space {constructor(id){this.id=id;this.events={};}addEventListener(k,fn){this.events[k]=fn;}}
class Session {constructor(){this.visibilityState='visible';this.inputSources=[left,right];this.events={};}
 addEventListener(k,fn){this.events[k]=fn;}requestAnimationFrame(fn){this.callback=fn;}}
class Frame {constructor(session){this.session=session;}
 getPose(space,reference){const m=original.value[space.side].slice();if(reference.id!=='controllers')m[12]+=1;
   return {transform:{matrix:m}};}
 getViewerPose(reference){const m=original.value.left.slice();if(reference.id!=='controllers')m[12]=7;
   return {emulatedPosition:false,transform:{matrix:m}};}}
class Socket {constructor(){this.bufferedAmount=0;}send(b){packets.push(Buffer.from(b).toString('base64'));}}
const navigator={xr:{requestSession:async()=>new Session()}};
const ctx={TextEncoder,Uint8Array,DataView,ArrayBuffer,WeakMap,WebSocket:Socket,navigator,
 XRFrame:Frame,XRSession:Session,performance:{now:()=>now},Date,document:{visibilityState:'visible'},
 setInterval:()=>{},window:{requestAnimationFrame:()=>{},XRFrame:Frame,XRSession:Session}};
vm.createContext(ctx);vm.runInContext(fs.readFileSync(process.argv[1],'utf8'),ctx);
const socket=new Socket(),data=Uint8Array.from(Buffer.from(process.argv[2],'base64'));
const reference=new Space('controllers'),renderer=new Space('renderer');
async function sample(mode){const session=await navigator.xr.requestSession(mode);
 session.requestAnimationFrame((time,frame)=>{for(const ref of [reference,renderer]){
   frame.getPose(left.targetRaySpace,ref);frame.getPose(right.targetRaySpace,ref);}});
 session.callback(now,new Frame(session));return session;}
(async()=>{socket.send(data);await sample('inline');socket.send(data);
 const session=await sample('immersive-vr');socket.send(data);now+=100;socket.send(data);
 reference.events.reset();now+=10;session.callback(now,new Frame(session));socket.send(data);
 session.visibilityState='hidden';session.events.visibilitychange();socket.send(data);
 process.stdout.write(JSON.stringify(packets));})().catch(e=>{console.error(e);process.exit(1);});
'''
        result=subprocess.run(['node','-e',harness,str(SCRIPT),payload,json.dumps(event)],
            check=True,capture_output=True,text=True)
        packets=[msgpack.unpackb(base64.b64decode(p),raw=False) for p in json.loads(result.stdout)]
        self.assertNotIn('bindu_head',packets[0]);self.assertNotIn('bindu_head',packets[1])
        cache=HeadsetSamples()
        first=cache.ingest(packets[2]['bindu_head'],10.,event['value'])
        self.assertEqual(first['head_pose'],tuple(matrix))
        self.assertEqual(cache.ingest(packets[3]['bindu_head'],10.1,event['value']),first)
        self.assertEqual(len(packets[2]['bindu_head']['samples']),2)
        renewed=cache.ingest(packets[4]['bindu_head'],10.11,event['value'])
        self.assertGreater(renewed['head_seq'],first['head_seq']);self.assertTrue(cache.changed)
        self.assertNotIn('bindu_head',packets[5])
        for packet in packets:
            packet.pop('bindu_timing');packet.pop('bindu_head',None)
            self.assertEqual(packet,event)


if __name__ == '__main__':
    unittest.main()
