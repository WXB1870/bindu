"""v3.4 CONTROLLER_MOVE wire input; no robot, ROS or IK imports.

Vuer is loaded only inside the explicitly started receiver process. Each
connection owns an identity; malformed events invalidate that side immediately.
"""
import asyncio
import math
import multiprocessing as mp
import queue
import socket
import struct
import time
import tempfile
import uuid
import sys
from pathlib import Path
from dataclasses import replace
import numpy as np
from bindu_contracts.teleoperation import VRFrame
from .mapping import pose_matrix


def decode_controller(value, side, source_id, seq, stamp, mapping="v34"):
    if side not in ('left', 'right'):
        raise ValueError('VR_INVALID_SIDE')
    try:
        # WebXR matrices arrive flattened in column-major order (v3.4 protocol).
        raw = np.asarray(value[side], dtype=float)
        if raw.shape != (16,):
            raise ValueError('VR_MATRIX_LAYOUT')
        pose = tuple(pose_matrix(raw.reshape(4, 4, order='F')).ravel())
        state = value[side + 'State']
        grip, trigger = float(state['squeezeValue']), float(state['triggerValue'])
        if not all(math.isfinite(x) and 0 <= x <= 1 for x in (grip, trigger)):
            raise ValueError('VR_INVALID_BUTTON')
        # Preserve v3.4: right B initializes left arm, right A initializes right.
        right = value.get('rightState', {})
        init = right.get('bButton' if side == 'left' else 'aButton', False)
        stop = value.get('leftState', {}).get('bButton', False)
        if not isinstance(init, bool) or not isinstance(stop, bool):
            raise ValueError('VR_INVALID_BUTTON')
        if mapping not in ('v34', 'pico_snowbot_v1'):
            raise ValueError('VR_MAPPING')
        buttons = 0
        if mapping == 'pico_snowbot_v1':
            bits = [state[key] for key in ('aButton', 'bButton', 'thumbstick')]
            if any(type(bit) is not bool for bit in bits):
                raise ValueError('VR_INVALID_BUTTON')
            buttons = sum(int(bit) << i for i,bit in enumerate(bits))
            init = False
        return VRFrame(source_id, seq, stamp, side, pose, grip, trigger, init, stop,
                       mapping=mapping, buttons=buttons)
    except (KeyError, TypeError, ValueError):
        return VRFrame(source_id, seq, stamp, side, (), valid=False)


class ButtonCounters:
    """Cumulative edges survive pose queue coalescing; never infer a release on loss."""
    def __init__(self):
        self.previous = [False]*4
        self.counts = [0]*8

    def update(self, frame):
        if not frame.valid:
            return frame
        values = [bool(frame.buttons & (1 << i)) for i in range(3)]
        pressed = self.previous[3]
        if frame.trigger > .7: pressed = True
        elif frame.trigger < .3: pressed = False
        values.append(pressed)
        for i,value in enumerate(values):
            if value != self.previous[i]:
                self.counts[2*i + int(not value)] += 1
        self.previous = values
        return replace(frame, button_edges=tuple(self.counts))


def _latest_put(output, item):
    try:
        output.put_nowait(item)
        return False
    except queue.Full:
        try:
            output.get_nowait()
        except queue.Empty:
            return True
        try:
            output.put_nowait(item)
        except queue.Full:
            pass
        return True


def quick_ack(sock):
    """Re-arm Linux's one-shot immediate ACK mode; never affect frame validity."""
    if sock is None or not hasattr(socket,'TCP_QUICKACK'):return False
    try:
        sock.setsockopt(socket.IPPROTO_TCP,socket.TCP_QUICKACK,1)
        return True
    except OSError:return False


def controller_socket_status(sock):
    """Read kernel evidence separately from browser and server-loop clocks."""
    if sock is None:return {}
    result={}
    try:result['tcp_nodelay']=bool(sock.getsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY))
    except OSError:pass
    if hasattr(socket,'TCP_INFO'):
        try:
            info=sock.getsockopt(socket.IPPROTO_TCP,socket.TCP_INFO,104)
            if len(info)>=104:
                # Linux struct tcp_info's stable initial 104 bytes.
                result.update(rtt_s=struct.unpack_from('=I',info,68)[0]*1e-6,
                    rtt_variation_s=struct.unpack_from('=I',info,72)[0]*1e-6,
                    total_retransmissions=struct.unpack_from('=I',info,100)[0])
        except OSError:pass
    return result


def _serve(config, output, feedback, health):
    try:
        from vuer import Vuer
        from vuer.schemas import MotionControllers, Scene, Grid

        controller_sockets={}

        class LocalVuer(Vuer):
            if config.get('input_timing_enabled',False):
                async def bound_fn(self,session_proxy):
                    # This is after Vuer's MessagePack decode, before its
                    # handler task is scheduled. It is not socket arrival.
                    upstream=super().bound_fn(session_proxy)
                    try:
                        outgoing=await upstream.__anext__()
                        while True:
                            event=yield outgoing
                            if event.etype=='CONTROLLER_MOVE':
                                event.bindu_dispatched_monotonic=time.monotonic()
                            outgoing=await upstream.asend(event)
                    finally:
                        await upstream.aclose()

            async def socket_index(self, request):
                # Vuer 0.0.60's HTTPS fallback omits the page's explicit port.
                # Keep the locally served page and socket on the same origin.
                if (request.headers.get('Upgrade', '').lower() != 'websocket'
                        and not request.query.get('ws')):
                    from aiohttp.web import HTTPFound
                    scheme = 'wss' if request.secure else 'ws'
                    query = dict(request.query)
                    query['ws'] = f'{scheme}://{request.host}{request.path}'
                    raise HTTPFound(location=str(request.rel_url.with_query(query)))
                if request.headers.get('Upgrade', '').lower() != 'websocket':
                    from aiohttp.web import Response
                    html = (self.client_root/'index.html').read_text()
                    return Response(text=html.replace('<head>',
                        '<head><script src="/static/client-timing.js"></script>', 1),
                        content_type='text/html', headers={'Cache-Control': 'no-store'})
                from vuer.server import websocket_handler
                return await websocket_handler(request,self.downlink,compress=False)

            async def downlink(self, request, ws):
                sock=request.transport.get_extra_info('socket') if request.transport else None
                if sock is not None:
                    try:sock.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
                    except OSError:pass
                controller_sockets[id(ws)]=sock
                quick_ack(sock)
                try:return await super().downlink(request,ws)
                finally:controller_sockets.pop(id(ws),None)

        static = tempfile.TemporaryDirectory(prefix='bindu-vr-static-')
        # Serve Pillow's bundled font locally; the headset needs no font CDN.
        from PIL import ImageFont
        (Path(static.name)/'display.ttf').write_bytes(ImageFont.load_default(size=14).font_bytes)
        (Path(static.name)/'client-timing.js').write_bytes(Path(__file__).with_name('client_timing.js').read_bytes())
        model_url = None
        if config.get('display_urdf'):
            from .model import prepare_model
            model_url=prepare_model(config['display_urdf'],static.name,config.get('display_tool_config'),
                                    short_rod=config.get('simulation_demo',False))
        app = LocalVuer(host=config['host'], port=config['port'],
                   cert=config['cert_file'], key=config['key_file'],
                   static_root=static.name, queries={'grid': True}, queue_len=2)
        owner = None
        identity = ''
        seq = 0
        clutch_seq = {'left': 0, 'right': 0}
        previous_grip = {'left': 0., 'right': 0.}
        stop_latched = False
        counters = {side: ButtonCounters() for side in ('left','right')}
        latest_display = None
        display_received = float('-inf')
        last_event = None
        last_report = 0.
        loop_delay = 0.
        from .timing import ControllerTiming
        controller_timing = ControllerTiming()
        health_overflows = 0

        def emit_health(report):
            nonlocal health_overflows
            report['health_queue_overflows'] = health_overflows
            if _latest_put(health, report):health_overflows += 1

        async def monitor_loop(session):
            nonlocal loop_delay
            while session.CURRENT_WS_ID in app.ws:
                due = time.monotonic()+.02
                await asyncio.sleep(.02)
                loop_delay=max(loop_delay,time.monotonic()-due)
                if time.time()-last_report>=1.:report_health(time.time())

        def report_health(now, gap=None):
            nonlocal last_report, loop_delay
            emit_health(dict(stamp=now,source_id=identity,seq=seq,
                event_interval_s=gap,event_loop_delay_max_s=loop_delay,
                websocket_clients=len(app.ws),last_event_stamp=last_event,
                controller_fps=60 if config.get('mapping')=='pico_snowbot_v1' else 30,
                socket=controller_socket_status(controller_sockets.get(id(app.ws.get(owner.CURRENT_WS_ID)))) if owner else {},
                stage='websocket_handler',stamp_semantics='server_report_wall_clock',
                last_event_stamp_semantics='controller_handler_entry_wall_clock'))
            last_report=now;loop_delay=0.
        from .connectivity import VRConnectivity
        connectivity = VRConnectivity() if config.get('connectivity_check') else None

        async def on_controller(event, session):
            nonlocal owner, identity, seq, clutch_seq, previous_grip, stop_latched, counters, last_event
            if owner is None or owner.CURRENT_WS_ID not in app.ws:
                owner, identity = session, uuid.uuid4().hex
                clutch_seq = {'left': 0, 'right': 0}
                previous_grip = {'left': 0., 'right': 0.}
                stop_latched = False
                counters = {side: ButtonCounters() for side in ('left','right')}
                last_event=None
            if owner is not session:
                return  # A second browser must not interleave commands.
            quick_ack(controller_sockets.get(id(app.ws.get(session.CURRENT_WS_ID))))
            seq += 1
            now = time.time()
            handler_started=time.monotonic()
            emit_health(controller_timing.observe(getattr(event, 'bindu_timing', None),
                now, handler_started, identity, seq))
            gap=now-last_event if last_event is not None else None
            last_event=now
            if now-last_report>=1. or gap is not None and gap>.1:report_health(now,gap)
            sides = ('left', 'right') if config['side'] == 'both' else (config['side'],)
            frames = [decode_controller(event.value, side, identity, seq, now, config.get("mapping", "v34")) for side in sides]
            stop_latched = stop_latched or any(frame.stop for frame in frames)
            paired = []
            for frame in frames:
                side = frame.side
                if frame.valid:
                    if frame.grip < .3 and previous_grip[side] >= .3:
                        clutch_seq[side] += 1
                    previous_grip[side] = frame.grip
                item = counters[side].update(frame)
                paired.append(replace(item, clutch_seq=clutch_seq[side],
                    stop=frame.stop if config.get('mapping') == 'pico_snowbot_v1' else stop_latched))
            # A single queue item keeps the two sides from separate event epochs together.
            if connectivity is not None:
                connectivity.ingest(tuple(paired))
            packet=('frame',tuple(paired) if len(paired)==2 else paired[0])
            if config.get('input_timing_enabled',False):
                packet+= (dict(handler_started=handler_started,frame_ready=time.monotonic(),
                               dispatched=getattr(event,'bindu_dispatched_monotonic',None)),)
            _latest_put(output,packet)

        async def display(session):
            nonlocal owner, latest_display, display_received
            from ..feedback import display_snapshot
            from .display import render
            session.set(Scene(camPosition=[0., 1., 2.], camRotation=[0., 0., 0.],
                bgChildren=[Grid(key='default-grid'),
                            MotionControllers(stream=True, key='motionControllers', left=True, right=True,
                                fps=60 if config.get('mapping')=='pico_snowbot_v1' else 30)]))
            render_error = ''
            render_key = None
            loop_probe = asyncio.create_task(monitor_loop(session))
            try:
                while session.CURRENT_WS_ID in app.ws:
                    for _ in range(2):
                        try:
                            display_received, latest_display = feedback.get_nowait()
                        except queue.Empty:
                            break
                    view = (connectivity.snapshot(time.time()) if connectivity is not None
                            else display_snapshot(latest_display, time.time()-display_received))
                    if config.get('mapping')=='pico_snowbot_v1' and connectivity is None:
                        view=dict(view,pico=True,model_url=model_url,
                            model_tool_url='/static/snowbot-tool.urdf' if model_url and config.get('display_tool_config') else None)
                    if config.get('runtime_mode')=='real':
                        view=dict(view,runtime_mode='real',simulated=False)
                    try:
                        key = None
                        if view.get('pico') and view.get('runtime_mode')!='real':
                            from .hud import hud_spec
                            key=(tuple(hud_spec(view).items()),view.get('load_state'),
                                 tuple(sorted((n,round(q,4)) for n,q in (view.get('model_joints') or {}).items())))
                        if key is None or key!=render_key:
                            scene = await asyncio.to_thread(render, view)
                            session.upsert(scene, to='bgChildren')
                            render_key=key
                        render_error = ''
                    except Exception as exc:
                        # Rendering is optional and must never invalidate controller input.
                        if str(exc) != render_error:
                            print('TELEOP_DISPLAY_FAILED:', exc, flush=True)
                            render_error = str(exc)
                    await asyncio.sleep(.1)
            finally:
                loop_probe.cancel()
                try:await loop_probe
                except asyncio.CancelledError:pass
                if owner is session:
                    owner = None

        # Probe the event loop independently of frames; never publish a
        # heartbeat as a new controller input or refresh a frame timestamp.
        app.add_handler('CONTROLLER_MOVE', on_controller)
        app.spawn(display, start=False)
        app.run()
    except Exception as exc:
        _latest_put(output, ('error', type(exc).__name__ + ': ' + str(exc)))


class VuerReceiver:
    def __init__(self, config):
        ctx = mp.get_context('spawn')
        self.output = ctx.Queue(maxsize=2)
        self.feedback = ctx.Queue(maxsize=1)
        self.health = ctx.Queue(maxsize=128)
        self.process = ctx.Process(target=_serve, args=(config, self.output, self.feedback, self.health), daemon=True)
        self.process.start()
        self.last_delivery=None

    def display(self, snapshot):
        # Wall clock is shared on supported hosts, including macOS Python <3.10.
        _latest_put(self.feedback, (time.time(), snapshot))

    def poll(self):
        latest = None
        self.last_delivery=None
        for _ in range(4):
            try:
                packet = self.output.get_nowait()
            except queue.Empty:
                break
            kind,item=packet[:2]
            if kind == 'error':
                self.last_delivery=None
                raise RuntimeError('VR_RECEIVER_FAILED: ' + item)
            latest = item
            if len(packet)==3:
                first=item[0] if isinstance(item,tuple) else item
                self.last_delivery=dict(packet[2],source_id=first.source_id,seq=first.seq,
                    dequeued=time.monotonic(),shared_monotonic=not(sys.platform=='darwin' and sys.version_info<(3,10)))
            else:self.last_delivery=None
        if not self.process.is_alive():
            self.last_delivery=None
            raise RuntimeError('VR_RECEIVER_EXITED')
        return latest

    def poll_health(self):
        reports=[]
        for _ in range(128):
            try:reports.append(self.health.get_nowait())
            except queue.Empty:break
        return reports

    def close(self):
        if self.process.is_alive():
            self.process.terminate()
        self.process.join(timeout=2.)
        if self.process.is_alive():
            self.process.kill()
            self.process.join(timeout=1.)
        self.output.close()
        self.output.cancel_join_thread()
        self.feedback.close()
        self.feedback.cancel_join_thread()
        self.health.close()
        self.health.cancel_join_thread()
