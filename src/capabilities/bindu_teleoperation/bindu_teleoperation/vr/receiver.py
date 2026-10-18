"""v3.4 CONTROLLER_MOVE wire input; no robot, ROS or IK imports.

Vuer is loaded only inside the explicitly started receiver process. Each
connection owns an identity; malformed events invalidate that side immediately.
"""
import asyncio
import math
import multiprocessing as mp
import queue
import time
import tempfile
import uuid
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
    except queue.Full:
        try:
            output.get_nowait()
        except queue.Empty:
            return
        try:
            output.put_nowait(item)
        except queue.Full:
            pass


def _serve(config, output, feedback):
    try:
        from vuer import Vuer
        from vuer.schemas import MotionControllers, Scene, Grid

        class LocalVuer(Vuer):
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
                return await super().socket_index(request)

        static = tempfile.TemporaryDirectory(prefix='bindu-vr-static-')
        # Serve Pillow's bundled font locally; the headset needs no font CDN.
        from PIL import ImageFont
        (Path(static.name)/'display.ttf').write_bytes(ImageFont.load_default(size=14).font_bytes)
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
        from .connectivity import VRConnectivity
        connectivity = VRConnectivity() if config.get('connectivity_check') else None

        async def on_controller(event, session):
            nonlocal owner, identity, seq, clutch_seq, previous_grip, stop_latched, counters
            if owner is None or owner.CURRENT_WS_ID not in app.ws:
                owner, identity = session, uuid.uuid4().hex
                clutch_seq = {'left': 0, 'right': 0}
                previous_grip = {'left': 0., 'right': 0.}
                stop_latched = False
                counters = {side: ButtonCounters() for side in ('left','right')}
            if owner is not session:
                return  # A second browser must not interleave commands.
            seq += 1
            now = time.time()
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
            _latest_put(output, ('frame', tuple(paired) if len(paired) == 2 else paired[0]))

        async def display(session):
            nonlocal owner, latest_display, display_received
            from ..feedback import display_snapshot
            from .display import render
            session.set(Scene(camPosition=[0., 1., 2.], camRotation=[0., 0., 0.],
                bgChildren=[Grid(key='default-grid'),
                            MotionControllers(stream=True, key='motionControllers', left=True, right=True)]))
            render_error = ''
            try:
                while session.CURRENT_WS_ID in app.ws:
                    for _ in range(2):
                        try:
                            display_received, latest_display = feedback.get_nowait()
                        except queue.Empty:
                            break
                    view = (connectivity.snapshot(time.time()) if connectivity is not None
                            else display_snapshot(latest_display, time.time()-display_received))
                    if config.get('runtime_mode')=='real':
                        view=dict(view,runtime_mode='real',simulated=False)
                    try:
                        scene = await asyncio.to_thread(render, view)
                        session.upsert(scene, to='bgChildren')
                        render_error = ''
                    except Exception as exc:
                        # Rendering is optional and must never invalidate controller input.
                        if str(exc) != render_error:
                            print('TELEOP_DISPLAY_FAILED:', exc, flush=True)
                            render_error = str(exc)
                    await asyncio.sleep(.1)
            finally:
                if owner is session:
                    owner = None

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
        self.process = ctx.Process(target=_serve, args=(config, self.output, self.feedback), daemon=True)
        self.process.start()

    def display(self, snapshot):
        # Wall clock is shared on supported hosts, including macOS Python <3.10.
        _latest_put(self.feedback, (time.time(), snapshot))

    def poll(self):
        latest = None
        for _ in range(4):
            try:
                kind, item = self.output.get_nowait()
            except queue.Empty:
                break
            if kind == 'error':
                raise RuntimeError('VR_RECEIVER_FAILED: ' + item)
            latest = item
        if not self.process.is_alive():
            raise RuntimeError('VR_RECEIVER_EXITED')
        return latest

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
