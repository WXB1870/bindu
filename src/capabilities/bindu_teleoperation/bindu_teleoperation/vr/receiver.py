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
from dataclasses import replace
import numpy as np
from bindu_contracts.teleoperation import VRFrame
from .mapping import pose_matrix


def decode_controller(value, side, source_id, seq, stamp):
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
        return VRFrame(source_id, seq, stamp, side, pose, grip, trigger, init, stop)
    except (KeyError, TypeError, ValueError):
        return VRFrame(source_id, seq, stamp, side, (), valid=False)


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


def _serve(config, output):
    try:
        from vuer import Vuer
        from vuer.schemas import MotionControllers
        static = tempfile.TemporaryDirectory(prefix='bindu-vr-static-')
        app = Vuer(host=config['host'], port=config['port'],
                   cert=config['cert_file'], key=config['key_file'],
                   static_root=static.name, queries={'grid': True}, queue_len=2)
        owner = None
        identity = ''
        seq = 0
        clutch_seq = 0
        previous_grip = 0.
        stop_latched = False

        async def on_controller(event, session):
            nonlocal owner, identity, seq, clutch_seq, previous_grip, stop_latched
            if owner is None or owner.CURRENT_WS_ID not in app.ws:
                owner, identity = session, uuid.uuid4().hex
                clutch_seq, previous_grip, stop_latched = 0, 0., False
            if owner is not session:
                return  # A second browser must not interleave commands.
            seq += 1
            now = time.time()
            frame = decode_controller(event.value, config['side'], identity, seq, now)
            if frame.valid:
                if frame.grip < .3 and previous_grip >= .3:
                    clutch_seq += 1
                previous_grip = frame.grip
                stop_latched = stop_latched or frame.stop
            frame = replace(frame, clutch_seq=clutch_seq, stop=stop_latched)
            _latest_put(output, ('frame', frame))

        async def display(session):
            nonlocal owner
            session.upsert(MotionControllers(stream=True, key='motionControllers',
                                             left=True, right=True), to='bgChildren')
            try:
                while session.CURRENT_WS_ID in app.ws:
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
        self.process = ctx.Process(target=_serve, args=(config, self.output), daemon=True)
        self.process.start()

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
