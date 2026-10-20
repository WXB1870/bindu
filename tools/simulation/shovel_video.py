"""Simulation-time viewport recording; rendering must never advance physics."""
import ctypes
import json
from pathlib import Path
import shutil
import subprocess
import time


class ViewportRecorder:
    def __init__(self, output, fps):
        from omni.kit.viewport.utility import get_active_viewport
        if not shutil.which('ffmpeg'):
            raise RuntimeError('VIDEO_FFMPEG_REQUIRED')
        self.output = Path(output)
        self.fps = fps
        self.viewport = get_active_viewport()
        if self.viewport is None:
            raise RuntimeError('VIDEO_VIEWPORT_REQUIRED')
        self.viewport.resolution = (1280, 720)
        self.proc = None
        self.log = (self.output/'video_encoder.log').open('w')
        self.times = []
        self.path = self.output/'physics.mp4'

    def due(self, stamp):
        return stamp + 1e-9 >= len(self.times)/self.fps

    def capture(self, world, stamp):
        from isaacsim.core.simulation_manager import SimulationManager
        from omni.kit.viewport.utility import capture_viewport_to_buffer
        import omni.kit.renderer_capture
        before = SimulationManager.get_num_physics_steps()
        captured = {}

        def receive(buffer, size, width, height, pixel_format):
            try:
                if (width, height) != (1280, 720) or size != width*height*4:
                    raise RuntimeError(f'VIDEO_BUFFER_LAYOUT:{width},{height},{size}')
                name = ctypes.pythonapi.PyCapsule_GetName
                name.argtypes, name.restype = [ctypes.py_object], ctypes.c_char_p
                pointer = ctypes.pythonapi.PyCapsule_GetPointer
                pointer.argtypes, pointer.restype = [ctypes.py_object, ctypes.c_char_p], ctypes.c_void_p
                captured['bytes'] = ctypes.string_at(pointer(buffer, name(buffer)), size)
                captured['format'] = str(pixel_format)
            except Exception as exc:
                captured['error'] = exc

        capture = capture_viewport_to_buffer(self.viewport, receive, is_hdr=False)
        renderer = omni.kit.renderer_capture.acquire_renderer_capture_interface()
        deadline = time.monotonic()+20.
        while not captured:
            world.render()
            renderer.wait_async_capture()
            if time.monotonic() > deadline:
                raise RuntimeError('VIDEO_CAPTURE_TIMEOUT')
        if SimulationManager.get_num_physics_steps() != before:
            raise RuntimeError('VIDEO_ADVANCED_PHYSICS')
        if 'error' in captured:
            raise captured['error']
        fmt = captured['format']
        if 'R8G8B8A8' in fmt or 'RGBA8' in fmt:
            raw_format = 'rgba'
        elif 'B8G8R8A8' in fmt or 'BGRA8' in fmt:
            raw_format = 'bgra'
        else:
            raise RuntimeError('VIDEO_PIXEL_FORMAT:'+fmt)
        if self.proc is None:
            self.proc = subprocess.Popen([
                'ffmpeg', '-hide_banner', '-loglevel', 'error', '-nostdin', '-n',
                '-f', 'rawvideo', '-pix_fmt', raw_format, '-s', '1280x720',
                '-r', str(self.fps), '-i', 'pipe:0', '-an', '-c:v', 'libx264',
                '-preset', 'fast', '-crf', '19', '-pix_fmt', 'yuv420p',
                '-movflags', '+faststart', str(self.path)], stdin=subprocess.PIPE, stderr=self.log)
        self.proc.stdin.write(captured['bytes'])
        self.times.append(float(stamp))
        # Keep the capture object alive until its callback has copied the buffer.
        del capture

    def close(self):
        if self.proc is not None:
            self.proc.stdin.close()
            try:
                code = self.proc.wait(timeout=30.)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait()
                raise RuntimeError('VIDEO_ENCODER_TIMEOUT')
            if code:
                raise RuntimeError('VIDEO_ENCODER_FAILED')
        self.log.close()
        report = dict(file='physics.mp4', fps=self.fps, frame_count=len(self.times),
                      frame_simulation_time_s=self.times, physics_steps_during_capture=0,
                      time_basis='recorded live during this PhysX run; startup settling excluded')
        (self.output/'video.json').write_text(json.dumps(report, indent=2)+'\n')
