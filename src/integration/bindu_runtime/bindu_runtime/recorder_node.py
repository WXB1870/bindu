import json
from pathlib import Path
from std_srvs.srv import SetBool
from rosidl_runtime_py.convert import message_to_ordereddict
from bindu_recording.recorder import AsyncRecorder
from bindu_interfaces.msg import RuntimeEvent, ExecutionState, RecorderHealth, MotionCommand, ObjectObservation, VRInput
from .common import RuntimeNode, EVENT_QOS, stamp, spin


class RecorderNode(RuntimeNode):
    def __init__(self):
        super().__init__('recorder', allow_hardware=True, default_mode='simulation')
        self.input_error = ''
        self.declare_parameter('output', '/tmp/bindu-runs')
        self.declare_parameter('recording_mode', 'compact')
        self.declare_parameter('device_backend', '')
        mode = self.get_parameter('recording_mode').value
        self.writer = AsyncRecorder(Path(self.get_parameter('output').value) / self.run_id,
                                    {'schema_version':1, 'run_id':self.run_id, 'simulated':self.simulation,
                                     'runtime_mode':self.runtime_mode,
                                     'device_backend':self.get_parameter('device_backend').value,
                                     'profile_hash':self.profile.digest,
                                     'profile':json.loads(Path(self.profile_path).read_text())}, mode=mode)
        self.capture_available=mode!='off'
        self.capture_enabled=self.capture_available
        self.create_service(SetBool,'recorder/capture',self.capture)
        self.get_logger().info('Recording mode: '+mode)
        if mode != 'off':
            self.create_subscription(RuntimeEvent, 'events', lambda m:self.record('event',m), EVENT_QOS)
            self.create_subscription(ExecutionState, 'execution/state', lambda m:self.record('feedback',m), 20)
            self.create_subscription(MotionCommand, 'execution/accepted', lambda m:self.record('reference',m), EVENT_QOS)
            self.create_subscription(ObjectObservation, 'perception/observations', lambda m:self.record('observation',m), EVENT_QOS)
            self.create_subscription(VRInput, 'teleop/vr/input', lambda m:self.record('vr_input',m), 20)
        self.pub = self.create_publisher(RecorderHealth, 'recorder/health', 5)
        self.create_timer(.1, self.health)

    def capture(self,request,reply):
        if not self.capture_available or self.writer.error or self.writer.dropped:
            reply.success,reply.message=False,'RECORDING_UNAVAILABLE'
            return reply
        self.writer.submit({'kind':'recording_control','received_at':self.now(),'enabled':request.data})
        self.capture_enabled=request.data
        reply.success=True
        reply.message='RECORDING_ENABLED' if request.data else 'RECORDING_DISABLED'
        return reply

    def record(self, kind, message):
        if not self.matches_mode(message):
            self.input_error = 'RECORDING_RUNTIME_MODE_MISMATCH'
            return
        if hasattr(message, 'profile_hash') and message.profile_hash != self.profile.digest:
            self.input_error = 'RECORDING_PROFILE_MISMATCH'
            return
        if not self.capture_enabled:return
        self.writer.submit({'kind':kind, 'received_at':self.now(), 'data':message_to_ordereddict(message)})

    def health(self):
        # ready means the configured mode is available. Explicit off permits
        # control without recording; it is not evidence that data was saved.
        self.pub.publish(RecorderHealth(stamp=stamp(self.now()), run_id=self.run_id, runtime_mode=self.runtime_mode,
                         ready=not bool(self.writer.error or self.input_error) and not bool(self.writer.dropped),
                         written=self.writer.written, dropped=self.writer.dropped, error=self.writer.error or self.input_error,
                         capture_enabled=self.capture_enabled))

    def destroy_node(self):
        self.writer.close()
        return super().destroy_node()


def main():
    spin(RecorderNode, threaded=False)
