import json
from pathlib import Path
from rosidl_runtime_py.convert import message_to_ordereddict
from bindu_recording.recorder import AsyncRecorder
from bindu_interfaces.msg import RuntimeEvent, ExecutionState, RecorderHealth, MotionCommand, ObjectObservation, VRInput
from .common import RuntimeNode, EVENT_QOS, stamp, spin


class RecorderNode(RuntimeNode):
    def __init__(self):
        super().__init__('recorder')
        self.declare_parameter('output', '/tmp/bindu-runs')
        self.declare_parameter('recording_mode', 'normal')
        mode = self.get_parameter('recording_mode').value
        self.writer = AsyncRecorder(Path(self.get_parameter('output').value) / self.run_id,
                                    {'schema_version':1, 'run_id':self.run_id, 'simulated':True,
                                     'profile_hash':self.profile.digest,
                                     'profile':json.loads(Path(self.profile_path).read_text())}, mode=mode)
        self.get_logger().info('Recording mode: '+mode)
        if mode != 'off':
            self.create_subscription(RuntimeEvent, 'events', lambda m:self.record('event',m), EVENT_QOS)
            self.create_subscription(ExecutionState, 'execution/state', lambda m:self.record('feedback',m), 20)
            self.create_subscription(MotionCommand, 'execution/accepted', lambda m:self.record('reference',m), EVENT_QOS)
            self.create_subscription(ObjectObservation, 'perception/observations', lambda m:self.record('observation',m), EVENT_QOS)
            self.create_subscription(VRInput, 'teleop/vr/input', lambda m:self.record('vr_input',m), 20)
        self.pub = self.create_publisher(RecorderHealth, 'recorder/health', 5)
        self.create_timer(.1, self.health)

    def record(self, kind, message):
        self.writer.submit({'kind':kind, 'received_at':self.now(), 'data':message_to_ordereddict(message)})

    def health(self):
        # ready means the configured mode is available. Explicit off permits
        # control without recording; it is not evidence that data was saved.
        self.pub.publish(RecorderHealth(stamp=stamp(self.now()), run_id=self.run_id,
                         ready=not bool(self.writer.error) and not bool(self.writer.dropped),
                         written=self.writer.written, dropped=self.writer.dropped, error=self.writer.error))

    def destroy_node(self):
        self.writer.close()
        return super().destroy_node()


def main():
    spin(RecorderNode, threaded=False)
