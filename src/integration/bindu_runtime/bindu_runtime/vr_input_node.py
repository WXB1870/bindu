"""Explicitly launched Vuer input process; no hardware output."""
import time
import json
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from bindu_teleoperation.vr.receiver import VuerReceiver
from bindu_interfaces.msg import VRInput
from .common import RuntimeNode, stamp, spin


class VRInputNode(RuntimeNode):
    def __init__(self):
        super().__init__('vr_input', allow_hardware=True, default_mode='simulation')
        defaults = {'host': '127.0.0.1', 'port': 8012, 'side': 'left', 'cert_file': '', 'key_file': ''}
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        cfg = {key: self.get_parameter(key).value for key in defaults}
        if cfg['side'] not in ('left', 'right', 'both'):
            raise ValueError('VR_INVALID_SIDE')
        cfg['cert_file'] = cfg['cert_file'] or None
        cfg['key_file'] = cfg['key_file'] or None
        self.receiver = VuerReceiver(cfg)
        # Both sides share one receiver event; retain both messages until DDS
        # delivers the pair to mode, single-arm and bimanual subscribers.
        self.publisher = self.create_publisher(VRInput, 'teleop/vr/input', 5)
        self.timer = self.create_timer(.005, self.poll)
        self.create_subscription(String, 'teleop/display', self.on_display,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

    def on_display(self, msg):
        try:
            view = json.loads(msg.data)
            if (view['run_id'] == self.run_id and view['profile_hash'] == self.profile.digest
                    and view.get('runtime_mode') == self.runtime_mode
                    and 0 <= self.now()-view['stamp'] <= .6):
                self.receiver.display(view)
        except (ValueError, KeyError, TypeError):
            pass

    def poll(self):
        try:
            frame = self.receiver.poll()
        except RuntimeError as exc:
            self.event('VR_INPUT_FAILED', str(exc))
            self.get_logger().error(str(exc))
            self.timer.cancel()
            return
        if frame is None:
            return
        for item in frame if isinstance(frame, tuple) else (frame,):
            age = time.time()-item.stamp
            if age < 0 or age > 1.:
                continue
            self.publisher.publish(VRInput(stamp=stamp(self.now()-age), source_id=item.source_id,
                seq=item.seq, side=item.side, runtime_mode=self.runtime_mode,
                pose=list(item.pose) if item.valid else [0.]*16,
                grip=item.grip, trigger=item.trigger, init=item.init, stop=item.stop,
                valid=item.valid, clutch_seq=item.clutch_seq))

    def destroy_node(self):
        self.receiver.close()
        return super().destroy_node()


def main():
    spin(VRInputNode, threaded=False)
