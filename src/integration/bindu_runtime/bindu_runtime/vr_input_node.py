"""Explicitly launched Vuer input process; no hardware output."""
import time
from bindu_teleoperation.vr.receiver import VuerReceiver
from bindu_interfaces.msg import VRInput
from .common import RuntimeNode, stamp, spin


class VRInputNode(RuntimeNode):
    def __init__(self):
        super().__init__('vr_input')
        defaults = {'host': '127.0.0.1', 'port': 8012, 'side': 'left', 'cert_file': '', 'key_file': ''}
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        cfg = {key: self.get_parameter(key).value for key in defaults}
        if cfg['side'] not in ('left', 'right'):
            raise ValueError('VR_INVALID_SIDE')
        cfg['cert_file'] = cfg['cert_file'] or None
        cfg['key_file'] = cfg['key_file'] or None
        self.receiver = VuerReceiver(cfg)
        self.publisher = self.create_publisher(VRInput, 'teleop/vr/input', 1)
        self.timer = self.create_timer(.005, self.poll)

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
        age = time.time()-frame.stamp
        if age < 0 or age > 1.:
            return
        self.publisher.publish(VRInput(stamp=stamp(self.now()-age), source_id=frame.source_id,
            seq=frame.seq, side=frame.side, pose=list(frame.pose) if frame.valid else [0.]*16,
            grip=frame.grip, trigger=frame.trigger, init=frame.init, stop=frame.stop,
            valid=frame.valid, clutch_seq=frame.clutch_seq))

    def destroy_node(self):
        self.receiver.close()
        return super().destroy_node()


def main():
    spin(VRInputNode, threaded=False)
