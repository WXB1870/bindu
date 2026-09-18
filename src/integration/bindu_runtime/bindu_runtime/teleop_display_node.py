"""Independent, best-effort observer; no motion publisher, lease or control client."""
import json
import math
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from bindu_interfaces.msg import ExecutionState, RuntimeEvent, VRInput
from bindu_kinematics.model import ArmModel
from bindu_teleoperation.config import load_config
from bindu_teleoperation.feedback import TeleopFeedback
from .common import RuntimeNode, seconds, spin


class TeleopDisplayNode(RuntimeNode):
    def __init__(self):
        super().__init__('teleop_display')
        self.declare_parameter('teleop_config', str(Path(get_package_share_directory('bindu_runtime'))/'config/teleop_v34.json'))
        cfg = load_config(self.get_parameter('teleop_config').value, self.profile,
                          Path(get_package_share_directory('bindu_kinematics'))/'models')
        self.feedback = TeleopFeedback(cfg)
        self.model = ArmModel(cfg['kinematics'])
        self.state = self.frame = None
        latest = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        events = QoSProfile(depth=64, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(ExecutionState, 'execution/state', lambda m: setattr(self, 'state', m), latest)
        self.create_subscription(VRInput, 'teleop/vr/input', lambda m: setattr(self, 'frame', m), latest)
        self.create_subscription(RuntimeEvent, 'events', self.on_event, events)
        self.publisher = self.create_publisher(String, 'teleop/display', latest)
        self.create_timer(.1, self.publish)

    def on_event(self, msg):
        if msg.run_id != self.run_id or msg.component != 'teleop' or not msg.simulated:
            return
        try:
            self.feedback.event(msg.state, msg.code, msg.task_id, seconds(msg.stamp))
        except (ValueError, KeyError, TypeError):
            self.get_logger().warning('Ignoring malformed display event')

    def publish(self):
        measured, feedback_stamp, error = None, float('-inf'), ''
        state = self.state
        if state is not None and state.simulated and state.profile_hash == self.profile.digest:
            feedback_stamp = seconds(state.feedback_stamp)
            if 0 <= self.now()-feedback_stamp <= self.feedback.cfg['feedback_max_age']:
                try:
                    if len(state.joints.name) != len(state.joints.position) or len(set(state.joints.name)) != len(state.joints.name):
                        raise ValueError('DISPLAY_JOINT_LAYOUT')
                    positions = dict(zip(state.joints.name, state.joints.position))
                    q = [positions[name] for name in self.model.names]
                    if not all(math.isfinite(v) for v in q):
                        raise ValueError('DISPLAY_JOINT_NONFINITE')
                    measured = self.model.fk(q).ravel().tolist()
                except (ValueError, KeyError) as exc:
                    error = 'DISPLAY_FK_INVALID: ' + str(exc)
        frame = self.frame
        view = self.feedback.snapshot(self.now(), measured, feedback_stamp,
            seconds(frame.stamp) if frame else float('-inf'),
            bool(frame and frame.valid and frame.side == self.feedback.cfg['side']), error)
        view.update(run_id=self.run_id, profile_hash=self.profile.digest, stamp=self.now())
        self.publisher.publish(String(data=json.dumps(view, allow_nan=False)))


def main():
    spin(TeleopDisplayNode, threaded=False)
