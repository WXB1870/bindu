"""Independent, best-effort observer; no motion publisher, lease or control client."""
import json
import math
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from bindu_interfaces.msg import ExecutionState, RuntimeEvent, VRInput, ToolPoseInput
from bindu_kinematics.model import ArmModel
from bindu_teleoperation.config import load_config
from bindu_teleoperation.vr.mapping import pose_matrix
from bindu_teleoperation.feedback import TeleopFeedback, ToolFeedback
from .common import RuntimeNode, seconds, spin


class TeleopDisplayNode(RuntimeNode):
    def __init__(self):
        super().__init__('teleop_display')
        self.declare_parameter('teleop_config', str(Path(get_package_share_directory('bindu_runtime'))/'config/teleop_v34.json'))
        self.declare_parameter('model_root', str(Path(get_package_share_directory('bindu_kinematics'))/'models'))
        self.declare_parameter('tool_config', '')
        tool_config = self.get_parameter('tool_config').value
        self.tool_mode = bool(tool_config)
        if self.tool_mode:
            cfg = json.loads(Path(tool_config).read_text())
            urdf = Path(cfg['urdf'])
            if not urdf.is_absolute():
                urdf = Path(self.get_parameter('model_root').value)/urdf
            names = [n for group in self.profile.groups.values() for n in group]
            self.model = ArmModel({'urdf': str(urdf), 'joint_names': names,
                'ee_link': cfg['left_frame'], 'tool_transform': cfg['left_to_tool'],
                'locked_joints': {}})
            self.feedback = ToolFeedback(cfg)
        else:
            cfg = load_config(self.get_parameter('teleop_config').value, self.profile,
                              self.get_parameter('model_root').value)
            self.feedback = TeleopFeedback(cfg)
            self.model = ArmModel(cfg['kinematics'])
        self.state = self.frame = None
        latest = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        events = QoSProfile(depth=64, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(ExecutionState, 'execution/state', lambda m: setattr(self, 'state', m), latest)
        if self.tool_mode:
            self.create_subscription(ToolPoseInput, 'teleop/tool/input', lambda m: setattr(self, 'frame', m), latest)
        else:
            self.create_subscription(VRInput, 'teleop/vr/input', lambda m: setattr(self, 'frame', m), latest)
        self.create_subscription(RuntimeEvent, 'events', self.on_event, events)
        self.publisher = self.create_publisher(String, 'teleop/tool/display' if self.tool_mode else 'teleop/display', latest)
        self.create_timer(.1, self.publish)

    def on_event(self, msg):
        if msg.run_id != self.run_id or msg.component != ('teleop_tool' if self.tool_mode else 'teleop') or not msg.simulated:
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
                    context = ([positions[n] for n in self.model.context_names]
                               if self.model.cfg.get('measured_context', False) else None)
                    if context is not None and not all(math.isfinite(v) for v in context):
                        raise ValueError('DISPLAY_CONTEXT_NONFINITE')
                    measured = self.model.fk(q, context).ravel().tolist()
                except (ValueError, KeyError) as exc:
                    error = 'DISPLAY_FK_INVALID: ' + str(exc)
        frame = self.frame
        input_stamp = seconds(frame.stamp) if frame else self.feedback.target_stamp
        input_valid = bool(frame and frame.valid and not frame.stop) if self.tool_mode else bool(
            frame and frame.valid and frame.side == self.feedback.cfg['side'])
        # The VR tool path has no ToolPoseInput publisher; its bounded QP event carries the target stamp.
        if self.tool_mode and frame is None:
            input_valid = self.feedback.target is not None
        view = self.feedback.snapshot(self.now(), measured, feedback_stamp, input_stamp, input_valid, error)
        if self.tool_mode:
            # Raw request may differ from the last target processed by the QP.
            view['requested'] = None
            if (frame is not None and input_valid and frame.frame_id == view['frame'] and
                    0 <= self.now()-input_stamp <= self.feedback.cfg['input_max_age']):
                try:
                    view['requested'] = pose_matrix(frame.pose).ravel().tolist()
                    view['grasp_requested'] = 'rigid' if frame.tight else 'right_axial'
                except ValueError:
                    view['reason'] = 'TOOL_INPUT_INVALID'
        view.update(run_id=self.run_id, profile_hash=self.profile.digest, stamp=self.now())
        self.publisher.publish(String(data=json.dumps(view, allow_nan=False)))


def main():
    spin(TeleopDisplayNode, threaded=False)
