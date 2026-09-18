"""Publish measured simulation joints and odom; never refresh frozen samples."""
import math
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from bindu_interfaces.msg import ExecutionState
from .common import RuntimeNode, seconds, spin


class DescriptionNode(RuntimeNode):
    def __init__(self):
        super().__init__('description_state')
        self.joints = self.create_publisher(JointState, 'joint_states', 1)
        self.tf = TransformBroadcaster(self)
        self.last_joint_stamp = self.last_base_stamp = -1.
        self.create_subscription(ExecutionState, 'execution/state', self.update, 1)

    def update(self, state):
        if state.profile_hash != self.profile.digest or not state.simulated:
            return
        now = self.now()
        joint_stamp = seconds(state.joints.header.stamp)
        names = {j for group in self.profile.groups.values() for j in group}
        if (set(state.joints.name) == names and len(state.joints.name) == len(names)
                and len(state.joints.position) == len(names)
                and all(math.isfinite(v) for v in state.joints.position)
                and 0 <= now-joint_stamp < .2 and joint_stamp > self.last_joint_stamp):
            self.joints.publish(state.joints)
            self.last_joint_stamp = joint_stamp
        sources = dict(zip(state.feedback_sources, state.source_stamps))
        if 'base' not in sources:
            return
        base_stamp = seconds(sources['base'])
        if not (0 <= now-base_stamp < .2 and base_stamp > self.last_base_stamp
                and all(math.isfinite(v) for v in (state.base_x, state.base_y, state.base_yaw))):
            return
        transform = TransformStamped()
        transform.header.stamp = sources['base']
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = state.base_x
        transform.transform.translation.y = state.base_y
        transform.transform.rotation.z = math.sin(state.base_yaw/2)
        transform.transform.rotation.w = math.cos(state.base_yaw/2)
        self.tf.sendTransform(transform)
        self.last_base_stamp = base_stamp


def main():
    spin(DescriptionNode, threaded=False)
