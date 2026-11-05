"""ROS transport for a reusable ToolSession."""

import os
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.task import Future
from std_srvs.srv import Trigger, SetBool
from bindu_interfaces.action import TeleopSession as TeleopAction
from bindu_interfaces.msg import VRInput, RecorderHealth, TeleopModeState, ToolPoseInput
from .common import RuntimeNode, spin
from .tool_session import ToolSession, WORKER_POLL_PERIOD


class BimanualToolNode(RuntimeNode):
    def __init__(self, *, gripper_config_loader=None, tool_config_loader=None):
        super().__init__("teleop_tool", allow_hardware=True, default_mode="simulation")
        self.waiters = set()
        self.session = ToolSession(
            self,
            self.pause,
            gripper_config_loader=gripper_config_loader,
            tool_config_loader=tool_config_loader,
        )
        session = self.session
        self.create_subscription(
            VRInput,
            "teleop/vr/input",
            session.on_frame,
            5,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.create_subscription(
            ToolPoseInput,
            "teleop/tool/input",
            session.on_tool_frame,
            5,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        if session.require_recording:
            self.create_subscription(RecorderHealth, "recorder/health", session.on_health, 5)
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(TeleopModeState, "teleop/mode_state", self.on_mode, qos)
        self.create_timer(WORKER_POLL_PERIOD, self.tick)
        self.create_service(Trigger, "teleop/tool_ready", session.ready)
        if session.independent_enabled:
            self.create_service(Trigger, "teleop/arms_ready", session.arms_ready)
        self.create_service(SetBool, "teleop/tool_pause", session.request_pause)
        self.server = ActionServer(
            self,
            TeleopAction,
            "teleop/tool_session",
            execute_callback=session.execute,
            goal_callback=session.goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup(),
        )

    def on_mode(self, message):
        self.session.mode_state = message if self.matches_mode(message) else None

    async def pause(self):
        future = Future(executor=self.executor)
        self.waiters.add(future)
        try:
            await future
        finally:
            self.waiters.discard(future)

    def tick(self):
        for future in tuple(self.waiters):
            if not future.done():
                future.set_result(None)
        self.session.poll_worker()

    def destroy_node(self):
        self.session.close()
        return super().destroy_node()


def main():
    os.environ.setdefault("RMW_FASTRTPS_PUBLICATION_MODE", "ASYNCHRONOUS")
    spin(BimanualToolNode, threaded=False)
