"""One execution transport and feedback cache per application node."""

from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Trigger
from bindu_interfaces.msg import ExecutionState
from bindu_interfaces.srv import Lease, SubmitMotion, ControlExecution


def parameter(node, name, default, **kwargs):
    """Shared components read the host's existing immutable configuration."""
    if node.has_parameter(name):
        return node.get_parameter(name)
    return node.declare_parameter(name, default, **kwargs)


class ExecutionClient:
    def __init__(self, node):
        self.state = None
        self.node = node
        group = ReentrantCallbackGroup()
        self.leases = node.create_client(Lease, "execution/lease", callback_group=group)
        self.submit = node.create_client(SubmitMotion, "execution/submit", callback_group=group)
        self.control = node.create_client(
            ControlExecution, "execution/control", callback_group=group
        )
        self.held_reference = node.create_client(
            Trigger, "execution/held_reference", callback_group=group
        )
        node.create_subscription(ExecutionState, "execution/state", self.receive, 1)

    def receive(self, message):
        self.state = (
            message
            if (
                self.node.matches_mode(message) and message.profile_hash == self.node.profile.digest
            )
            else None
        )
