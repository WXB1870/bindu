"""Local teleoperation lifecycle and results, independent of ROS transport."""

from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class LeaseRejection:
    instance_id: str
    epoch: int
    profile_hash: str
    reason: str


@dataclass
class SessionResult:
    simulated: bool
    success: bool = False
    code: str = ""
    accepted_commands: int = 0
    lease_rejection: Optional[LeaseRejection] = None


class SessionOperation:
    """Local counterpart of the ROS goal's feedback and cancellation API."""

    def __init__(self, request, feedback):
        self.request = request
        self.feedback = feedback
        self.is_cancel_requested = False
        self.status = "running"

    def cancel(self):
        self.is_cancel_requested = True

    def publish_feedback(self, feedback):
        self.feedback(feedback)

    def succeed(self):
        self.status = "succeeded"

    def canceled(self):
        self.status = "canceled"

    def abort(self):
        self.status = "failed"
