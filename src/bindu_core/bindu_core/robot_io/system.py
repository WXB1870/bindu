from .routing import CommandRouter
from .feedback import FeedbackCollector


class RobotIO:
    """Thin composition of command and feedback ports; owns no device physics."""
    def __init__(self, profile, joint_drivers, base_driver):
        self.commands = CommandRouter(profile, joint_drivers, base_driver)
        self.feedback = FeedbackCollector(profile, joint_drivers, base_driver)
        self.stop_failures = ()

    @property
    def last_feedback(self):
        return self.feedback.last

    @property
    def reference_positions(self):
        return dict(self.commands.reference)

    def write_joints(self, target):
        self.commands.write_joints(target)

    def write_base(self, velocity):
        self.commands.write_base(velocity)

    def stop(self):
        self.stop_failures = self.commands.request_stop()
        # Stop is a separate operation; no position reference remains active.
        self.commands.reference.clear()

    def read_feedback(self, now, dt):
        return self.feedback.read(now, dt)
