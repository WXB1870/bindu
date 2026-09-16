from .simulated_joints import SimJointDriver


class SimHandDriver(SimJointDriver):
    """Independent hand endpoint with a slower simulated actuator response.

    This fixture has no coupling, force, tactile, or physical grasp model.
    A real hand adapter may share the position contract while owning its SDK.
    """
    def __init__(self, names, max_speed):
        super().__init__(names, max_speed * .8)
