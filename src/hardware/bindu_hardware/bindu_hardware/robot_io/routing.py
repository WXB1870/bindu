class CommandRouter:
    """Route one group's command; never split a command into partial writes."""
    def __init__(self, profile, joint_drivers, base_driver):
        if set(joint_drivers) != set(profile.groups):
            raise ValueError('DRIVER_GROUP_LAYOUT')
        self.groups = profile.groups
        self.joints, self.base = joint_drivers, base_driver
        self.reference = {j: 0. for names in profile.groups.values() for j in names}

    def write_joints(self, target):
        group = next((g for g, names in self.groups.items() if set(names) == set(target)), None)
        if group is None:
            raise RuntimeError('JOINT_ROUTE_MISMATCH')
        self.joints[group].write_positions(target)
        self.reference.update(target)  # Update only after the adapter accepts.

    def write_base(self, velocity):
        self.base.write_velocity(velocity)

    def request_stop(self):
        failures = []
        # One failed endpoint must not prevent stopping the remaining devices.
        for name, driver in list(self.joints.items()) + [('base', self.base)]:
            try:
                driver.request_stop()
            except RuntimeError:
                failures.append(name)
        return tuple(failures)
