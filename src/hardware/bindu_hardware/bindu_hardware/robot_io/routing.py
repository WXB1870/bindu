class CommandRouter:
    """Route complete named groups; stop all adapters after a partial write."""
    def __init__(self, profile, joint_drivers, base_driver):
        if set(joint_drivers) != set(profile.groups):
            raise ValueError('DRIVER_GROUP_LAYOUT')
        self.groups = profile.groups
        self.joints, self.base = joint_drivers, base_driver
        self.reference = {j: 0. for names in profile.groups.values() for j in names}

    def write_joints(self, target):
        selected = [(g, names) for g, names in self.groups.items() if names and set(names) <= set(target)]
        if not selected or set(target) != {j for _, names in selected for j in names}:
            raise RuntimeError('JOINT_ROUTE_MISMATCH')
        try:
            for group, names in selected:
                self.joints[group].write_positions({j: target[j] for j in names})
        except RuntimeError as exc:
            if len(selected) == 1:
                raise
            # A backend can accept its packet before another rejects. Stop all
            # endpoints and leave the higher-level executor to revoke the lease.
            failures = self.request_stop()
            raise RuntimeError('PARTIAL_WRITE:'+group+':'+str(exc)+
                               (':STOP_FAILED:'+','.join(failures) if failures else '')) from exc
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
