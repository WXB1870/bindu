from bindu_contracts.contracts import JointPlan


class PlannerStrategy:
    async def plan(self, profile, group, start, target):
        duration = max(.5, max(abs(a-b) for a, b in zip(start, target)) / profile.max_speed + .1)
        return JointPlan(group, tuple(profile.groups[group]), (duration,), (tuple(target),))
