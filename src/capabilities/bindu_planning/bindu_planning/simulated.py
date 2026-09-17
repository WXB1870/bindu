from bindu_contracts.contracts import JointPlan


class PlannerStrategy:
    async def plan(self, profile, group, start, target):
        duration = max(.5, 1.05*max(max(1.875*abs(a-b)/profile.speed(n),
            (5.774*abs(a-b)/profile.acceleration(n))**.5,
            (60*abs(a-b)/profile.jerk(n))**(1/3))
            for n, a, b in zip(profile.groups[group], start, target)))
        return JointPlan(group, tuple(profile.groups[group]), (duration,), (tuple(target),))
