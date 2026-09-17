from bindu_contracts.contracts import JointPlan


class ChunkStrategy:
    """Synthetic timed action knots, including their derivatives; no VLA model.

    This analytic test trajectory exercises the shared sampler without a second
    100 Hz position filter or stopping at each sample.
    """
    async def plan(self, profile, group, start, target):
        duration = max(.5, 1.05*max(max(1.875*abs(a-b)/profile.speed(n),
            (5.774*abs(a-b)/profile.acceleration(n))**.5,
            (60*abs(a-b)/profile.jerk(n))**(1/3))
            for n, a, b in zip(profile.groups[group], start, target)))
        times, points, velocities, accelerations = [], [], [], []
        for u in (.25, .5, .75, 1.):
            times.append(duration*u)
            points.append(tuple(a+(b-a)*(10*u**3-15*u**4+6*u**5) for a, b in zip(start, target)))
            velocities.append(tuple((b-a)*(30*u**2-60*u**3+30*u**4)/duration for a, b in zip(start, target)))
            accelerations.append(tuple((b-a)*(60*u-180*u**2+120*u**3)/duration**2 for a, b in zip(start, target)))
        return JointPlan(group, tuple(profile.groups[group]), tuple(times), tuple(points),
                         tuple(velocities), tuple(accelerations))
