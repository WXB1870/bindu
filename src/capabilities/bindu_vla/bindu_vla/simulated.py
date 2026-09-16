from bindu_core.contracts import JointPlan


class ChunkStrategy:
    """Deterministic 100 Hz chunk stand-in; deliberately contains no VLA model."""
    async def plan(self, profile, group, start, target):
        duration = max(.5, max(abs(a-b) for a, b in zip(start, target)) / profile.max_speed + .1)
        count = int(duration * 100) + 1
        times = tuple((i+1) / 100 for i in range(count))
        points = tuple(tuple(a + (b-a)*(i+1)/count for a,b in zip(start,target)) for i in range(count))
        return JointPlan(group, tuple(profile.groups[group]), times, points)
