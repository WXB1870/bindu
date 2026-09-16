import math


class Scene:
    """Task-owned facts; simulation evidence must remain explicitly simulated."""
    def __init__(self):
        self.observation = None
        self.held_object = None

    def accept(self, observation, now):
        if (not observation.identity or observation.frame != 'map' or
                not 0 <= now - observation.stamp <= .5 or
                len(observation.position) != 3 or not all(math.isfinite(x) for x in observation.position)):
            raise ValueError('INVALID_OBSERVATION')
        self.observation = observation

    def confirm_grasp(self, object_id, positions, names, targets):
        if not self.observation or self.observation.object_id != object_id:
            raise ValueError('NO_OBJECT_EVIDENCE')
        if (not names or len(names) != len(targets) or
                any(j not in positions or not math.isfinite(positions[j]) or not math.isfinite(t) or
                    abs(positions[j] - t) > .025 for j, t in zip(names, targets))):
            raise ValueError('GRASP_NOT_CONFIRMED')
        if not self.observation.simulated:
            raise ValueError('SIMULATED_GRASP_EVALUATOR_ONLY')
        self.held_object = object_id
