from dataclasses import dataclass
from typing import Tuple


@dataclass(frozen=True)
class ObjectEstimate:
    found: bool
    frame: str
    position: Tuple[float, float, float]


class SimObjectLocator:
    """Known-object fixture; does not consume images or estimate a real pose."""
    def locate(self, object_id):
        return ObjectEstimate(object_id == 'drink', 'map', (.4, 0., .8))
