"""ROS/NumPy-independent types. Poses are row-major 4x4, metres and radians.

Input stamps mark event reception (not the headset capture time). Never refresh
them when polling. generation fences IK results across clutch/re-anchor events.
"""
from dataclasses import dataclass
from typing import Tuple, Protocol


@dataclass(frozen=True)
class VRFrame:
    source_id: str
    seq: int
    stamp: float
    side: str
    pose: Tuple[float, ...]
    grip: float = 0.
    trigger: float = 0.
    init: bool = False
    stop: bool = False
    valid: bool = True
    clutch_seq: int = 0  # Release counter survives latest-frame queue coalescing.


@dataclass(frozen=True)
class IKRequest:
    request_id: str
    generation: int
    stamp: float
    expires: float
    names: Tuple[str, ...]
    seed: Tuple[float, ...]
    target: Tuple[float, ...] = ()  # Empty requests FK only.
    context: Tuple[float, ...] = ()  # Measured non-active joints, sorted by name.


@dataclass(frozen=True)
class IKResult:
    request: IKRequest
    success: bool
    code: str
    positions: Tuple[float, ...] = ()
    pose: Tuple[float, ...] = ()
    position_error: float = 0.
    rotation_error: float = 0.
    elapsed: float = 0.


class Kinematics(Protocol):
    def solve(self, request: IKRequest) -> IKResult: ...
