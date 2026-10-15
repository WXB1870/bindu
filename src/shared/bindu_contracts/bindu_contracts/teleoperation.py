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
    mapping: str = 'v34'
    buttons: int = 0  # primary, secondary, thumbstick-click; physical side is explicit.
    button_edges: Tuple[int, ...] = (0,)*8  # press/release counters for those 3 + Trigger.
    virtual_grip: bool = False  # Coordinated PICO X request, not right Trigger.



@dataclass(frozen=True)
class ToolPoseFrame:
    """Absolute tool pose in the robot base; independent of VR and task phases.

    VR mapping combines clutch-relative deltas with the measured tool anchor
    before producing this target. No raw headset/controller pose is implied.
    tight=False means right axial support, not releasing the tool. The left
    grip remains locked. Source/sequence/time are transport safety metadata.
    """
    source_id: str
    seq: int
    stamp: float
    frame_id: str
    pose: Tuple[float, ...]
    tight: bool
    valid: bool = True
    stop: bool = False


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
