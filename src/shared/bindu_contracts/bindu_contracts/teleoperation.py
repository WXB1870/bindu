"""ROS/NumPy-independent types. Poses are row-major 4x4, metres and radians.

Input stamps mark event reception (not the headset capture time). Never refresh
them when polling. generation fences IK results across clutch/re-anchor events.
"""
from dataclasses import dataclass
from typing import Tuple, Protocol
import math


def reference_tracking_budget(reference, measured, limit, *, executed=None, reserve_fraction=.8):
    """Separate a target update from its executor-owned moving reference.

    Only an authenticated, fresh executor sample may be supplied as executed.
    An existing target gap outside the normal reserve may then converge, but
    no joint may grow that gap. Generation uses a smaller reserve than admission
    to leave room for measured feedback to advance while solving. Integration
    still starts at the accepted goal.
    """
    if not math.isfinite(limit) or limit<=0:
        raise ValueError('BIMANUAL_REFERENCE_LIMIT_CONFIG')
    if (not reference or len(reference)!=len(measured) or
            executed is not None and len(executed)!=len(reference) or
            not all(math.isfinite(q) for row in (reference,measured,
                reference if executed is None else executed) for q in row)):
        raise ValueError('BIMANUAL_REFERENCE_LAYOUT')
    tracking=reference if executed is None else executed
    error=max(abs(q-m) for q,m in zip(tracking,measured))
    budgets=tuple(max(reserve_fraction*limit,abs(q-m)) if executed is not None else reserve_fraction*limit
                  for q,m in zip(reference,measured))
    return error,budgets


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
    stick: Tuple[float, ...] = ()  # WebXR thumbstickValue (x,y); absent is unknown.
    head_pose: Tuple[float, ...] = ()  # Same OpenXR reference space as this controller.
    head_stamp: float = 0.  # First receiver observation of this head sample.
    head_age_s: float = 0.  # Browser sample age at that first observation.
    head_source_id: str = ''
    head_seq: int = 0



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
