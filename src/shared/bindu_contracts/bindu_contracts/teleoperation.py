"""ROS/NumPy-independent types. Poses are row-major 4x4, metres and radians.

Input stamps mark event reception (not the headset capture time). Never refresh
them when polling. generation fences IK results across clutch/re-anchor events.
"""
from dataclasses import dataclass
from typing import Tuple, Protocol
import math
import json
from pathlib import Path
from .resources import resource_file


def tool_with_timing(config, teleop, solver):
    return dict(config, input_max_age=teleop['input_max_age_s'],
                feedback_max_age=teleop['feedback_max_age_s'], command_max_age=teleop['command_ttl_s'],
                worker_timeout=solver['worker_timeout_s'], worker_startup_timeout=solver['startup_timeout_s'],
                solve_timeout_us=round(solver['timeout_s'] * 1e6))


def load_tool_config(path, *, sources=None):
    """Read standalone tool settings, resolving shared defaults when declared."""
    config = json.loads(Path(path).read_text())
    if sources is not None:
        sources.append(Path(path))
    if 'timing_defaults' in config:
        timing_path = resource_file(config.pop('timing_defaults'), owner=path)
        timing = json.loads(timing_path.read_text())
        if sources is not None:
            sources.append(timing_path)
        config = tool_with_timing(config, timing['teleop'], timing['solver'])
    return config


def tool_grasp_mode(config):
    """Select a continuous measured axial grasp without changing gripper opening."""
    source=config.get('grasp_reference_source','configured')
    if source not in ('configured','measured_rigid','measured_axial'):
        raise ValueError('BIMANUAL_GRASP_REFERENCE_SOURCE')
    if source=='measured_axial':
        if config.get('right_axial_grasp') is None:
            raise ValueError('BIMANUAL_AXIAL_GEOMETRY_REQUIRED')
        if config.get('measured_grasp_policy','enforce')!='enforce':
            raise ValueError('BIMANUAL_MEASURED_AXIAL_REQUIRES_HELD_GRASP')
        return 'right_axial'
    return 'rigid'


def joint_reference_limits(names, default, overrides=None):
    """Resolve named hard tracking bounds; unspecified joints retain the default."""
    overrides = {} if overrides is None else overrides
    if (isinstance(default,bool) or not isinstance(default,(int,float)) or
            not math.isfinite(default) or default<=0 or not isinstance(overrides,dict) or
            set(overrides)-set(names) or any(isinstance(v,bool) or
                not isinstance(v,(int,float)) or not math.isfinite(v) or v<=0
                for v in overrides.values())):
        raise ValueError('BIMANUAL_REFERENCE_LIMIT_CONFIG')
    return tuple(overrides.get(name,default) for name in names)


def reference_tracking_budget(reference, measured, limit, *, executed=None, reserve_fraction=.8,
                              target_limit=None, joint_limits=None):
    """Separate a target update from its executor-owned moving reference.

    Only an authenticated, fresh executor sample may be supplied as executed.
    An existing target gap outside the normal reserve may then converge, but
    no joint may grow that gap. Generation uses a smaller reserve than admission
    to leave room for measured feedback to advance while solving. Integration
    still starts at the accepted goal. target_limit bounds candidate advance
    separately from the hard executed/measured limit; existing gaps may only
    remain or shrink outside that reserve.
    """
    if not math.isfinite(limit) or limit<=0:
        raise ValueError('BIMANUAL_REFERENCE_LIMIT_CONFIG')
    target_limit = limit if target_limit is None else target_limit
    if (isinstance(target_limit, bool) or not isinstance(target_limit, (int, float)) or
            not math.isfinite(target_limit) or
            not 0 < target_limit <= limit):
        raise ValueError('BIMANUAL_TARGET_LEAD_LIMIT_CONFIG')
    if (not reference or len(reference)!=len(measured) or
            executed is not None and len(executed)!=len(reference) or
            not all(math.isfinite(q) for row in (reference,measured,
                reference if executed is None else executed) for q in row)):
        raise ValueError('BIMANUAL_REFERENCE_LAYOUT')
    tracking=reference if executed is None else executed
    if joint_limits is not None:
        if (len(joint_limits)!=len(reference) or any(isinstance(v,bool) or
                not isinstance(v,(int,float)) or not math.isfinite(v) or
                not target_limit<=v<=limit for v in joint_limits)):
            raise ValueError('BIMANUAL_REFERENCE_LIMIT_CONFIG')
        if any(abs(q-m)>bound for q,m,bound in zip(tracking,measured,joint_limits)):
            raise ValueError('BIMANUAL_REFERENCE_TRACKING_ERROR')
    error=max(abs(q-m) for q,m in zip(tracking,measured))
    budgets=tuple(max(reserve_fraction*target_limit,abs(q-m)) if executed is not None else reserve_fraction*target_limit
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
