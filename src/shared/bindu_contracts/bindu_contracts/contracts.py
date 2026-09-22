"""ROS-independent immutable commands, observations and feedback contracts."""
from dataclasses import dataclass, field
from typing import Dict, Tuple


@dataclass(frozen=True)
class Motion:
    command_id: str
    lease_id: str
    epoch: int
    profile_hash: str
    task_id: str
    observation_id: str
    mode: str
    group: str
    stamp: float
    valid_for: float
    names: Tuple[str, ...] = ()
    positions: Tuple[float, ...] = ()
    offsets: Tuple[float, ...] = ()
    points: Tuple[Tuple[float, ...], ...] = ()
    velocity: Tuple[float, float] = (0., 0.)
    duration: float = 0.
    schema_version: int = 1
    velocities: Tuple[Tuple[float, ...], ...] = ()
    accelerations: Tuple[Tuple[float, ...], ...] = ()
    expected_revision: int = 0  # 0 disables the check; chunks require a current revision.


@dataclass
class Feedback:
    stamp: float
    positions: Dict[str, float]
    base_x: float
    base_yaw: float
    base_velocity: Tuple[float, float]
    simulated: bool = True
    base_y: float = 0.
    joint_velocities: Dict[str, float] = field(default_factory=dict)


@dataclass(frozen=True)
class Event:
    stamp: float
    task_id: str
    command_id: str
    state: str
    code: str
    observation_id: str = ''


@dataclass
class Observation:
    identity: str
    object_id: str
    stamp: float
    frame: str
    position: Tuple[float, float, float]
    simulated: bool


@dataclass(frozen=True)
class JointPlan:
    group: str
    names: Tuple[str, ...]
    offsets: Tuple[float, ...]
    points: Tuple[Tuple[float, ...], ...]
    velocities: Tuple[Tuple[float, ...], ...] = ()
    accelerations: Tuple[Tuple[float, ...], ...] = ()
