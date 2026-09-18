"""Nonblocking device contracts in SI units; no task or trajectory ownership."""
from dataclasses import dataclass
from typing import Dict, Protocol, Tuple


@dataclass(frozen=True)
class JointReading:
    stamp: float
    positions: Dict[str, float]
    simulated: bool


@dataclass(frozen=True)
class BaseReading:
    stamp: float
    x: float
    yaw: float
    velocity: Tuple[float, float]
    simulated: bool
    y: float = 0.


class JointPositionDriver(Protocol):
    def write_positions(self, target: Dict[str, float]) -> None: ...
    def request_stop(self) -> None: ...
    def read(self, now: float, dt: float) -> JointReading: ...


class BaseVelocityDriver(Protocol):
    def write_velocity(self, velocity: Tuple[float, float]) -> None: ...
    def request_stop(self) -> None: ...
    def read(self, now: float, dt: float) -> BaseReading: ...
