"""Structural interfaces implemented by replaceable capability adapters."""
from typing import Dict, Protocol, Tuple
from .contracts import Feedback, JointPlan


class GraspStrategy(Protocol):
    async def plan(self, profile, group, start, target) -> JointPlan: ...


class NavigationProvider(Protocol):
    async def navigate(self, port, context, site: str) -> None: ...
