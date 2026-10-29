"""Compatibility import for the installed Snowbot display scene."""
from importlib import import_module
import sys

_implementation = import_module('snowbot_bringup.simulation.kinematic_scene')
sys.modules[__name__] = _implementation
