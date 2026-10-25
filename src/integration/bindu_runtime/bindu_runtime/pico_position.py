"""Compatibility import; implementation is maintained in snowbot_teleop.position."""
from importlib import import_module
import sys

_implementation = import_module('snowbot_teleop.position')
sys.modules[__name__] = _implementation
