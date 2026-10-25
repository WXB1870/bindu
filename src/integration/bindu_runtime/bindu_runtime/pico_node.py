"""Compatibility import; implementation is maintained in snowbot_teleop.node."""
from importlib import import_module
import sys

_implementation = import_module('snowbot_teleop.node')
sys.modules[__name__] = _implementation
