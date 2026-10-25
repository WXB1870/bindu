"""Compatibility import; implementation is maintained in snowbot_teleop.control."""
from importlib import import_module
import sys

_implementation = import_module('snowbot_teleop.control')
sys.modules[__name__] = _implementation
