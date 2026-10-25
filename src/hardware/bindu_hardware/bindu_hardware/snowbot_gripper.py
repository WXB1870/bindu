"""Compatibility import; implementation is maintained in snowbot_hardware.gripper."""
from importlib import import_module
import sys

_implementation = import_module('snowbot_hardware.gripper')
sys.modules[__name__] = _implementation
