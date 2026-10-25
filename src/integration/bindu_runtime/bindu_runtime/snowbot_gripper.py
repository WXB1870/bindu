"""Compatibility import; implementation is maintained in snowbot_hardware.gripper_node."""
from importlib import import_module
import sys

_implementation = import_module('snowbot_hardware.gripper_node')
sys.modules[__name__] = _implementation
