"""Compatibility import; implementation is maintained in snowbot_hardware.position_sim."""
from importlib import import_module
import sys

_implementation = import_module('snowbot_hardware.position_sim')
sys.modules[__name__] = _implementation
