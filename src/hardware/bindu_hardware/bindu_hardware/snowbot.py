"""Compatibility import; implementation is maintained in snowbot_hardware.protocol."""
from importlib import import_module
import sys

_implementation = import_module('snowbot_hardware.protocol')
sys.modules[__name__] = _implementation
