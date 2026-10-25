"""Compatibility import; implementation is maintained in snowbot_hardware.adapter."""
from importlib import import_module
import sys

_implementation = import_module('snowbot_hardware.adapter')
sys.modules[__name__] = _implementation
