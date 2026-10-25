"""Compatibility import; implementation is maintained in snowbot_hardware.session."""
from importlib import import_module
import sys

_implementation = import_module('snowbot_hardware.session')
sys.modules[__name__] = _implementation
