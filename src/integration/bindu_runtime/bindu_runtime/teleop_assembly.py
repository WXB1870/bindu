"""Compatibility import; implementation is maintained in snowbot_bringup.assembly."""
from importlib import import_module
import sys

_implementation = import_module('snowbot_bringup.assembly')
sys.modules[__name__] = _implementation
