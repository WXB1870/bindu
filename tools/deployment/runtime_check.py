"""Compatibility import for the installed Snowbot runtime checks."""
from importlib import import_module
import sys

_implementation = import_module('snowbot_bringup.runtime.runtime_check')
sys.modules[__name__] = _implementation
