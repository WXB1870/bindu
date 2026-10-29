"""Compatibility import for the installed Snowbot runtime checks."""
from importlib import import_module
import sys
from tools import ROOT

_implementation = import_module('snowbot_bringup.teleop.runtime_check')
_implementation.ROOT = ROOT
sys.modules[__name__] = _implementation
