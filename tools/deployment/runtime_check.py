"""Compatibility import for the installed Snowbot runtime checks."""
from importlib import import_module
import sys

_implementation = import_module('snowbot_bringup.runtime.runtime_check')
if __name__ == '__main__':
    raise SystemExit(_implementation.main())
else:
    sys.modules[__name__] = _implementation
