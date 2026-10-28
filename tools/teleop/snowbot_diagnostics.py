"""Compatibility entry; implementation is maintained by Snowbot."""
from importlib import import_module
import sys

if __name__ == "__main__":
    from runpy import run_module
    run_module('snowbot_bringup.teleop.snowbot_diagnostics', run_name="__main__")
else:
    sys.modules[__name__] = import_module('snowbot_bringup.teleop.snowbot_diagnostics')
