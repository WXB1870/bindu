"""Compatibility entry; implementation is maintained by Snowbot."""
from importlib import import_module
import sys
from pathlib import Path
import os
os.environ.setdefault("BINDU_FRAMEWORK_SOURCE", str(Path(__file__).resolve().parents[2]))

if __name__ == "__main__":
    from runpy import run_module
    run_module('snowbot_bringup.teleop.run_vr', run_name="__main__")
else:
    sys.modules[__name__] = import_module('snowbot_bringup.teleop.run_vr')
