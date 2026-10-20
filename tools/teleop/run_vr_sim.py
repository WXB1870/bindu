#!/usr/bin/env python3
"""Compatibility entry. Use run_vr.py for deployment-selected VR sessions."""
from tools.teleop.run_vr import *
if __name__ == '__main__':
    raise SystemExit(run(parse_args()))
