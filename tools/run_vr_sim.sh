#!/usr/bin/env bash
# Compatibility entry; select the device through the same deployment config.
exec bash "$(dirname -- "${BASH_SOURCE[0]}")/run_vr.sh" "$@"
