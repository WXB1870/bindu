#!/usr/bin/env bash
# Source this file in a vendor-only terminal. It never starts a ROS node or driver.
# The optional argument is the vendor workspace; defaults match robot 118.
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    echo 'Use: source env_118.sh [/home/cc/snowbot_ws]' >&2
    exit 2
fi

_bindu_vendor_env_118() {
    local bindu_vendor_workspace="${1:-/home/cc/snowbot_ws}"
    local bindu_vendor_file
    if [[ -n "${VIRTUAL_ENV:-}" || -n "${CONDA_PREFIX:-}" ]]; then
        echo 'Use a terminal without a Python venv or Conda environment for the vendor drivers.' >&2
        return 2
    fi
    if [[ "${ROS_DISTRO:-jazzy}" != jazzy ]]; then
        echo 'Robot 118 requires ROS Jazzy; use a terminal without another ROS distro.' >&2
        return 2
    fi
    for bindu_vendor_file in /opt/ros/jazzy/setup.bash \
        "$bindu_vendor_workspace/install/local_setup.bash" \
        /opt/openrobots/lib/libpinocchio_parsers.so.4.1.0 \
        /opt/openrobots/lib/libpinocchio_default.so.4.1.0; do
        if [[ ! -r "$bindu_vendor_file" ]]; then
            echo "Missing vendor dependency: $bindu_vendor_file" >&2
            return 2
        fi
    done
    source /opt/ros/jazzy/setup.bash || return
    source "$bindu_vendor_workspace/install/local_setup.bash" || return
    export LD_LIBRARY_PATH="/opt/openrobots/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
    export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
    export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
}

if _bindu_vendor_env_118 "$@"; then
    unset -f _bindu_vendor_env_118
else
    unset -f _bindu_vendor_env_118
    return 2
fi
