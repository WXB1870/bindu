#!/usr/bin/env bash
# Keep Isaac's Python runtime ahead of ROS/Conda libraries. GUI is the default.
set -e
bindu_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
bindu_isaac="${ISAAC_SIM_PATH:-$HOME/isaacsim/_build/linux-x86_64/release}"
bindu_install="${BINDU_INSTALL:-$bindu_root/install}"
if [ ! -x "$bindu_isaac/python.sh" ] || [ ! -f "$bindu_install/local_setup.bash" ]; then
  echo 'Set ISAAC_SIM_PATH to an Isaac Sim 6.0 runtime and BINDU_INSTALL to a rebuilt Bindu install.' >&2
  exit 1
fi
unset PYTHONPATH LD_LIBRARY_PATH CONDA_PREFIX CONDA_DEFAULT_ENV PYTHONHOME PYTHONEXE
source /opt/ros/jazzy/setup.bash
source "$bindu_install/local_setup.bash"
export PYTHONNOUSERSITE=1
export PYTHONPATH="$bindu_isaac/kit/python/lib/python3.12:$PYTHONPATH"
export LD_LIBRARY_PATH="$bindu_isaac/kit/python/lib:$LD_LIBRARY_PATH"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-125}" ROS_LOCALHOST_ONLY=1
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
exec "$bindu_isaac/python.sh" -u "$bindu_root/tools/run_g1_isaac.py" "$@"
