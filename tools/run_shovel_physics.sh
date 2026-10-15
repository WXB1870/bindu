#!/usr/bin/env bash
# Isaac 6.2, no ROS: align bundled NVIDIA libraries inside this process only.
set -euo pipefail
bindu_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
bindu_sdk="${ISAAC_SIM_PATH:-$HOME/isaacsim-6.2.0}"
bindu_state="${BINDU_ISAAC_STATE:-$bindu_root/artifacts/isaac62-state}"
bindu_script="$bindu_root/tools/run_shovel_physics.py"
if [ "${1:-}" = --kinematic ]; then
  bindu_script="$bindu_root/tests/validate_shovel_isaac.py"
  shift
fi
test -x "$bindu_sdk/python.sh"
mkdir -p "$bindu_state"/{kit,xdg_cache,xdg_config,xdg_data,cuda_cache,tmp}
unset PYTHONPATH PYTHONHOME PYTHONEXE LD_LIBRARY_PATH LD_PRELOAD
unset CARB_APP_PATH EXP_PATH ISAAC_PATH OLD_PYTHONPATH
unset CONDA_PREFIX CONDA_DEFAULT_ENV VIRTUAL_ENV AMENT_PREFIX_PATH COLCON_PREFIX_PATH
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_PACKAGE_PATH CUDA_HOME CUDA_PATH
export PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
export XDG_CACHE_HOME="$bindu_state/xdg_cache" XDG_CONFIG_HOME="$bindu_state/xdg_config"
export XDG_DATA_HOME="$bindu_state/xdg_data" CUDA_CACHE_PATH="$bindu_state/cuda_cache"
export TMPDIR="$bindu_state/tmp"
bindu_nv_dirs=""
for bindu_lib in "$bindu_sdk/exts/isaacsim.pip.nv/pip_prebundle/nvidia"/*/lib; do
  test -d "$bindu_lib" || continue
  bindu_nv_dirs="${bindu_nv_dirs:+$bindu_nv_dirs:}$bindu_lib"
done
test -n "$bindu_nv_dirs"
export LD_LIBRARY_PATH="$bindu_nv_dirs"
exec "$bindu_sdk/python.sh" --no-ros-env "$bindu_script" "$@" \
  --portable --portable-root "$bindu_state/kit" --/isaac/startup/ros_bridge_extension=
