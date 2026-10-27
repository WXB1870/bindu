#!/usr/bin/env bash
# Legacy interface checks using an explicitly selected installation.
set -eo pipefail
bindu_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd -- "$bindu_root"
for bindu_argument in "$@"; do
  if [[ "$bindu_argument" == --help || "$bindu_argument" == -h ]]; then
    exec python3 -m tools.deployment.snowbot_teleop "$@"
  fi
done
case "${1:-}" in
  configure)
    if [[ -n "${BINDU_TELEOP_WORKSPACE:-}" && -f "$BINDU_TELEOP_WORKSPACE/env.sh" ]]; then
      source "$BINDU_TELEOP_WORKSPACE/env.sh"
    fi
    exec python3 -m tools.deployment.snowbot_teleop "$@" ;;
  -h|--help|'') exec python3 -m tools.deployment.snowbot_teleop "$@" ;;
  build)
    echo '旧专项工具不再单独构建；请使用 python3 -m tools.deployment.x86_real build --workspace /已准备的部署目录。专项配方通过 --config 选择。' >&2
    exit 2 ;;
  check|start|status|gripper) ;;
  *) echo "未知命令：$1；使用 --help 查看用法。" >&2; exit 2 ;;
esac
if [[ -z "${BINDU_TELEOP_WORKSPACE:-}" ]]; then
  echo '旧四节点专项工具需要显式设置 BINDU_TELEOP_WORKSPACE；日常真机启动用 bash tools/run_vr.sh real --enable-commands。' >&2
  exit 2
fi
if [[ -f "$BINDU_TELEOP_WORKSPACE/env.sh" ]]; then
  source "$BINDU_TELEOP_WORKSPACE/env.sh"
  exec python3 -m tools.deployment.snowbot_teleop "$@"
fi
# Compatibility for explicitly selected older installations without env.sh.
bindu_distro="${BINDU_ROS_DISTRO:-${ROS_DISTRO:-}}"
if [[ -z "$bindu_distro" ]]; then
  echo '请显式设置 BINDU_ROS_DISTRO，或先加载选定的 ROS 环境。' >&2
  exit 2
fi
if [[ -n "${ROS_DISTRO:-}" && "$ROS_DISTRO" != "$bindu_distro" ]]; then
  echo "已加载 ROS ${ROS_DISTRO}，与指定的 $bindu_distro 不一致；请使用新终端。" >&2
  exit 2
fi
if [[ ! -f "/opt/ros/$bindu_distro/setup.bash" ]]; then
  echo "缺少 /opt/ros/$bindu_distro/setup.bash；请先安装目标 ROS。" >&2; exit 2
fi
source "/opt/ros/$bindu_distro/setup.bash"
if [[ -n "${BINDU_VENDOR_SETUP:-}" ]]; then
  source "$BINDU_VENDOR_SETUP"
fi
if [[ -n "${BINDU_VENV:-}" ]]; then
  source "$BINDU_VENV/bin/activate"
fi
export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
# Follow the existing wheel environment, with its libraries before ROS EigenPy.
bindu_cmeel="$(python3 - <<'PY'
import os, site
from pathlib import Path
deps = os.environ.get('BINDU_TELEOP_DEPS')
if deps and not Path(deps).is_dir():
    raise SystemExit('BINDU_TELEOP_DEPS 目录不存在')
roots = [deps] if deps else site.getsitepackages()
print(next((str(Path(p).resolve()/'cmeel.prefix') for p in roots
            if (Path(p)/'cmeel.prefix').is_dir()), ''))
PY
)"
if [[ -n "${BINDU_TELEOP_DEPS:-}" ]]; then
  export PYTHONPATH="$BINDU_TELEOP_DEPS${PYTHONPATH:+:$PYTHONPATH}"
fi
if [[ -n "$bindu_cmeel" ]]; then
  bindu_python_version="$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
  export PYTHONPATH="$bindu_cmeel/lib/python$bindu_python_version/site-packages${PYTHONPATH:+:$PYTHONPATH}"
  export LD_LIBRARY_PATH="$bindu_cmeel/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  export CMAKE_PREFIX_PATH="$bindu_cmeel${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
fi
# A pip --target directory may contain obsolete backports such as argparse.
# Keep the selected interpreter's standard library ahead of those backports.
bindu_stdlib="$(python3 -c 'import sysconfig; print(sysconfig.get_path("stdlib"))')"
export PYTHONPATH="$bindu_stdlib${PYTHONPATH:+:$PYTHONPATH}"
if [[ ! -f "$BINDU_TELEOP_WORKSPACE/install/local_setup.bash" ]]; then
  echo '缺少安装环境；请选定已完成两仓构建的 BINDU_TELEOP_WORKSPACE。' >&2; exit 2
fi
source "$BINDU_TELEOP_WORKSPACE/install/local_setup.bash"
exec python3 -m tools.deployment.snowbot_teleop "$@"
