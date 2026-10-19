#!/usr/bin/env bash
# One deployment-selected entry for VR control and its operator view.
set -eo pipefail
bindu_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd -- "$bindu_root"
for bindu_argument in "$@"; do
  if [[ "$bindu_argument" == --help || "$bindu_argument" == -h ]]; then
    exec python3 tools/run_vr.py "$@"
  fi
done
bindu_env="${BINDU_VR_ENV:-$bindu_root/artifacts/vr/env.sh}"
if [[ -f "$bindu_env" ]]; then
  source "$bindu_env"
elif [[ -n "${BINDU_VR_ENV:-}" || -z "${ROS_DISTRO:-}" ]]; then
  echo '未找到 VR 环境。请先加载匹配的 ROS / 工作区 / 数值依赖，或设置 BINDU_VR_ENV。' >&2
  exit 2
fi
bindu_viewer_deps="${BINDU_VIEWER_DEPS:-}"
if [[ -z "$bindu_viewer_deps" ]]; then
  for bindu_candidate in "$bindu_root/artifacts/snowbot-viewer-deps" "$bindu_root/artifacts/vr-live-2026-10-18/viewer-deps"; do
    if [[ -d "$bindu_candidate/meshcat" ]]; then
      bindu_viewer_deps="$bindu_candidate"; break
    fi
  done
fi
if [[ -n "$bindu_viewer_deps" ]]; then
  export PYTHONPATH="$bindu_viewer_deps${PYTHONPATH:+:$PYTHONPATH}"
fi
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
exec python3 tools/run_vr.py "$@"
