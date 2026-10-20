#!/usr/bin/env bash
# Navigation-only public entry; no driver, motor, network or JetPack setup.
set -eo pipefail
bindu_nav_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
bindu_nav_workspace="$bindu_nav_root/artifacts/orin-navigation"
bindu_nav_args=("$@")
for ((i=0;i<${#bindu_nav_args[@]};i++)); do
  case "${bindu_nav_args[i]}" in
    --workspace)
      if ((i+1>=${#bindu_nav_args[@]})); then echo '--workspace 需要目录' >&2; exit 2; fi
      bindu_nav_workspace="${bindu_nav_args[i+1]}" ;;
    --workspace=*) bindu_nav_workspace="${bindu_nav_args[i]#--workspace=}" ;;
  esac
done
# Keep source/import roots stable when invoked from another directory.
cd "$bindu_nav_root"
if [[ -f "$bindu_nav_workspace/env.sh" ]]; then
  source "$bindu_nav_workspace/env.sh"
fi
exec /usr/bin/python3 -m tools.deployment.orin_navigation "$@"
