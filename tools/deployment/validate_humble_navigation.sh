#!/usr/bin/env bash
# Isolated x86 Humble verification; no real hardware, no host package changes.
set -eo pipefail
bindu_nav_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
bindu_nav_output="${1:-$bindu_nav_root/artifacts/humble-navigation-$(date +%Y%m%d-%H%M%S)}"
if [[ -e "$bindu_nav_output" ]]; then
  echo "请使用新的验证目录，保留已有批次：$bindu_nav_output" >&2; exit 2
fi
mkdir -p -- "$bindu_nav_output"
bindu_nav_output="$(cd -- "$bindu_nav_output" && pwd)"
docker run --rm -i --env "RESULT_UID=$(id -u)" --env "RESULT_GID=$(id -g)" \
  --mount "type=bind,src=$bindu_nav_root,dst=/workspace,readonly" \
  --mount "type=bind,src=$bindu_nav_output,dst=/validation" \
  ros:humble-ros-base bash -s <<'CONTAINER'
set -eo pipefail
export DEBIAN_FRONTEND=noninteractive BINDU_NAVIGATION_ONLY=1 PYTHONDONTWRITEBYTECODE=1
export CMAKE_BUILD_PARALLEL_LEVEL=2 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
trap 'chown -R "$RESULT_UID:$RESULT_GID" /validation' EXIT
apt-get update > /validation/apt.log 2>&1
apt-get install -y --no-install-recommends python3-dev python3-numpy python3-yaml \
  ros-humble-nav2-msgs ros-humble-sensor-msgs-py >> /validation/apt.log 2>&1
source /opt/ros/humble/setup.bash
cd /workspace
colcon --log-base /validation/log list --base-paths src --packages-up-to bindu_runtime --names-only > /validation/packages.txt
python3 /usr/bin/colcon --log-base /validation/log build --base-paths src \
  --packages-up-to bindu_runtime --build-base /validation/build --install-base /validation/install \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3 > /validation/build.log 2>&1
source /validation/install/local_setup.bash
export PYTHONPATH="/workspace:$PYTHONPATH" ROS_DOMAIN_ID=197 ROS_LOCALHOST_ONLY=1
python3 -m unittest -v tests.unit.test_orin_navigation tests.unit.test_navigation \
  tests.unit.test_execution_fault_sequences tests.unit.test_runtime_mode \
  > /validation/unit-tests.log 2>&1
python3 -m tests.integration.validate_nav2_identity \
  --binary /validation/install/bindu_navigation/lib/bindu_navigation/nav2_velocity_identity \
  --output /validation/identity > /validation/identity.log 2>&1
python3 -m tests.integration.validate_orin_navigation --output /validation/preflight > /validation/preflight.log 2>&1
CONTAINER
