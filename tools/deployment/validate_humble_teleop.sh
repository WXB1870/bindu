#!/usr/bin/env bash
# Build and test the simulated teleop assembly in Ubuntu 22.04 / ROS 2 Humble.
set -eo pipefail

bindu_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
output_path="${1:-$bindu_root/artifacts/humble-teleop-$(date +%Y%m%d-%H%M%S)}"
mkdir -p -- "$output_path"
output_path="$(cd -- "$output_path" && pwd)"

docker run --rm -i \
  --env "RESULT_UID=$(id -u)" --env "RESULT_GID=$(id -g)" \
  --mount "type=bind,src=$bindu_root,dst=/workspace,readonly" \
  --mount "type=bind,src=$output_path,dst=/validation" \
  ros:humble-ros-base bash -s <<'CONTAINER'
set -eo pipefail
export DEBIAN_FRONTEND=noninteractive PYTHONDONTWRITEBYTECODE=1
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 CMAKE_BUILD_PARALLEL_LEVEL=2
export PIP_CACHE_DIR=/validation/pip-cache
trap 'chown -R "$RESULT_UID:$RESULT_GID" /validation' EXIT

apt-get update
apt-get install -y --no-install-recommends python3-venv python3-dev python3-scipy \
  python3-zmq python3-yaml libeigen3-dev libsimde-dev \
  ros-humble-nav2-msgs ros-humble-robot-state-publisher
# Keep wheel dependencies separate from ROS system Python packages.
python3 -m venv --system-site-packages /validation/venv
source /validation/venv/bin/activate
# CMake < 3.30 can overwrite PACKAGE_PREFIX_DIR while finding Eigen/Simde,
# breaking the installed ProxSuite Python-module target path.
python3 -m pip install -r /workspace/tools/requirements/requirements-teleop.txt \
  proxsuite==0.7.3 scipy==1.15.3 cmake==3.31.6
source /opt/ros/humble/setup.bash
cmeel_prefix="$(python3 -c 'from pathlib import Path; import site; print(next(str(Path(p)/"cmeel.prefix") for p in site.getsitepackages() if (Path(p)/"cmeel.prefix").is_dir()))')"
export CMAKE_PREFIX_PATH="$cmeel_prefix:$CMAKE_PREFIX_PATH"
export LD_LIBRARY_PATH="$cmeel_prefix/lib:$LD_LIBRARY_PATH"
python3 -c 'import numpy, pinocchio, casadi; print("Dependencies:", numpy.__version__, pinocchio.__version__, casadi.__version__)'

cd /workspace
python3 /usr/bin/colcon --log-base /validation/log build --base-paths src \
  --packages-up-to bindu_runtime bindu_bimanual_qp \
  --build-base /validation/build --install-base /validation/install \
  --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  "-DPython3_EXECUTABLE=$(command -v python3)" "-DPYTHON_EXECUTABLE=$(command -v python3)"
source /validation/install/local_setup.bash
PYTHONPATH="/workspace:$PYTHONPATH" python3 -m unittest -v \
  tests.unit.test_teleop_modes tests.unit.test_teleoperation tests.unit.test_teleop_deployment tests.unit.test_one_euro \
  tests.unit.test_bimanual_qp > /validation/unit-tests.log 2>&1
python3 - <<'PYTEST'
import importlib.metadata, json, platform, re, sys
from pathlib import Path
import numpy, pinocchio, rclpy
log = Path('/validation/unit-tests.log').read_text()
count = int(re.search(r'Ran (\d+) tests?', log).group(1))
skipped = len(re.findall(r'\.\.\. skipped ', log))
if skipped:
    raise RuntimeError('Humble acceptance requires all selected tests to run')
Path('/validation/environment.json').write_text(json.dumps({
    'os': Path('/etc/os-release').read_text(), 'python': sys.version,
    'architecture': platform.machine(), 'ros': 'Humble', 'numpy': numpy.__version__,
    'pinocchio': pinocchio.__version__, 'rclpy': rclpy.__file__,
    'dependencies': {name: importlib.metadata.version(name) for name in
        ('cmake', 'proxsuite', 'cmeel-urdfdom', 'cmeel-tinyxml2', 'scipy')},
    'tests': count, 'failures': 0, 'errors': 0, 'skipped': skipped,
    'scope': 'Ubuntu 22.04 x86_64 container; no ARM64 or real robot'}, indent=2)+'\n')
print('Humble unit tests:', count, 'passed; skipped:', skipped)
PYTEST
ctest --test-dir /validation/build/bindu_bimanual_qp --output-on-failure
ROS_DOMAIN_ID=181 ROS_LOCALHOST_ONLY=1 python3 -m tests.integration.validate_bimanual \
  --snowbot --axial-grasp --tool-pose-input --pause-resume --drop-tool-input \
  --tool-display --output /validation/ros-resume-drop
CONTAINER
