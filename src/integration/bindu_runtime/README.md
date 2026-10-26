# bindu_runtime：ROS 运行装配

[项目模块导航](../../../README.md#模块快速导航) · [编译与运行](#编译与运行) · [职责与边界](#职责与边界) · [配置目录](#配置目录) · [扫雪遥操作](#扫雪遥操作) · [源码导航](#源码导航) · [接入与验证](#接入与验证) · [关联模块](#关联模块)

提供 ROS 2 节点、launch 与配置，把公共契约、执行库、设备和能力组合为可启动的运行实例。算法核心仍在各自模块中维护。

## 按任务查阅

| 要修改或核对的内容 | 相关章节 |
|---|---|
| 场景选择、配置与参数接线 | [配置目录](#配置目录)、[扫雪遥操作](#扫雪遥操作) |
| 初次构建、数值依赖与ABI | [编译与运行](#编译与运行) |
| 既有x86部署的日常操作 | [日常使用](#日常使用) |
| 新机迁移或旧位置接口专项 | [首次部署](#首次部署)、[现场部署顺序](#现场部署顺序) |
| 节点实现与相关验证 | [源码导航](#源码导航)、[接入与验证](#接入与验证) |

各行按任务选择；历史最小接口步骤无需作为日常部署的前置阅读。

## 编译与运行

以下命令均在仓库根目录执行，使用与 ROS 消息 ABI 匹配的系统 Python。示例为 Ubuntu 24.04／Jazzy；Ubuntu 22.04／Humble 将 `jazzy` 换为 `humble`，不要在同一终端混用发行版。基础 ROS、colcon、rosdep 和各包依赖按 [环境准备](../../../运行与验证.md#环境准备与构建) 安装。

### 依赖准备

**导航专用的Orin环境**使用[导航专用入口](../../capabilities/bindu_navigation/README.md#orin-导航专用部署与新手操作)，设置`BINDU_NAVIGATION_ONLY=1`收敛为七包（LIO另加一包）。该配置只供导航launch，不包含VR／IK；Snowbot位置遥操作同样以Orin为目标，需要按完整遥操作依赖另行构建，ARM64链路尚未验证。

默认完整装配依赖运动学及原生 QP，即使只启动任务示例，首次构建也要准备 C++17、Python 开发头文件、Eigen3、SIMDe，以及匹配的 Pinocchio／ProxSuite。已有可用数值环境可直接加载；若使用仓库固定版本的 wheel，先在新的终端准备隔离目录：

```bash
source /opt/ros/jazzy/setup.bash
python3 -m pip install --target artifacts/teleop-deps -r tools/requirements/requirements-teleop.txt \
  proxsuite==0.7.3 'scipy==1.15.3; python_version < "3.11"' \
  'scipy==1.16.3; python_version >= "3.11"'
export BINDU_TELEOP_DEPS="$PWD/artifacts/teleop-deps"
export BINDU_PYTHON_VERSION="$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
export PYTHONPATH="$BINDU_TELEOP_DEPS/cmeel.prefix/lib/python$BINDU_PYTHON_VERSION/site-packages:$BINDU_TELEOP_DEPS${PYTHONPATH:+:$PYTHONPATH}"
export LD_LIBRARY_PATH="$BINDU_TELEOP_DEPS/cmeel.prefix/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export CMAKE_PREFIX_PATH="$BINDU_TELEOP_DEPS/cmeel.prefix${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
# 保持标准库优先，避免依赖目录中的旧版同名回移包遮蔽标准库。
export PYTHONPATH="$(python3 -c 'import sysconfig; print(sysconfig.get_path("stdlib"))'):$PYTHONPATH"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
```

Pinocchio、Coal及其Octomap依赖须来自同一数值环境。切换环境时仅修改`CMAKE_PREFIX_PATH`不会替换旧的`coal_DIR`缓存；wheel与ROS的两套Octomap混入同一进程可在退出时重复释放。使用wheel手动构建时，应加`--cmake-clean-cache`，并在`--cmake-args`后显式指定`-Dpinocchio_DIR="$BINDU_TELEOP_DEPS/cmeel.prefix/lib/cmake/pinocchio" -Dcoal_DIR="$BINDU_TELEOP_DEPS/cmeel.prefix/lib/cmake/coal"`。不要通过改SONAME、库软链接或跳过退出处理规避错误。

安装只需在首次准备或依赖变更时执行；每个新终端仍需加载 ROS、上述依赖路径和对应工作区。不要直接复用其他机器的 `.so` 或本地 `artifacts/vr/env.sh`。Conda／独立构建目录的配置见 [兼容性说明](../../../运行与验证.md#conda-兼容性)。

### 构建与模拟启动

在已准备上述依赖的同一终端执行：

```bash
colcon build --base-paths src --packages-up-to bindu_runtime --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE="$(command -v python3)" -DPYTHON_EXECUTABLE="$(command -v python3)"
source install/local_setup.bash
ros2 launch bindu_runtime skeleton.launch.py runtime_mode:=simulation recording_mode:=normal
```

`--packages-up-to bindu_runtime` 构建运行包及其依赖链，共15包，不含可选 `bindu_lio`。三维建图需改用 `--packages-up-to bindu_runtime bindu_lio` 并准备 PCL／OpenMP；直接构建全部包使用 `colcon build --base-paths src --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release`。

此入口启动模拟执行、感知、任务与记录，不自动提交运动任务。另一终端在仓库根目录加载相同 ROS、依赖与 `install/local_setup.bash`，保持 ROS 域一致后查看：

```bash
ros2 service call /bindu_sim/tasks/ready std_srvs/srv/Trigger '{}'
ros2 topic echo /bindu_sim/execution/state --once
```

任务提交见 [任务模块命令](../../tasks/bindu_tasks/README.md#编译与运行)，Ctrl+C 收尾。需要检查启动参数时，先用 `ros2 launch bindu_runtime skeleton.launch.py --show-args`，无需启动节点。

### VR、导航与真机入口

- PICO 与浏览器操作台：按 [遥操作模块命令](../../capabilities/bindu_teleoperation/README.md#编译与运行) 加载查看器依赖与 TLS，再用 `bash tools/run_vr.sh --config src/integration/bindu_runtime/config/deployments/snowbot_vr_sim.json`。
- 三维建图／定位：按 [LIO 模块命令](../../capabilities/bindu_lio/README.md#编译与运行) 构建可选包并提供点云、IMU及标定配置。
- 真实 Snowbot：目标为Orin ARM64，平台确认及验收缺口见[Snowbot运行记录](../../../../../snowbot/运行与验证.md)。下节[x86部署](#x86-真机部署已有适配底层)保留供x86环境复现；底层接口检查见[现场部署顺序](#snowbot-真机最小遥操作)。底层使能按独立现场流程执行，真机验证状态单独记录。

## x86 真机部署（已有适配底层）

本节仅适用于x86_64；`x86_real.py`仅支持x86，Orin需使用ARM64目标环境的部署流程。Orin须核对实际系统／ROS／Python版本并原生重建厂商消息、驱动依赖及QP扩展，不能搬用x86的`install/`或二进制；Orin部署尚未验收。

底层流程与接口差异见[真机驱动](../../../运行与验证.md#snowbot-真机驱动与双臂扫雪部署)。遵守[项目约束](../../../AGENTS.md#3-机器人软件工程约束)：本机改码后[同步版本](../../../运行与验证.md#开发同步)，机器人端不编辑源码或控制配置；本节包含真实硬件运动命令，启动前由现场操作者确认设备状态与操作条件。

目标机使用与本机相同的已适配厂商 `src`，支持 Ubuntu 22.04／ROS 2 Humble／Python 3.10，或 Ubuntu 24.04／Jazzy／Python 3.12。真机保留浏览器运动学显示，不需要 Isaac。新机重新编译，不复制本机 `install/`、`.so`、venv 或串联历史实验目录的 `artifacts/vr/env.sh`。

### 日常使用

首次部署完成后，在仓库根目录使用现有入口；自动加载ROS、底层消息、venv、Bindu安装和`artifacts/x86-real/site.json`：

| 操作 | 命令 | 频率 |
|---|---|---|
| 仅预检 | `python3 -m tools.deployment.x86_real check` | 检查配置、依赖、证书、端口及重复节点 |
| 现场人员启动 | `python3 -m tools.deployment.x86_real start` | 每次使用；已包含预检；由现场操作者确认运动条件后启动 |
| 更新后构建 | `python3 -m tools.deployment.x86_real build` | `src/`源码或包内配置变更后；仅文档／工具脚本更新无须重建 |

日常复用现有环境、证书和现场配方，无须重新`prepare`、安装依赖、填参数或逐项`source`。ROS域／RMW沿用终端设置并与底层一致；需指定域时加`--domain-id <实际域号>`。非默认部署目录统一加`--workspace /实际部署目录`。

`build`会清理CMake缓存，将Pinocchio／Coal固定到该部署的wheel前缀，并在独立进程加载QP和完成空ROS节点的创建／销毁；该检查仅使用回环域225，不发布控制目标。子进程正常退出后才将构建标记为就绪；未带此退出检查的旧构建需重新`build`，无需重新安装依赖。`environment.json`保留实际数值库路径，验证同时检查功能断言与进程退出状态。

仅现场配方变更时，在本机修改并同步后预检；证书到期或访问地址变化时更新TLS。依赖清单变化时重新`prepare/build`，保留现有配方；系统／Python／ROS、源码根目录或底层安装位置变化时新建部署环境。手动监测终端仍需加载一次`env.sh`；原五终端驱动测试不属于每日重新配置。

### 首次部署

**0．确认 Git 版本与需要单独准备的材料。** 新机普通双臂／持工具遥操均使用本节的 `tools.deployment.x86_real` 与统一 PICO 入口；后文 `snowbot_teleop.sh` 的旧四节点流程用于专项接口检查，按键和会话操作不混用。

| 材料 | 是否随 Git 分发 | 新机处理 |
|---|---|---|
| `tools/deployment/x86_real.py`、`tools/run_vr.sh`、输入诊断、浏览器页面、依赖清单 | 是 | 克隆／拉取选定提交，无须另找启动脚本 |
| `src/` 中的消息、QP、执行器、Snowbot URDF／网格与配方样例 | 是 | 从当前源码重新构建；模型不依赖历史参考目录 |
| Snowbot底层准备工具与专属补丁 | 否，独立场景资产 | 在文稿/snowbot维护，按其运行说明准备后提供厂商安装入口 |
| 底层原始 `src.zip`、已适配的独立底层工作区 | 否 | 单独携带源码并核对补丁版本；本机原包在 `/home/wxb/Downloads/src.zip` |
| `artifacts/vr/env.sh`、`artifacts/vr/default.json`、其他历史环境脚本和构建结果 | 否，属于生成／本机文件 | 用下面的 `prepare/build` 生成新机环境与 `site.json`，不搬历史覆盖层 |
| 现场标定文件、实际话题／节点、TLS 证书与私钥 | 不由通用配方提供 | 携带已核实参数或现场标定；按新机 IP 准备证书，私钥不入库 |

出发前在源码机执行 `git fetch origin`、`git status --short --branch`、`git log --oneline origin/main..HEAD`，确认所需提交已经推送；只在本地提交或修改，新机无法通过 Git 获取。首次克隆：

```bash
git clone --branch main https://github.com/WXB1870/bindu.git
cd bindu
git rev-parse HEAD
```

已有目标工作区时，先检查分支及未提交修改，再在 `main` 执行 `git pull --ff-only origin main`；保留现场改动，核对源码机与新机 `HEAD` 一致。克隆不会带来底层原包、证书或现场参数。

**1．准备系统依赖与现有底层消息。** 先安装对应 ROS 2。在新终端进入本仓库，以下以 Jazzy 为例，22.04 改用 Humble：

```bash
source /opt/ros/jazzy/setup.bash
sudo apt-get update
sudo apt-get install -y build-essential python3-dev python3-venv \
  python3-colcon-common-extensions python3-rosdep libeigen3-dev libsimde-dev \
  python3-yaml python3-zmq python3-cryptography patch \
  "ros-$ROS_DISTRO-nav2-msgs" "ros-$ROS_DISTRO-robot-state-publisher"
# 首次使用 rosdep 时先执行 sudo rosdep init；已初始化不重复执行。
rosdep update
mapfile -t bindu_packages < <(colcon list --base-paths src --packages-up-to bindu_runtime --paths-only)
rosdep install --from-paths "${bindu_packages[@]}" --ignore-src --rosdistro "$ROS_DISTRO" \
  --skip-keys 'pinocchio proxsuite' -r -y
```

Pinocchio／ProxSuite 由下一步固定版本 wheel 提供。这里只解析运行依赖链，不安装可选 LIO／Isaac；`python3-cryptography` 供本机生成 TLS 证书使用。底层 `src` 在自己的工作区按既有流程构建；若只准备上层编译和只读检查，可仅构建消息：

```bash
export BINDU_VENDOR_WS=/绝对路径/vendor_ws
(
  cd "$BINDU_VENDOR_WS"
  colcon build --base-paths src --packages-select snowbot_ctl_msgs --executor sequential
)
```

不要对已适配工作区重复套补丁或启动控制器。若只有已核对的**原始** `src.zip`，以下补丁准备仅在本机副本执行；校验、打包后同步到机器人新目录，再将`BINDU_VENDOR_WS`指向该目录构建消息。生成器核对原包SHA256，拒绝其他版本和已有输出：

底层原包准备及版本检查由[Snowbot工作区](../../../../../snowbot/运行与验证.md#本机底层源码准备与离线验证)维护，完成后提供选定的厂商工作区及安装入口。

历史 `artifacts/*/vendor*` 有不同补丁版本，不能按目录名里的 `final` 选取。已有底层源码须与当前补丁核对；构建消息成功不证明完整底层驱动已可运行。

**2．独立准备并构建 Bindu。** [`x86_real.py`](../../../tools/deployment/x86_real.py) 自动核对 OS／架构／Python 配对，从系统 Python 建立继承 ROS 系统包的 venv，安装[完整固定依赖清单](../../../tools/requirements/requirements-x86-real.txt)，然后构建 Bindu 运行依赖链和四个 Snowbot 应用包：

```bash
# 将路径替换为新机已适配厂商工作区的实际安装位置。
python3 -m tools.deployment.x86_real prepare \
  --application-source /绝对路径/snowbot \
  --vendor-setup /绝对路径/vendor_ws/install/local_setup.bash
python3 -m tools.deployment.x86_real build
```

默认产物均在忽略入库的 `artifacts/x86-real/`，可在每条命令加同一个 `--workspace /绝对路径/部署目录`。`prepare` 可在同一部署目录重试，保留已编辑的现场配方；拒绝接管无部署标识的非空目录或改换厂商环境。源码移动、系统／Python／ROS 升级时新建部署目录。生成的 `env.sh` 清除继承的 Python／ROS 覆盖层，只加载选定 ROS、厂商 `local_setup.bash`、独立 venv 和本次安装；保留调用方 DDS／ROS 域设置。`environment.json` 记录实际 Python、模块路径／版本、包前缀、提交及脏工作区状态，`requirements-resolved.txt` 记录本次解析的依赖版本；直接依赖使用固定版本，传递依赖仍由包管理器解析。

**3．先验 PICO 输入与证书。** 在 Bindu 仓库根目录，加载本次生成的环境，将示例 IP 换为新机连接头显的实际局域网 IPv4；已有匹配且受信任的证书时跳过生成步骤，改用原证书路径：

```bash
source artifacts/x86-real/env.sh
export BINDU_VR_IP=192.168.1.100
python3 -m tools.teleop.vr_connectivity \
  --make-cert artifacts/x86-real/tls --advertise-host "$BINDU_VR_IP"
python3 -m tools.teleop.vr_connectivity \
  --host 0.0.0.0 --advertise-host "$BINDU_VR_IP" \
  --cert-file artifacts/x86-real/tls/server.crt \
  --key-file artifacts/x86-real/tls/server.key --duration 1800
```

只向头显导入公开 `rootCA.crt`，私钥留在电脑；生成证书后，头显还需配置证书信任。PICO 自带浏览器打开 `https://新机实际IP:8012/`、进入 VR，逐个检查左右位姿、Squeeze／Trigger、X／Y／A／B，并验证退出／重连。电脑与头显须可互通、8012 可达；本步骤排查VPN／虚拟网卡、访客隔离和无线断流，保留原输入时效配置。诊断不启动 ROS 或机器人，完成后 Ctrl+C 释放8012端口。详细判据见[首次输入诊断](../../capabilities/bindu_teleoperation/README.md#vr-首次连通检查)。

**4．核实现场配方后预检。** 将生成的`artifacts/x86-real/site.json`带回本机，修改`arguments`后校验并同步；机器人端不直接编辑。配方默认`command_enabled=false`，按下表填写：

| 字段／配置 | 现场填写依据 |
|---|---|
| `runtime_mode`、`device_backend` | 保持 `real`、`snowbot_vendor` |
| `vendor_namespace` | 统一真机配方默认 `/`，匹配118原底层根话题；只有现场实际完成 `/snowbot/...` remap时才改为 `/snowbot` |
| `arm_receiver_node`、`servo_receiver_node`、`mid_receiver_node` | 默认节点不匹配时增加字段，填实际完整节点名 |
| `host`、`port`、`cert_file`、`key_file` | `0.0.0.0`、8012，以及新机 IP 对应的证书／私钥绝对路径 |
| `pico_config` | 现场独立副本；具名关节 `prepare_positions_rad` 单位 rad，缺省为 `null`，A 准备不可用 |
| `gripper_config` | 现场独立副本；默认 `calibrated=false`，实测开／闭／虚握开度与保持后才设为已标定 |

`@runtime/` 指向本次安装包；其他配置路径相对 `site.json`，跨目录可用绝对路径。`prepare`保留已经存在的现场配方，更新源码模板不会自动迁移旧`site.json`；118已有的`/snowbot`副本应在本机改为`/`后同步，保留TLS路径、端口和只读状态。现场副本可从 `src/integration/bindu_runtime/config/robots/snowbot/` 的 `pico_snowbot_position_real.json`、`snowbot_grippers_real.json` 复制，未标定时保留原状态，不套模拟值。普通双臂从当前实测末端接合，不依赖 A 准备位；夹爪与持工具操作按各自标定和确认条件开放。源码或依赖清单变更后重新 `build`；检查入口会按构建时的源码指纹拒绝旧产物。

**118反馈传输。** x86部署入口在加载现有环境前默认设置`FASTDDS_BUILTIN_TRANSPORTS=UDPv4`，新生成的`env.sh`也采用该默认值；显式传输选择、ROS域和RMW设置保留。更新入口脚本即可覆盖旧`env.sh`未设置传输的情况，不必为此重新`prepare`。该设置用于处理root驱动与cc观察端之间已确认的默认传输收包问题，证据及边界见[底层D118-13](../../../../../snowbot/底层驱动问题记录.md#已确认接口与问题)，不修改驱动或共享内存权限。接收帧率、ROS发布时间与成功PDO采样分别核验，收到16轴消息不能解除头腰测量和保护门槛。

```bash
ROS_DOMAIN_ID=42 python3 -m tools.deployment.x86_real check
# 底层已按现场核实流程运行后，先以 command_enabled=false 启动上层和浏览器。
ROS_DOMAIN_ID=42 python3 -m tools.deployment.x86_real start
```

底层与上层使用相同 ROS 域／兼容 RMW；分机通信另核对DDS发现与网卡配置，并按实际通信拓扑选择localhost设置。电脑页面默认 `http://127.0.0.1:8013/`，头显使用现场 HTTPS 地址8012端口。该工具不启动厂商控制器、不使能电机；写权限仍由已审阅的现场配方显式决定。底层本身启动可能回零／清零，持物时遵守[现场启动限制](#现场部署顺序)。

`check` 在启动控制节点前实际加载执行层扩展、QP／Pinocchio 和 ROS 消息原生支持，构造并关闭 QP；核对安装中的 robot／profile／tool／PICO／夹爪配置与当前源码、TLS、端口和重复节点。它会短暂建立只读 ROS 图检查节点，不发送运动指令；通过不表示底层已就绪。

**5．只读反馈通过后，再进入现场运动验收。** 在另一终端加载同一个 `env.sh`，使用与底层、启动终端相同的 `ROS_DOMAIN_ID`／RMW，检查：

```bash
source artifacts/x86-real/env.sh
export ROS_DOMAIN_ID=42
ros2 topic echo /bindu_snowbot/execution/state --once
ros2 topic echo /bindu_snowbot/teleop/vr/input --once
# 下面以底层根话题为例；有前缀时使用实际完整话题。
ros2 topic echo /bindu/control_source --once
ros2 topic echo /bindu/servo_health
```

浏览器应显示新鲜真实关节反馈与双手输入，模型方向／零偏／关节顺序与实物一致；持续检查PDO样本推进、年龄、WKC和故障，源采样有效性依据PDO数据判定。写入前还须核对 `source=topic`、`guard_enabled=true`、`ready=true` 及接收路由；只读阶段 `guard_armed=false` 本身不是故障。

现场运动获准且上述条件通过后，松开 Squeeze，退出只读上层，在本机生成并同步`command_enabled=true`的配方后，由现场人员重新`check/start`；启动由现场操作者执行。先在普通双臂模式分别做小范围低速跟随，验证逐侧松握保持、Y停止、输入断流及重连后的重新接合；确认实测停稳后再验收 B 切模式、已标定的 A 准备与夹爪／持工具操作。软件位置到位、实体夹稳和虚握防滑分别确认，真实六维力／滑移接入仍待验证。完整按键与收尾见[统一入口](../../../运行与验证.md#vr-统一部署与启动)，不沿用旧四节点的手工会话指令。部署证据保留本次 `environment.json`、`requirements-resolved.txt`、现场配置和 `log/vr/<run-id>/`。

## Snowbot 应用与配置归属（2026-10-26）

Snowbot 当前配方、模型、设备适配及 PICO 操作分别由独立工作区的四个应用包维护，入口见[Snowbot README](../../../../../snowbot/README.md)。本包保留通用 ROS 节点、配置解析和旧导入／launch 的兼容转发；源码中的 `config/robots/snowbot` 与四份 Snowbot 部署配方已迁出。本文相关文件链接指向维护位置，历史相对路径只说明原版本。

首次 `x86_real prepare` 增加 `--application-source /绝对路径/snowbot`；部署身份固定两个源码工作区，构建从两边的 `src/` 选择 `snowbot_bringup` 及其依赖，不构建原始厂商基线，也不启动设备。更新后必须重新构建，旧安装不能只覆盖转发文件。有效配置与来源哈希保存在部署目录 `resolved-config.json`，运行时 CLI 覆盖另外记录在 `session.json` 的 `effective_arguments`。TLS 内容不会进入快照。

开发机离线读取应用资源时，将实际包根或包含应用包的 `src/` 显式加入 `BINDU_PACKAGE_PATH`；安装环境通过 ament 索引解析 `package://`。不推测相邻仓库路径，不自动复用历史候选。旧 `@runtime/config/robots/snowbot/...` 引用仍由兼容解析指向唯一维护源。

## 职责与边界

- 负责消息转换、依赖选择、设备／遥操作装配、本体校验与进程生命周期。
- 启动时明确仿真／真实环境，检查指令和反馈模式一致性；环境选择与设备后端、写权限分别配置。
- 真实模式支持 Snowbot 双臂／腰部位置遥操作装配，默认只读；PICO位置操作及独立夹爪服务已接入；实体虚握、Pi和导航未完成真机接入。
- 配置选择不替代真实模型标定、设备能力核实和现场验证。

## 配置目录

| 位置 | 作用与适用范围 |
|---|---|
| [deployments/](config/deployments) | 选择节点、输入和后端；含默认只读的Snowbot真机最小配方 |
| [robots/snowbot/](../../../../../snowbot/src/snowbot_bringup/config/robots/snowbot) | 本体、位置执行、单／双臂、PICO及物理参数；真实接口与仿真分别维护，见下表 |
| [robots/g1/](config/robots/g1) | 临时G1模型、执行限制、单／双臂及碰撞检查，均用于模拟验证 |
| [robots/huawei_v34/](config/robots/huawei_v34) | 旧单臂模型的执行与遥操作模拟基线 |
| [robots/wheel_gen2/](config/robots/wheel_gen2) | 二代位置映射、Airy雷达预适配及未补齐的本体描述 |
| [navigation/](config/navigation) | LIO／定位算法参数、导航站点、仿真房间及传感器扰动；真机还需模型与标定 |
| [vla/](config/vla) | Pi客户端及回环示例，真实模型服务待联调 |
| [simulation/](config/simulation) | 通用模拟执行参数，用于接口与任务验证 |
| [tests/config/](../../../tests/config) | 验证专用夹具，不安装到运行包 |

运行包提供参数配置与示例；真机配置需按对应设备与现场条件选择；新增配置按职责放入对应目录。Snowbot源位置接口保留`snowbot_vendor_position.json`，真机工具装配使用`snowbot_real_position.json`；二代的[position_mapping_wheel_gen2.json](config/robots/wheel_gen2/position_mapping_wheel_gen2.json)维护通道／单位／方向映射，[navigation_wheel_gen2_airy.json](config/robots/wheel_gen2/navigation_wheel_gen2_airy.json)尚缺基座系、外参与高度范围。`robot_wheel_gen2.json`的模型、profile及遥操作仍为空；`robot_snowbot.json`当前绑定仿真profile。真机位置链使用下表专用配方，完整PICO开合／虚握流程仍待实体适配。

安装保持相同目录结构。旧`teleop_snowbot_deployment.json`并入`deployments/teleop_robot_deployment.json`，schema 1读取仍兼容；升级需重建运行包并更新外部脚本中的旧平铺路径。

## 扫雪遥操作

### 场景选择

| 目的 | 当前入口 | 反馈与验证范围 |
|---|---|---|
| 工具位姿输入、模式切换与暂停／恢复 | [teleop_only.launch.py](launch/teleop_only.launch.py)，默认加载 [teleop_robot_deployment.json](config/deployments/teleop_robot_deployment.json) | ROS 运动学模拟；默认 `tool_pose` 输入，现场 VR 待接入 |
| PICO按键与操作状态机 | 同一launch，选择[teleop_pico_snowbot_sim.json](../../../../../snowbot/src/snowbot_bringup/config/deployments/teleop_pico_snowbot_sim.json) | 成对VR相对映射、准备／装卸与模拟开合；仅运动学后端，实体夹持及真实恢复未接入 |
| 整段双臂协调、CAD几何与实时回放 | [run_shovel_physics.sh](../../../tools/run_shovel_physics.sh)，使用 `--kinematic ... --gui` | 日常运动学检查；生产QP／执行器生成输入，Isaac 6.2核对关节和法兰并回放，无承载／接触验收 |
| 短时重力、夹爪接触与载荷 | 同一入口，使用 `--closed-loop` | 默认contact，有限力矩J8与CAD接触；私有IPC复用求解／会话／执行核心；旧弹簧观测须显式选proxy，不启动ROS节点 |
| Snowbot 真机最小遥操作 | 同一launch，选择[snowbot_real_teleop.json](../../../../../snowbot/src/snowbot_bringup/config/deployments/snowbot_real_teleop.json) | VR接收、模式、双臂QP、位置适配共4节点；不启动记录、单臂、显示和任务模块，默认只读；[操作边界](#snowbot-真机最小遥操作) |
| Snowbot 底层位置接口联调 | [snowbot_adapter.launch.py](launch/snowbot_adapter.launch.py) | 独立位置入口，默认真实模式且只读；不包含遥操作节点 |

`runtime.launch.py` 的默认 [runtime_deployment.json](config/deployments/runtime_deployment.json) 选择位置适配入口的仿真模式。运行扫雪工具会话从 `teleop_only.launch.py` 进入；部署时选择完整的`snowbot_vr_real.json`或底层联调配方`snowbot_real_teleop.json`，不能只切换`runtime_mode`。

统一PICO入口为[`tools/run_vr.sh`](../../../tools/run_vr.sh)，选择`snowbot_vr_sim.json`或`snowbot_vr_real.json`。两者使用`position_contract=true`、同一真实位置profile／工具几何和协调器；只在设备IO、初始测量状态及现场标定处区分。状态统一到`teleop/state`，夹爪由同一执行节点提供服务。旧四节点配方保留底层联调用途；具体启动与操作见[统一入口](../../../运行与验证.md#vr-统一部署与启动)。

真实腰臂通过`position_protocol=session_v3`显式选择目标流，须与servo只读参数`position_reference_mode=target_stream`配对（wire 5／reference 2／普通目标配方version 2，显式刚性预测version 3），消息及使用方成套重建。上层只发送16轴联合目标和源身份／期限，servo非实时后台复用`PreparedSegment`生成；仅首次／恢复接管有一次真实接纳握手；正常更新不等待上一目标接纳，PAUSE立即绕过等待，由执行侧从已采用名义参考制动。适配器已移除旧`session_v2`曲线准备、ACK串行依赖及跨层提前量配置；`ExecutionState.reference`来自有效servo采样的q／dq／ddq及采样时刻，缺失时为空。

刚性双臂模式按`rigid_reference_prediction`显式选择version 3几何预测，独立臂、准备与PAUSE保持version 2；须与包含实际模型的servo成套构建，具体边界见[执行说明](../../control/bindu_execution/README.md#扫雪位置执行)。

当前真实VR配方的全局目标流上限为0.5 rad/s、1.5 rad/s²；双臂14轴使用该值，两腰轴在逐轴profile中单独限制为0.25 rad/s、0.5 rad/s²，发送时按关节取较小值；这不会改变已有部署中的覆盖profile。servo须加载[Snowbot目标流配方](../../../../../snowbot/tools/deployment/snowbot_vendor/position_session/target_stream.yaml)中的同名上限；任一层仍有更低限制时取其更小值。此组参数已完成离线验证，实体跟踪与停止仍待验证。

目标轴序为左臂J1～J7、右臂J1～J7、腰J1～J2，rad。动态界逐轴取profile与发送端只读启动参数`trajectory_max_velocity`（默认0.25 rad/s）、`trajectory_max_acceleration`（默认0.5 rad/s²）及固定162 rad/s³的较小值，servo默认帽一致。配置20 ms执行tick／servo名义1 ms采样保持；submit也可触发执行tick，50 Hz不是发送硬限频，实际频率与抖动须现场记录。普通目标最多300 ms，新PAUSE独立固定2 s停止期限，详见[执行时间语义](../../control/bindu_execution/README.md)；沿途不续期。此新路径只支持`execution_mode=position`，与阻抗组合明确拒绝；既有可选阻抗入口的控制和暂停语义保持，其兼容性不属于本轮验证；含可选阻抗的准备器候选不应用目标流补丁，也不接受`target_stream`配置。真实腰臂配方为`session_v3`、`position`、`command_enabled=false`；servo默认仍`follower`／`unconfirmed`，必须显式成套选择新路径。版本、离线证据与现场未完成项见[Snowbot当前进度](../../../../../snowbot/当前进度.md)。

真实`session_v3`配方的`arguments`可设置这两个速度／加速度参数，统一遥操与独立适配器launch均透传；只接受有限正值，其他协议／仿真配方显式设置时拒绝。servo已有同名只读启动参数，必须允许下发的逐轴有效值，超出接收帽仍拒收；不自动远程修改servo参数。同一会话内动态界不得改变。提高发送帽不会自动提高QP使用的profile速度，profile更低时仍受其限制；调参须成套选择启动值和逐轴profile，尤其不要借全局参数提高腰部等不打算调整的轴。当前真实配方与逐轴profile采用上述腰臂分组上限；急动度、参考超差保护、TTL和PAUSE期限保持。

### 配套配置与流程

VR输入、PICO协调、双臂求解和执行进程入口在ROS初始化前默认设置`RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS`，保留现场显式覆盖，避免Fast DDS默认同步发送占住输入／控制回调。输入采样时间、QoS及原时效／停止契约保持；该传输设置不证明现场网络或调度停顿已经消失。

真机配方增加`snowbot_grippers_real.json`，默认`calibrated=false`禁止夹爪写入；位置闭环及授权Trigger操作见[真实夹爪接口](../../hardware/bindu_hardware/README.md#snowbot真实夹爪位置闭环)。夹爪服务由同一execution节点提供，原四节点范围不变。底层需配套`bindu_gripper_control`保护入口，实体持物／虚握不由位置到位推断。

| 配置 | 作用 |
|---|---|
| [robot_snowbot.json](../../../../../snowbot/src/snowbot_bringup/config/robots/snowbot/robot_snowbot.json) | 本体装配索引：绑定模型、执行profile、单臂与工具配置，当前选仿真 |
| [robot_snowbot_real.json](../../../../../snowbot/src/snowbot_bringup/config/robots/snowbot/robot_snowbot_real.json) | 真机模型／profile／工具索引，不包含单臂配置或模拟初始姿态 |
| [snowbot_real_position.json](../../../../../snowbot/src/snowbot_bringup/config/robots/snowbot/snowbot_real_position.json) | 真实位置运行限制；头部第2轴取源接口与CAD交集±0.79 rad，其余沿用源接口，仍需现场核实 |
| [bimanual_snowbot_real.json](../../../../../snowbot/src/snowbot_bringup/config/robots/snowbot/bimanual_snowbot_real.json) | 真机位置链的名义工具安装／CAD几何／QP与时效参数；不包含演示轨迹，安装尺寸和地面需标定 |
| [snowbot_grippers_real.json](../../../../../snowbot/src/snowbot_bringup/config/robots/snowbot/snowbot_grippers_real.json) | 独立夹爪位置闭环、超时与开合参数；默认未标定，拒绝写入 |
| [snowbot_base.json](../../../../../snowbot/src/snowbot_bringup/config/robots/snowbot/snowbot_base.json) | 可选PICO两轴`cmd_vel`端口、摇杆限速、反馈与停止条件；设备独立超时未确认，默认底盘只读 |
| [snowbot_shovel_sim.json](../../../../../snowbot/src/snowbot_bringup/config/robots/snowbot/snowbot_shovel_sim.json) | 当前扫雪仿真的资源组、限位、动态限制与腰臂同步执行 |
| [snowbot_sim.json](../../../../../snowbot/src/snowbot_bringup/config/robots/snowbot/snowbot_sim.json) | 旧Snowbot模拟入口的执行基线，供兼容入口及回归使用 |
| [snowbot_vendor_position.json](../../../../../snowbot/src/snowbot_bringup/config/robots/snowbot/snowbot_vendor_position.json) | 真实位置接口的20轴资源与限制；适配器默认只读，独立夹爪接口需另配已标定配置，真机未验收 |
| [teleop_snowbot_left.json](../../../../../snowbot/src/snowbot_bringup/config/robots/snowbot/teleop_snowbot_left.json)／[right](../../../../../snowbot/src/snowbot_bringup/config/robots/snowbot/teleop_snowbot_right.json) | 左／右单臂的相对映射、IK与时效参数，当前仅仿真 |
| [bimanual_snowbot_shovel.json](../../../../../snowbot/src/snowbot_bringup/config/robots/snowbot/bimanual_snowbot_shovel.json) | 工具安装、握持约束、姿态策略、几何与输入时效 |
| [bimanual_snowbot_sim.json](../../../../../snowbot/src/snowbot_bringup/config/robots/snowbot/bimanual_snowbot_sim.json) | 早期双臂共同工具的简单模拟案例，供兼容入口及回归使用 |
| [pico_snowbot_sim.json](../../../../../snowbot/src/snowbot_bringup/config/robots/snowbot/pico_snowbot_sim.json) | PICO准备与模拟第8轴开合／虚握参数，不能用于实机 |
| [snowbot_shovel_physics.json](../../../../../snowbot/src/snowbot_bringup/config/robots/snowbot/snowbot_shovel_physics.json) | 空铲木质质量、接触夹持、Fabric与GUI刷新；保留显式proxy对照参数 |

基础构建和启动命令见本文[编译与运行](#编译与运行)，系统依赖见[环境准备](../../../运行与验证.md#环境准备与构建)。普通工具装配需有效握姿和新鲜反馈，再显式选择`bimanual_tool`并启动`teleop/tool_session`；[专用装配与暂停](../../capabilities/bindu_teleoperation/README.md#遥操作专用装配与暂停)维护完整步骤。默认零位不是准备姿态，测试夹具仅由专用模拟入口显式初始化。

`snowbot_vr_real.json`／`snowbot_vr_sim.json`已选择可选`base_config`，由原PICO节点承载，无新增控制进程。方向映射、`/cmd_vel`／`Odometry`前提及模拟预览范围见[底盘适配](../../capabilities/bindu_teleoperation/README.md#底盘摇杆适配)；修改`VRInput.stick`后须重建接口和运行包，不能只复制Python文件到旧安装。旧配方不配置`base_config`时保持原行为。

PICO仿真由协调器管理上述Action，无需手动开启工具会话；真机最小配方仅接受位姿／Squeeze／Y，仍需显式启动Action。按职责查阅：[按键与相对映射](../../capabilities/bindu_teleoperation/README.md#pico-按键方案) · [启动、装卸及重接步骤](../../../运行与验证.md#pico-扫雪操作流程) · [状态契约](../../../动作执行契约.md#61-snowbot-工具操作与握持切换) · [真实底层启动与重启限制](../../../软件架构设计.md#832-snowbot-实机启动与准备)。

`tool_pose`消费映射后的目标，`vr`在会话内完成相对映射；仿真中`tight=false`仅释放右手绕杆转动，当前真实位置链收到该请求会停止，完整输入语义见[遥操作模块](../../capabilities/bindu_teleoperation/README.md#扫雪输入与握持)。

### 物理反馈与柔顺

接触／proxy模型、反馈选项、GUI启动及内力定义统一见[PhysX受力实验](../../hardware/bindu_hardware/README.md#木质雪铲的-physx-受力实验2026-10-15)。当前无双臂内力上限优化或额外上层导纳／阻抗；底层已有柔顺，仿真参数尚未与实机标定。

## 源码导航

| 入口 | 内容 |
|---|---|
| [setup.py](setup.py) | 全部 ROS 可执行入口与安装资源 |
| [plugins.py](bindu_runtime/plugins.py)、[device_assembly.py](bindu_runtime/device_assembly.py)、[simulation_devices.py](bindu_runtime/simulation_devices.py) | 能力选择、设备与模拟后端装配 |
| [teleop_assembly.py](../../../../../snowbot/src/snowbot_bringup/snowbot_bringup/assembly.py) / [teleop_deployment.py](bindu_runtime/teleop_deployment.py) | 遥操作依赖和部署配置 |
| [pico_node.py](../../../../../snowbot/src/snowbot_teleop/snowbot_teleop/node.py) | PICO操作串接、反馈与执行权交接 |
| [robot_setup.py](bindu_runtime/robot_setup.py) / [common.py](bindu_runtime/common.py) | 本体／自由度校验、ROS 转换与公共运行辅助 |
| [execution_node.py](bindu_runtime/execution_node.py) / [snowbot_adapter.py](../../../../../snowbot/src/snowbot_hardware/snowbot_hardware/adapter.py) | 共享执行节点与底层位置适配 |
| [bindu_runtime/](bindu_runtime) | 遥操作、VR、Pi、导航、感知、任务、记录等节点源码 |
| [launch/](launch) / [config/](config) | 启动装配与配置全集 |

### 常用启动与配置入口

| 用途 | 启动文件 | 配置示例 |
|---|---|---|
| 环境选择与运行装配 | [runtime.launch.py](launch/runtime.launch.py) | [runtime_deployment.json](config/deployments/runtime_deployment.json) |
| 本体与遥操作模式选择 | [robot_teleop.launch.py](launch/robot_teleop.launch.py) | [robot_snowbot.json](../../../../../snowbot/src/snowbot_bringup/config/robots/snowbot/robot_snowbot.json) |
| 旧 Snowbot 模拟入口兼容 | [snowbot_sim.launch.py](launch/snowbot_sim.launch.py) | [snowbot_shovel_sim.json](../../../../../snowbot/src/snowbot_bringup/config/robots/snowbot/snowbot_shovel_sim.json) 与 [bimanual_snowbot_shovel.json](../../../../../snowbot/src/snowbot_bringup/config/robots/snowbot/bimanual_snowbot_shovel.json) 须配套选择 |
| Snowbot 底层适配 | [snowbot_adapter.launch.py](launch/snowbot_adapter.launch.py) | [snowbot_vendor_position.json](../../../../../snowbot/src/snowbot_bringup/config/robots/snowbot/snowbot_vendor_position.json) |
| 导航物理场景 | [navigation_physics.launch.py](launch/navigation_physics.launch.py) | [navigation_room.json](config/navigation/navigation_room.json) |
| 三维建图／地图定位 | [lio.launch.py](launch/lio.launch.py) / [localization.launch.py](launch/localization.launch.py) | [navigation_lio.yaml](config/navigation/navigation_lio.yaml) / [navigation_localization.yaml](config/navigation/navigation_localization.yaml) |

`runtime.launch.py` 读取部署配置并选择具体 launch；[teleop_robot_deployment.json](config/deployments/teleop_robot_deployment.json) 是选择 `robot_teleop.launch.py` 的部署示例。`robot_teleop.launch.py` 自身通过 `robot_config` 读取本体配置。上表配置为场景示例，所需参数及默认值见对应运行说明。

## 接入与验证

常用构建、环境加载与启动命令见本文[编译与运行](#编译与运行)；完整场景参数、操作顺序和验证边界见 [运行与验证](../../../运行与验证.md)。

- 环境切换：[统一仿真与真实模式选择](../../../运行与验证.md#统一仿真与真实模式选择)。
- 本体与装配检查：[本体配置](../../../tests/unit/test_robot_configuration.py)、[模式](../../../tests/unit/test_runtime_mode.py)、[遥操作装配](../../../tests/unit/test_teleop_deployment.py)。
- ROS 检查：[示例任务](../../../tests/integration/validate_ros.py)、[环境一致性](../../../tests/integration/validate_runtime_mode.py)、[遥操作](../../../tests/integration/validate_teleop.py)、[底层适配](../../../tests/integration/validate_snowbot_adapter.py)。各脚本是否自行启动装配见其参数；[真机位置链合成验证](../../../tests/integration/validate_real_teleop.py)自带替身控制器，不接硬件。

## 关联模块

[公共契约](../../shared/bindu_contracts/README.md) · [ROS 接口](../../shared/bindu_interfaces/README.md) · [共享执行](../../control/bindu_execution/README.md) · [设备适配](../../hardware/bindu_hardware/README.md) · [遥操作](../../capabilities/bindu_teleoperation/README.md) · [全部模块](../../../README.md#模块快速导航)

## 运行与专项验证

以下命令和项目路径均相对于仓库根目录。带日期内容仅代表当时的专项实验，现行控制规则以动作执行契约和统一部署为准。

### 临时 G1 全身模拟入口

采用固定版本的 Galbot One Golf 派生[运动学模型](../../hardware/bindu_description/urdf/g1_provisional.urdf)，[执行配置](config/robots/g1/g1_provisional_sim.json)覆盖20个关节轴，底盘另用机体线速度`v`和角速度`ω`控制：

| 控制组 | 关节顺序 | 数量 |
|---|---|---|
| `leg` | `leg_joint1`、`leg_joint2`，连杆升降 | 2 |
| `waist` | `leg_joint3`俯仰、`leg_joint4`回转 | 2 |
| `head` | `head_joint1`、`head_joint2` | 2 |
| `left_arm` | `left_arm_joint1` → `left_arm_joint7` | 7 |
| `right_arm` | `right_arm_joint1` → `right_arm_joint7` | 7 |

角度单位为rad。位置限位逐轴取自原URDF，保留左右臂不对称范围；速度上限暂取原值与0.5 rad/s的较小值，加速度/jerk的2 rad/s²、12 rad/s³为模拟执行配置，不能作实机参数。2026-10-01起临时仿真模型启用`leg_joint4`腰部回转，`leg_joint5`仍固定零位；手爪与轮子子树省略，底盘由差速模拟驱动负责，手臂保留法兰安装坐标。模型只保留运动学/惯性，不含视觉网格或碰撞几何；20轴配置是临时近似，尚未做实机标定或核实实机回转轴。

```bash
ros2 launch bindu_runtime g1_sim.launch.py recording_mode:=normal
python3 -m tests.integration.validate_g1 --output artifacts/g1-check
```

第一条命令启动执行器、记录器、实测状态桥、`robot_state_publisher`和到站导航Action；第二条自行启动独立命名空间做验证。模型通过`robot_description`加载，20轴反馈发布为`joint_states`并驱动TF；`odom→base_link`使用差速模拟反馈，保留源时间，失联不会刷新旧姿态。TF话题隔离在各自命名空间，当前不发布`map→odom`。默认仍是单组动作；可选双臂工具节点按三组同一时间轴执行，本入口不启动依赖手部的取物任务。

G1 能力统一从同一入口选择（需要重建工作区以生成新增的 `NavigateToSite` Action）：

```bash
# 左臂 VR/IK 与显示；右臂改为 side:=right。头显连接另需 host/TLS 配置。
ros2 launch bindu_runtime g1_sim.launch.py recording_mode:=normal vr_enabled:=true side:=left
# 已有 VRInput 源时，仅启用 IK 与显示。
ros2 launch bindu_runtime g1_sim.launch.py recording_mode:=normal teleop_enabled:=true side:=left
# Pi 客户端；另需配置服务端点并提供三路 RGB 与全身关节观测。
ros2 launch bindu_runtime g1_sim.launch.py recording_mode:=normal pi_enabled:=true
```

以上是不同启动方式，不应在同一命名空间重复启动。VR 左右臂分别使用 `teleop_g1_left/right.json`；会话资源为 `left_arm` 或 `right_arm`，一次控制一臂。IK 和显示使用当前腿腰、头部及另一臂反馈，优化变量只有所选臂的7轴；非活动关节缺失/非法或求解期间姿态变化时拒绝下发。末端暂用法兰坐标，不包含手爪TCP标定。Pi 的 `pi_g1.json` 显式映射20轴观测，要求动作关联观测与会话，允许同样的两个单臂资源；不表示远端模型已支持该本体，默认端点仍为本地测试地址。可选`bimanual_enabled:=true`启动双臂工具QP、成对VR和手动模式选择；目前仅空场景无接触仿真，G1物理取消停稳尚有失败，状态及触发条件见[架构接入计划](../../capabilities/bindu_kinematics/README.md#snowbot-双臂与两轴腰部遥操作)。

G1左右IK默认启用[简化自碰撞配置](config/robots/g1/g1_ik_collision.json)：左右`link7`各4个包围球，躯干2个局部轴向包围盒，检查左右link7之间及各自与躯干，共32个几何对。尺寸包住固定版本上游碰撞网格并加1mm几何余量；净间距要求20mm。形状随当前关节反馈变换，链接名／几何／碰撞对都在配置中，求解器不依赖G1名称，替换URDF需同步替换该配置。

CasADi/IPOPT施加间距硬约束，数值Pinocchio FK独立复核当前姿态、解及最大关节间隔0.025rad的线性关节路径采样。当前姿态侵入返回`IK_COLLISION_SEED`，采样路径侵入返回`IK_COLLISION_PATH`，不可解／残差／时效超限仍拒绝出指令。旧v3.4配置未自动启用此几何。这是单臂IK的局部自碰撞保护，不检查其余臂段、手、相机或环境，也不是全局绕障规划；离散路径检查不保证执行器插值、跟踪误差或同时运动的其他部件在整个连续轨迹上无碰撞，直接关节／Pi入口也不经过此IK检查。

IK预热只做数值计算，不发送动作。替换模型若默认零位与碰撞配置冲突，可在`kinematics.warmup_seed`按`joint_names`顺序指定可行关节位置；缺省为限位内零位。非法或碰撞预热姿态分别拒绝为`IK_INVALID_WARMUP_SEED`／`IK_UNSAFE_WARMUP_SEED`，实际请求仍使用反馈关节和身体姿态。

2026-09-23验证：3个受影响包重建，9项碰撞测试（含两臂160组符号／数值几何一致性）、6项G1和23项遥操作模块检查通过；左右ROS正常跟随、松握接管和本体姿态共5项通过；240次连续IK全通过，单次求解中位数7.1ms、P95 8.7ms（非端到端100Hz保证）。Isaac GUI中两臂各6次受约束目标通过，按实际关节反馈计算的最小代理间距为106.4／106.5mm；碰撞拒绝测试未向仿真下发侵入目标。原始证据及失败测试保存在`artifacts/ik-collision-2026-09-23/verification.json`。在已启动同命名空间Isaac桥接的终端，可运行`python -m tests.physics.validate_physics --namespace /bindu_g1_collision --suite collision --output artifacts/ik-collision-check`复测；此入口默认读取`artifacts/g1-physics-model`。

到站导航无需手部/感知服务，默认开启，可用 `navigation_enabled:=false` 关闭。接入满足前述身份与时间契约的导航后端后，可提交：

```bash
ros2 action send_goal /bindu_g1_sim/navigation/navigate_to_site bindu_interfaces/action/NavigateToSite \
  "{task_id: g1_nav_1, site: pickup}" --feedback
```

Action 仅申请底盘控制权；后端成功、定位误差与停稳条件仍必须同时满足。启动入口不包含导航后端、地图、定位或SLAM，无后端会明确失败。`navigation_sim.json` 中的站点只用于协议模拟。

G1 集成回归入口（均自行启动隔离模拟进程）：

```bash
python3 -m tests.integration.validate_teleop --g1 --output artifacts/g1-vr-left --cases normal body_pose clutch cancel
python3 -m tests.integration.validate_teleop --g1 --side right --output artifacts/g1-vr-right --cases normal body_pose
python3 -m tests integration pi --g1 --output artifacts/g1-pi
python3 -m tests integration navigation --g1 --output artifacts/g1-navigation
```

2026-09-19 本机 macOS/ARM64、Python 3.12 完成106项模块测试，无跳过；当时因开发机SSH连接超时，新增ROS场景未运行。2026-09-22 在 Ubuntu 24.04 / Jazzy / bindu Conda Python 3.12.14 上完成14包独立重建（含新增Action），106项模块无跳过、9项G1全身、左/右臂VR 11/3项、Pi左臂14项及右臂2种通信模式、18项到站导航全部通过；旧入口16 ROS、14 Pi、18导航及11 VR回归通过。VR包含真实Vuer服务接收合成输入，导航仍使用协议替身。

本批首轮构建因Conda旧`argparse`覆盖标准库失败，导航首轮因未加载隔离`nav2_msgs 1.3.13`失败；调整环境路径后通过，受影响的G1全身及左右VR/Pi完整启动另行复测通过，未修改运行源码或放宽阈值。Vuer连接断开及部分节点SIGINT收尾仍有异常日志；该批功能断言通过，退出过程仍有异常。本批模拟进程已收尾，未运行现场头显、真实Pi模型、真实Nav2/SLAM或真机。

模型来源与修改范围见[架构](../../../软件架构设计.md#854-当前代码组织与替换边界)，生成命令与分发范围见[模型与资产](../../hardware/bindu_description/README.md#机器人模型与资产)。

此前9月18日Ubuntu/Jazzy验证：14包构建、103项模块、9项全身场景及5项相关导航回归通过。TF与FK按同一时间戳的实测样本对照；初轮用较早TF与最终目标比较而失败，修正验证器后通过，精度阈值未放宽。



### Snowbot 真机最小遥操作

新机双臂遥操主流程见前文 [x86 真机部署](#x86-真机部署已有适配底层)。以下现场顺序与 `snowbot_teleop.sh` 自动化保留旧四节点接口检查用途，不包含统一 PICO 协调器／浏览器，不能用其按键限制判断新入口能力。


### 现场部署顺序

本节面向旧四节点最小位置遥操作：先完成环境部署和只读验收，再在获准实验范围内验证位置跟随。该入口没有统一PICO实体装卸、X虚握、A准备和B装夹确认；真实夹爪可通过[独立位置闭环](../../hardware/bindu_hardware/README.md#snowbot真实夹爪位置闭环)检查。统一入口的软件按键已接入，实体标定与载荷确认仍须验收。底层待命／保持目前只是部署假设，正式运动前须核实；当前底层源码启动仍含回位和六维清零，`command_enabled=false`及`bindu_guard=true`都不能阻止这些底层启动动作。持铲状态下不按本流程重启旧底层。

**1．带齐源码与现场输入。** 先选定本轮运行上层的目标机，确认Ubuntu／ROS／CPU架构；首批尽量在同一主机运行Bindu与底层ROS链路，PICO经局域网访问VR服务。准备Bindu源码、用户提供的底层`src.zip`、现场有效的TLS证书／私钥路径、实际话题前缀和节点名称、工具安装尺寸及初始握姿。`artifacts/`、底层原包和证书不随Git分发；不复制Jazzy或另一架构的`build/install`作为目标机安装。

出发前确认所需源码已推送到现有`origin/main`，再在目标机保留工作区修改、检查`main`分支后执行`git pull --ff-only origin main`，核对与选定本地提交一致。未推送的本地提交不能由目标机pull取得；只整理本文不隐含已经推送／同步。部署终端先检查：

```bash
cd /home/wxb/bindu
git status --short --branch
git rev-parse HEAD
uname -m
cat /etc/os-release
test -f /opt/ros/humble/setup.bash
```

**2．准备目标机依赖与底层消息。** 厂商源码补丁在本机准备后同步，下面的源码生成命令不得在机器人执行；目标端仅安装依赖／构建，不启动底层控制。先具备colcon、ROS依赖及Eigen／SIMDe开发文件，具体系统依赖见[环境准备](../../../运行与验证.md#环境准备与构建)和[`validate_humble_teleop.sh`](../../../tools/deployment/validate_humble_teleop.sh)；不要为遥操作顺带部署可选LIO等能力。源码准备生成工作副本，消息构建提供接口类型；底层部署另需按现场版本核对完整依赖、网卡与启动流程。

```bash
source /opt/ros/humble/setup.bash
export BINDU_ROS_DISTRO=humble
export BINDU_TELEOP_WORKSPACE="$PWD/artifacts/real-teleop"
# 底层源码先由独立Snowbot工作区在本机准备并同步；此处只构建选定版本。
export BINDU_VENDOR_WS=/绝对路径/vendor_ws
colcon --log-base "$BINDU_VENDOR_WS/log" build \
  --base-paths "$BINDU_VENDOR_WS/src/ctl_msgs" \
  --packages-select snowbot_ctl_msgs --executor sequential \
  --build-base "$BINDU_VENDOR_WS/build" \
  --install-base "$BINDU_VENDOR_WS/install"
export BINDU_VENDOR_SETUP="$BINDU_VENDOR_WS/install/local_setup.bash"

# 新建与目标ROS的Python匹配、继承系统包的venv；已有环境先核对再复用。
python3 -m venv --system-site-packages "$BINDU_TELEOP_WORKSPACE/venv"
export BINDU_VENV="$BINDU_TELEOP_WORKSPACE/venv"
source "$BINDU_VENV/bin/activate"
python3 -m pip install -r tools/requirements/requirements-teleop.txt \
  proxsuite==0.7.3 scipy==1.15.3 cmake==3.31.6
bash tools/snowbot_teleop.sh build
```

上述Python依赖组合有Humble x86_64容器证据，ARM64／Orin仍需现场构建验证；缺wheel、动态库或ABI错误必须在进入下一步前解决。若使用现场已有消息安装，`BINDU_VENDOR_SETUP`改为它的路径，并确认`snowbot_ctl_msgs`定义与当前源码一致。独立构建涉及15包，成功后检查colcon汇总无失败。

**3．生成现场只读配置并离线检查。** 先确认底层实际话题。原包常见根话题时使用`/`；只有现场明确使用`/snowbot/...`才设为`/snowbot`。接收节点非默认名称时增加下文自动化入口的三个`--*-receiver-node`参数。以下选择根话题作为例子，TLS路径须替换为本机真实文件：

```bash
export SNOWBOT_VENDOR_NS=/
bash tools/snowbot_teleop.sh configure --config artifacts/real-teleop/site-readonly.json \
  --host 0.0.0.0 --cert-file /绝对路径/cert.pem --key-file /绝对路径/key.pem \
  --vendor-namespace "$SNOWBOT_VENDOR_NS"
bash tools/snowbot_teleop.sh check --config artifacts/real-teleop/site-readonly.json
```

检查工具安装尺寸、URDF／关节方向／限位、准备握姿、地面与工作区配置；现有profile和QP的速度等数值不是已验收的实机低速参数。需要调整时在本机现场副本维护配套robot/profile/tool配置及部署引用，校验后同步，再重新`check`，不靠修改安装目录临时绕过检查。首次只读核对时底层可未加补丁；写入前必须采用与当前适配器匹配的补丁版本。`check`只离线检查文件、消息和数值库，不证明设备已就绪。

**4．按现场核实流程启动底层，再启动Bindu只读装配。** 底层可能运动，须先核对空手／支撑、空间、急停及启动回位行为；本文不提供未经现场核实的一键电机启动命令。已有控制进程保持运行时，先核对其版本与状态，不为运行Bindu重复启动。上层四节点启动命令为：

```bash
bash tools/snowbot_teleop.sh start --config artifacts/real-teleop/site-readonly.json
```

另开观察终端，在同一仓库设置与启动终端一致的四个`BINDU_*`变量、`SNOWBOT_VENDOR_NS`及ROS域／RMW设置。脚本加载的环境只在子进程内生效，手工执行`ros2`命令前还须在观察终端加载：

```bash
source /opt/ros/humble/setup.bash
source "$BINDU_VENDOR_SETUP"
source "$BINDU_VENV/bin/activate"
source "$BINDU_TELEOP_WORKSPACE/install/local_setup.bash"
bash tools/snowbot_teleop.sh status --config artifacts/real-teleop/site-readonly.json
ros2 topic info -v "${SNOWBOT_VENDOR_NS%/}/dualArmCtl_fdback"
ros2 topic echo "${SNOWBOT_VENDOR_NS%/}/dualArmCtl_fdback" --once
ros2 topic echo /bindu_snowbot/execution/state --once
```

核对20轴反馈的关节顺序、rad／rad/s、实物方向及零偏，`ExecutionState.joints`为实测、`reference`为命令参考；底盘零字段为缺省值，里程计反馈仍待接入。模式应处于SAFE，未选择模式、未申请会话，且Bindu没有底层运动命令／心跳发布者。`status`检查上层状态、profile、身份与时效，PDO健康及底层运行状态需单独核对。

**5．连接PICO，先验输入，再核对写入前提。** 头显访问输入主机`https://实际IP:8012`，使用现场受信任且地址匹配的证书，进入VR；本配方没有PICO协调器和图形状态面板，不以页面出现橙色反馈末端作为本入口就绪条件。保持Squeeze释放，查看输入：

```bash
# 在观察终端持续看双手输入，Ctrl+C结束观察后再执行下一条。
ros2 topic echo /bindu_snowbot/teleop/vr/input
# 以下健康话题要求底层已应用配套补丁。
ros2 topic echo "${SNOWBOT_VENDOR_NS%/}/bindu/control_source" --once
# 连续观察sample与sample_age，Ctrl+C仅结束这条观察命令。
ros2 topic echo "${SNOWBOT_VENDOR_NS%/}/bindu/servo_health"
```

连续观察左右输入，核对`valid`、`side`、`mapping=pico_snowbot_v1`、`runtime_mode=real`、位姿方向和Squeeze／Y含义。缺少手柄帧先解决TLS／网络／追踪，不提交会话。写入前核对底层实际`source=topic`、`guard_enabled=true`、`ready=true`、总线正常、WKC匹配、故障掩码、PDO样本持续推进和年龄；只读阶段`guard_armed=false`本身不是异常。PDO更新通过原始采样号、年龄与成功样本推进确认。健康消息、反馈发布者和内部两段ROS路由须与配置一致，详见[底层适配](../../hardware/bindu_hardware/README.md#snowbot-位置控制适配层)。

**6．现场获准后切换写入，先做最小跟随。** 保持手柄释放，退出只读Bindu，确认旧四节点从ROS图中清退。核对应用补丁的底层已按现场流程以`control_source:=topic`、`bindu_guard:=true`运行；这两个参数作用于命令保护，底层启动回位仍保留。按步骤3同样的TLS／前缀／接收节点参数生成另一份配置，额外加`--enable-commands`，再检查和启动：

```bash
bash tools/snowbot_teleop.sh configure --config artifacts/real-teleop/site-write.json \
  --host 0.0.0.0 --cert-file /绝对路径/cert.pem --key-file /绝对路径/key.pem \
  --vendor-namespace "$SNOWBOT_VENDOR_NS" --enable-commands
bash tools/snowbot_teleop.sh check --config artifacts/real-teleop/site-write.json
bash tools/snowbot_teleop.sh start --config artifacts/real-teleop/site-write.json
```

若步骤3还配置了自定义robot/profile/tool或接收节点，写入副本也须保留同样配置，只改变写权限。先用已核实的底层操作将空手双臂置于符合名义握杆几何的姿态，核对实测FK与实物，再选择模式；任意姿态不保证能通过双手几何检查，不以关闭检查消除拒绝。观察终端执行：

```bash
ros2 topic echo /bindu_snowbot/teleop/mode_state --once
read -r -p '填入刚观察到的 generation: ' SNOWBOT_MODE_GENERATION
ros2 action send_goal /bindu_snowbot/teleop/select_mode \
  bindu_interfaces/action/SelectTeleopMode \
  "{mode: bimanual_tool, side: '', expected_generation: $SNOWBOT_MODE_GENERATION, timeout: 10.0}" --feedback
# 确认模式成功切换，并收到切换后的新鲜左右手柄及关节反馈，再开一个短会话。
ros2 action send_goal /bindu_snowbot/teleop/tool_session \
  bindu_interfaces/action/TeleopSession \
  "{task_id: snowbot_onsite_check, resource_group: bimanual_tool, duration: 30.0}" --feedback
```

双侧先释放再同时握住Squeeze一秒，确认首次接合在当前实测工具／手柄姿态建立基准，无跳变。先做现场限定的小范围、低速位姿输入，再验证松任一Squeeze停止、重新握持重建基准、Y结束会话、受控输入断流后保持及新会话拒绝／恢复条件；逐项观察真实位置和速度确认停稳。会话到时是控制结束，不是任务成功。当前工具跟随会话中按Trigger、X、A、B或摇杆点击会停止会话，不执行虚握／准备；Trigger的真实开合须先结束臂会话并按下节另行取得夹爪租约、授权装卸。

**7．持铲验收单独进入。** 空手跟随及停止通过后，才在实体夹持、外部支撑、工具安装、载荷和保持条件已核实的实验范围内做固定刚握的小范围持铲测试。基本开合位置闭环已有独立入口；软件刚握约束已提供；完整扫雪所需的实体夹持确认、右手虚握／回锁及六维力／滑移接口仍待接入；本次最小入口不能完成仿真中的全部倒雪换握流程。

**8．收尾。** 松Squeeze、Y结束会话，确认实测速度收敛且停止没有未确认项，再在Bindu启动终端Ctrl+C，检查退出结果和节点清退。底层控制的失能／断电／总线关闭按现场核实流程单独处理，先处理持物支撑；Bindu软件保持不是硬件急停。出现`STOP_UNCONFIRMED`、反馈/PDO冻结、路由冲突或模型不符时结束本轮，不通过延长TTL、绕过健康门槛或反复重启带载设备继续。记录源码提交、目标环境、现场配置、反馈／保持结果和阻塞点；结构化记录器沿用当前关闭设置。



### 部署自动化

[`tools/snowbot_teleop.sh`](../../../tools/snowbot_teleop.sh)提供下列命令，复用现有四节点真机配方；辅助校验在[`snowbot_teleop.py`](../../../tools/deployment/snowbot_teleop.py)。默认面向Humble，不安装系统依赖、不启动底层控制、不自动选择模式或开启会话。先完成[环境准备](../../../运行与验证.md#环境准备与构建)及匹配的Pinocchio／ProxSuite／SciPy依赖安装，在新终端设置现场路径：

```bash
export BINDU_ROS_DISTRO=humble
export BINDU_VENDOR_SETUP=/绝对路径/底层消息工作区/install/local_setup.bash
# 按实际依赖安装方式选填；venv须继承系统ROS依赖。
export BINDU_VENV=/绝对路径/遥操作venv
# 若使用pip --target而非venv安装数值依赖，再填写：
# export BINDU_TELEOP_DEPS=/绝对路径/teleop-deps
```

脚本依次加载ROS、底层消息和可选venv，补充cmeel库路径。`build`使用独立的`artifacts/real-teleop/{build,install,log}`，标准安装遥操作依赖链15包，指定当前Python和Release；其他命令加载该目录的`install/local_setup.bash`。可通过`BINDU_TELEOP_WORKSPACE`更换整个构建目录；不复制其他架构／ROS／Python环境的产物。开发机Jazzy测试须显式改为`BINDU_ROS_DISTRO=jazzy`，已有发行版与指定值冲突时拒绝继续。

```bash
# 无ROS依赖也能生成本机只读配方；已存在的文件不会覆盖。
bash tools/snowbot_teleop.sh configure
bash tools/snowbot_teleop.sh build
bash tools/snowbot_teleop.sh check
bash tools/snowbot_teleop.sh start
# 另开已设置相同环境变量的终端，只读观察3秒：
bash tools/snowbot_teleop.sh status --seconds 3
```

默认配置为`artifacts/real-teleop/site.json`，在本机修改副本后同步，机器人端不直接编辑；源码示例保持默认只读。每条命令可用`--help`；`configure/check/start/status/gripper`可用`--config /绝对路径/site.json`指定文件。夹爪位置操作及VR装卸授权通过`gripper`子命令，见[夹爪闭环](../../hardware/bindu_hardware/README.md#snowbot真实夹爪位置闭环)。局域网PICO接入示例（证书须在现场有效且受头显信任；此处填写证书文件路径）：

```bash
bash tools/snowbot_teleop.sh configure --config artifacts/real-teleop/site-lan.json \
  --host 0.0.0.0 --cert-file /绝对路径/cert.pem --key-file /绝对路径/key.pem \
  --vendor-namespace /snowbot
bash tools/snowbot_teleop.sh check --config artifacts/real-teleop/site-lan.json
bash tools/snowbot_teleop.sh start --config artifacts/real-teleop/site-lan.json
```

现场底层沿用根话题时，生成配置须选择`--vendor-namespace /`；默认接收节点身份由此前缀推导，非默认名称使用`--arm-receiver-node`、`--servo-receiver-node`及`--mid-receiver-node`显式填写完整绝对名称。ROS域、RMW及跨机发现设置沿用当前终端，脚本打印但不修改；需要跨机DDS时须核对`ROS_LOCALHOST_ONLY`，不能照搬本机隔离测试值。

`check`离线校验最小配方、安装配置与源码一致性、本体／模型／限位、消息原生typesupport、执行扩展、QP ABI／几何及TLS证书私钥匹配，不启动ROS节点。`start`先执行相同检查，再只读查询同命名空间已有节点并拒绝明显重复启动，随后前台运行launch；发现检查不提供跨进程原子锁。`status`只订阅真实模式执行／模式状态，检查profile、发布者身份与反馈时效，缺失、过期或停止未确认返回非零；不发送目标、租约或硬件命令，也不替代底层健康／停稳与机械验收。

默认`command_enabled=false`。仅在已获准的运动实验中，才给`configure`增加`--enable-commands`生成另一份允许写入的现场配方，并完成下文底层看门狗与保持核查。启动后仍按下文显式选模式、开启Action和Squeeze接合。退出前先释放Squeeze并确认停稳，再在启动终端按Ctrl+C；脚本不使用`pkill`、不自动重启，也不启动电机控制进程。现场急停和带载保持仍使用已核实的底层能力。

脚本回归：`python3 -m tests unit test_snowbot_teleop_tools`。已完成的检查覆盖构建、离线逻辑与合成ROS状态；目标Humble／Orin重建、现场TLS／PICO及真机验证仍待完成。



### 配方与操作接口

选择[snowbot_real_teleop.json](../../../../../snowbot/src/snowbot_bringup/config/deployments/snowbot_real_teleop.json)，装配仅有`execution`（`snowbot_adapter`）、`teleop_mode`、`teleop_tool`、`vr_input`四个ROS节点，QP子进程由工具节点管理。位置控制覆盖双臂各J1～J7及两轴腰部；头／第8轴作为实测上下文。不启动记录器、单臂、描述／显示、导航、任务、感知或VLA；ROS必要诊断仍保留。包安装依赖尚未裁剪。

先加载匹配的ROS、底层`snowbot_ctl_msgs`和Bindu工作区，运动学依赖按[环境准备](../../../运行与验证.md#环境准备与构建)配置。此命令启动上层只读装配，不启动底层设备：

```bash
ros2 launch bindu_runtime teleop_only.launch.py \
  config:=$(ros2 pkg prefix --share bindu_runtime)/config/deployments/snowbot_real_teleop.json
ros2 topic echo /bindu_snowbot/execution/state
ros2 topic echo /bindu_snowbot/teleop/mode_state
```

配方默认`command_enabled=false`、`host=127.0.0.1`且TLS为空，仅供本机检查。现场在配方中填写实际监听地址、证书／私钥**路径**、底层话题前缀和接收节点身份；头显连接要求见[VR入口](../../capabilities/bindu_teleoperation/README.md#vr-遥操作运行入口)，位置接口及健康／看门狗条件见[底层适配](../../hardware/bindu_hardware/README.md#snowbot-位置控制适配层)。写入实验经现场确认后才在部署配方开启`command_enabled`。

启动默认`SAFE / MODE_NOT_SELECTED`，不回位、不准备、不使能电机、不清零。按用户当前要求，现场底层待命／保持及夹爪接口先作为可用条件准备，正式测试前再核实；旧底层源码仍包含启动回位／六维清零；待命改造与带载重启仍待实现和验证。

当前真机配方使用`pico_snowbot_v1`相对位姿和Squeeze接合，Y停止并结束会话；**不启动模拟PICO协调器**。Trigger、X、A、B及摇杆点击均会停止活动工具会话；实体开合需先结束臂会话、通过独立夹爪租约显式授权，虚握／准备／确认仍未接通，第8轴模拟角不能用于真机。释放按键并确认停稳后，可以显式新开会话，旧目标不自动续跑。

现场核实实测握姿／安装／支撑后，保持双侧Squeeze释放，先向`teleop/select_mode`发送`SelectTeleopMode`（`mode=bimanual_tool`、空`side`、当前`expected_generation`、`timeout≤15`）；成功后等待新状态／输入，再向`teleop/tool_session`发送`TeleopSession`（非空`task_id`、`resource_group=bimanual_tool`、有效`duration`）。然后两侧各先释放再同时握住Squeeze一秒；以当前实测工具和手柄建立基准。模式选择后仍需显式启动会话，实体夹持由独立反馈确认。

真机本体索引为[robot_snowbot_real.json](../../../../../snowbot/src/snowbot_bringup/config/robots/snowbot/robot_snowbot_real.json)，引用独立位置运行profile和名义工具几何；没有仿真初始关节或演示轨迹。头部第2轴取源接口±0.791与CAD±0.79 rad的交集，保持启动严格校验；其余限位／速度沿用源接口，工具安装、地面和动态参数需现场标定。

以下检查自行启动替身控制器及真实模式上层，使用隔离本机ROS域，不连接EtherCAT或真实机器人；默认只读及替身写入分别验证：

```bash
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=223 python3 -m tests.integration.validate_real_teleop \
  --output artifacts/snowbot-real-wire-check
# 当前PICO位置链路；普通双臂与工具模式共用真实WebSocket接收器。
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=224 python3 -m tests integration position \
  --mode simulation --teleop-mode both --websocket-input --output artifacts/position-check
```



### 手动选择本体与自由度

Snowbot 与二代轮式是两个独立真机目标，分别维护配置，不共享型号专用关节映射、标定或握持参数。`robot_snowbot.json` 当前可运行已有运动学模拟；`robot_wheel_gen2.json` 是明确未完成的本体登记，`model/profile/teleop` 为 `null`，检查会返回 `ROBOT_CONFIG_INCOMPLETE:wheel_gen2`，不会回退到 Snowbot。G1 的 `robot_g1_fixture.json` 仅作既有模型兼容验证。

默认`teleop_only.launch.py`读取`config/deployments/teleop_robot_deployment.json`（schema 2），通过`robot_config`选择模型、profile、单臂和工具配置。旧`teleop_snowbot_deployment.json`示例已并入该入口；schema 1解析与`snowbot_sim.launch.py`参数仍兼容。分类位置与升级要求见[运行装配的配置目录](#配置目录)。

```bash
# 构建并加载工作区后，先只检查配置；不会启动ROS节点或设备。
ros2 run bindu_runtime check_robot /absolute/path/robot_config.json
# 直接选择本体配置，默认仅运动学模拟、无启动动作。
ros2 launch bindu_runtime robot_teleop.launch.py robot_config:=/absolute/path/robot_config.json
# 或使用部署文件同时选择命名空间、输入源、显示和日志。
ros2 launch bindu_runtime teleop_only.launch.py config:=/absolute/path/deployment.json
```

实现分布在现有包内，没有新增ROS包或另一套求解器：

| 位置 | 新增文件与职责 |
|---|---|
| `src/integration/bindu_runtime/bindu_runtime/` | `robot_setup.py` 校验本体；`teleop_assembly.py` 统一新旧启动装配 |
| `src/integration/bindu_runtime/config/` | `robot_snowbot.json`、`robot_wheel_gen2.json` 分别登记两台真机目标；`robot_g1_fixture.json` 只作回归；`teleop_robot_deployment.json` 选择启动参数；`position_mapping_wheel_gen2.json` 描述协议映射 |
| `src/integration/bindu_runtime/launch/` | `robot_teleop.launch.py` 通用入口 |
| `src/hardware/bindu_hardware/bindu_hardware/` | `position_mapping.py` 双向位置转换，不负责设备通信 |
| `tests/` | `test_robot_configuration.py` 检查自由度组合、配置拒绝及协议映射 |

本体配置结构参考 `src/integration/bindu_runtime/config/robots/snowbot/robot_snowbot.json`：

- `model`：控制URDF、显示URDF和基座frame；机械尺寸、转轴和机械限位来自模型。
- `profile`：具名执行组与运行限制；每个关节只能属于一个执行组。工具配置通过组列表定义共享身体及左右臂的关节，自由度由列表计算。
- `teleop.single_arm/tool`：引用匹配本体的单臂及工具控制配置；工具安装变换和握持约束仍由原工具配置维护。
- `position_mapping`：可选的独立位置协议映射文件。模型使用自身关节名和SI量；当前映射支持转动关节的rad/deg、方向和偏置。偏置以协议单位表示，`wire = direction × unit_scale × model + offset`，反馈反向转换。机械零位已由底层校准时，不在此重复添加编码器零位。

本体配置中的相对路径以**本体配置文件**为基准，模型可用 `package://包名/相对路径` 从ament安装索引查找。既有单臂/工具配置内部的`urdf`和工具碰撞`demo.display_urdf`仍相对所选控制URDF目录，单臂`collision_model`相对单臂配置文件；复制配置时须保留这些路径约定。启动前检查模型树、标量关节类型、分组和限位覆盖、末端frame、左右组对应以及完整实测上下文。控制与显示模型的关节树、类型、转轴及固定/活动关节安装变换须一致，显示模型不能多出无反馈的活动轴；工具碰撞检查必须引用所选显示模型。这是模型和配置的交叉检查，求解器及工具几何的进一步预检仍在建立会话时执行。固定/被动/mimic并非“补零的活动关节”：当前求解器不支持的关节类型或耦合会明确拒绝；不参与求解的活动关节仍须在profile中提供反馈。非零区间限位现在合法，运动学模拟仅为这些轴选择区间内的初值，不据此生成真实准备动作。

配置摘要包含模型自由度、工具活动关节、测量上下文和SHA256。新入口的Bindu运行节点复核模型、profile、工具、单臂/碰撞配置及可选映射的同一指纹，外部`robot_state_publisher`使用装配时读取的URDF；发现启动期间配置变化就拒绝启动。更换本体应停用并重启装配，不在运动会话中热换。

**二代位置协议映射范围。** `position_mapping_wheel_gen2.json` 根据2026-09-20资料中的`ipc_protocol.hpp`和`rt_robot_config.hpp`定义：14臂轴及2躯干轴映射到48通道，两个躯干模型角均取硬件角的负值，其余32通道明确列为保持通道。`PositionMapping`只负责严格双向转换与整包组装；须先由调用方确认反馈新鲜、控制权及接管条件，再捕获保持值。无捕获、部分目标、缺失/NaN反馈、重复映射会拒绝，坏反馈、标定换算后的非有限结果或失败重捕获会废弃旧保持值。此映射不是ROS硬件驱动，不能把保持手指角度等同于可靠夹持，也未实现`tight`到灵巧手的实体动作。

二代资料中的`dexbot.urdf`含12个腿部活动关节，而当前接口未提供其反馈，也没有颈部/手部完整运动链；需核对实际轮式模型和工具安装后填写该本体的model/profile/teleop。Snowbot使用上节独立底层位置适配入口，不能沿用二代48通道协议；通用遥操作入口的`soem_ros2`等未实现后端仍在节点启动前拒绝，原有模拟保护保留。



### 8.3.1 按需装配与现场转接

用户于2026-10-09明确：现场不一定能部署全部代码或直接适配现有动作接口，可能需要增加转接层，部分模块也不会启动。上面的部署单元是职责划分；实际部署按所需能力和已有平台选取，功能链路不要求每个环节都使用 Bindu 的节点。

| 接入范围 | 部署方式与职责 |
|---|---|
| 沿用 Bindu 执行链 | 部署所需输入、求解和执行模块，在设备边界适配实际命令、停止及反馈接口 |
| 复用现场执行系统 | 按接口能力选取 Bindu 输入映射、运动学等模块，在目标或关节输出边界增加转接；由现场控制器承担其已有的执行职责 |
| 只验证部分能力 | 启动对应测试或离线工具；无关的任务、导航、感知、VLA、可视化等模块可不部署或不启动 |

转接层按实际差异实现，不预先要求增加统一网关。需要核对的内容包括关节命名与单位、坐标系、命令类型、时间与有效期、接纳与完成状态、取消/停止及故障反馈。现场缺失的动作能力标为未接入或不支持，成功状态和实测反馈仅由已接入的设备路径提供；该能力暂不可用不要求无关能力同时停止建设。接口字段与执行时序仍以[动作执行契约](../../../动作执行契约.md)为准，现场接入需明确能实现的子集及差异。

每条实际运动链路应明确控制权、插值、限幅、设备周期和停止的负责人；这些职责可以由现场系统承担，不要求重复部署 Bindu 对应节点。既有控制器已完成的插值或队列处理不再重复串接，同一资源保留唯一运动写入方。更换执行边界后，须基于实际反馈重新验证跟踪、取消和断流，原仿真结果只覆盖原装配。

真机目标为Ubuntu 22.04／ROS 2 Humble，接收关节位置命令，具体SDK待核实。准备与装卸属于显式操作，不混入连续工具位姿；实体夹爪及反馈需单独适配，接口条件统一见[工具操作契约](../../../动作执行契约.md#61-snowbot-工具操作与握持切换)，模拟装配见[运行入口](../../capabilities/bindu_teleoperation/README.md#遥操作专用装配与暂停)。

**当前实现边界：** `teleop_only.launch.py` 已通过配置复用 Snowbot 模拟装配；`g1_sim.launch.py` 已有遥操作、双臂、VR、Pi、导航等部分开关，各示例 launch 的开关并不一致；`bindu_runtime`的包依赖仍集中声明，停用节点时仍保留安装依赖。当前遥操作会话依赖执行服务和记录器健康状态，`RuntimeNode` 仍拒绝真实设备模式。模块裁剪、最小依赖打包和现场转接需针对具体部署分别实现并验证。



### 8.4 单机首期的模式与配置

借鉴 Galbot 的管理、业务就绪和配置分层，但首期不自建多级 launcher。一个 RuntimeSupervisor 装配配置/依赖；本地执行器强制检查权限，Supervisor 不承担高频采样。

2026-10-14已实现配置级环境选择：`runtime_mode=simulation|real`由启动装配统一传给执行、记录和观察节点，与本体选择及下述业务模式分别表达。公共环境解析位于`bindu_contracts/runtime_mode.py`，具体设备路由仍由运行装配负责；Snowbot可选择运动学／外部仿真或底层接口，真实接口默认只读，未实现的真实遥操作、任务和VLA装配显式拒绝启动。命令、状态、事件和输入的环境一致性与兼容规则集中于[执行契约3.3](../../../动作执行契约.md#33-已实现的仿真与真实环境选择2026-10-14)，配置入口见[运行说明](../../../运行与验证.md#统一仿真与真实模式选择)。模式选择后需核对设备连接和反馈有效性，真机验证状态单独记录。

首期模式：OBSERVE（只读）、TELEOP（示教控制）、AUTO（自主任务）、REPLAY（离线消费，真机执行另经正常合同）。readiness 按当前技能所需能力求交集：单臂实验不等待 ASR/Nav2，传统抓取不等待 VLA，示教必须检查记录能力。模式切换先撤销新输入、完成现有资源收敛、装配依赖，再发布模式可用；模式名更新不是就绪证据。

配置顺序建议：defaults → robot → site → task profile → 显式运行覆盖。启动解析完整依赖并生成 ConfigSnapshot（来源、generation/hash、有效值）。同一任务/重试沿用固定快照；首期不实现任意热更新。URDF、标定、模型或控制拓扑修改，在停止后显式重建 Runtime，禁止后台 watcher 修改正在运行的控制合同。
