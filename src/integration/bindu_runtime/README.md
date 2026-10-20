# bindu_runtime：ROS 运行装配

[项目模块导航](../../../README.md#模块快速导航) · [编译与运行](#编译与运行) · [职责与边界](#职责与边界) · [配置目录](#配置目录) · [扫雪遥操作](#扫雪遥操作) · [源码导航](#源码导航) · [接入与验证](#接入与验证) · [关联模块](#关联模块)

提供 ROS 2 节点、launch 与配置，把公共契约、执行库、设备和能力组合为可启动的运行实例。算法核心仍在各自模块中维护。

## 编译与运行

以下命令均在仓库根目录执行，使用与 ROS 消息 ABI 匹配的系统 Python。示例为 Ubuntu 24.04／Jazzy；Ubuntu 22.04／Humble 将 `jazzy` 换为 `humble`，不要在同一终端混用发行版。基础 ROS、colcon、rosdep 和各包依赖按 [环境准备](../../../运行与验证.md#环境准备与构建) 安装。

### 依赖准备

运行包依赖运动学及原生 QP，即使只启动任务示例，首次构建也要准备 C++17、Python 开发头文件、Eigen3、SIMDe，以及匹配的 Pinocchio／ProxSuite。已有可用数值环境可直接加载；若使用仓库固定版本的 wheel，先在新的终端准备隔离目录：

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
- 真实 Snowbot：优先使用 [x86 独立部署](#x86-真机部署已有适配底层)；底层联调沿用 [现场部署顺序](#snowbot-真机最小遥操作)，准备厂商消息、现场配方与反馈，默认只读；构建和启动上层不等于底层使能或真机验收。

## x86 真机部署（已有适配底层）

目标机使用与本机相同的已适配厂商 `src`，支持 Ubuntu 22.04／ROS 2 Humble／Python 3.10，或 Ubuntu 24.04／Jazzy／Python 3.12。真机保留浏览器运动学显示，不需要 Isaac。新机重新编译，不复制本机 `install/`、`.so`、venv 或串联历史实验目录的 `artifacts/vr/env.sh`。

**1．准备系统依赖与现有底层消息。** 先安装对应 ROS 2。在新终端进入本仓库，以下以 Jazzy 为例，22.04 改用 Humble：

```bash
source /opt/ros/jazzy/setup.bash
sudo apt-get update
sudo apt-get install -y build-essential python3-dev python3-venv \
  python3-colcon-common-extensions python3-rosdep libeigen3-dev libsimde-dev \
  python3-yaml python3-zmq "ros-$ROS_DISTRO-nav2-msgs" "ros-$ROS_DISTRO-robot-state-publisher"
# 首次使用 rosdep 时先执行 sudo rosdep init；已初始化不重复执行。
rosdep update
mapfile -t bindu_packages < <(colcon list --base-paths src --packages-up-to bindu_runtime --paths-only)
rosdep install --from-paths "${bindu_packages[@]}" --ignore-src --rosdistro "$ROS_DISTRO" \
  --skip-keys 'pinocchio proxsuite' -r -y
```

Pinocchio／ProxSuite 由下一步固定版本 wheel 提供。这里只解析运行依赖链，不安装可选 LIO／Isaac。底层 `src` 在自己的工作区按既有流程构建；若只准备上层编译和只读检查，可在底层工作区用 `colcon build --packages-select snowbot_ctl_msgs` 仅构建消息。不要为迁移重新套补丁，也不要启动控制器；`local_setup.bash` 必须来自目标 ROS／系统 Python 重新生成的消息安装，不能复制另一机器的二进制。

**2．独立准备并构建 Bindu。** [`x86_real.py`](../../../tools/deployment/x86_real.py) 自动核对 OS／架构／Python 配对，从系统 Python 建立继承 ROS 系统包的 venv，安装[完整固定依赖清单](../../../tools/requirements/requirements-x86-real.txt)，然后构建运行依赖链15包：

```bash
# 将路径替换为新机已适配厂商工作区的实际安装位置。
python3 -m tools.deployment.x86_real prepare \
  --vendor-setup /绝对路径/vendor_ws/install/local_setup.bash
python3 -m tools.deployment.x86_real build
```

默认产物均在忽略入库的 `artifacts/x86-real/`，可在每条命令加同一个 `--workspace /绝对路径/部署目录`。`prepare` 可在同一部署目录重试，保留已编辑的现场配方；拒绝接管无部署标识的非空目录或改换厂商环境。源码移动、系统／Python／ROS 升级时新建部署目录。生成的 `env.sh` 清除继承的 Python／ROS 覆盖层，只加载选定 ROS、厂商 `local_setup.bash`、独立 venv 和本次安装；保留调用方 DDS／ROS 域设置。`environment.json` 记录实际 Python、模块路径／版本、包前缀、提交及脏工作区状态，`requirements-resolved.txt` 记录本次解析的依赖版本；固定直接依赖不等于锁定所有传递依赖。

**3．核实现场配方后预检。** 编辑生成的 `artifacts/x86-real/site.json`：它来自统一 PICO 真机配方，默认 `command_enabled=false`，不是本机仿真配方。填写底层命名空间／接收节点；头显跨机访问时设置 `host=0.0.0.0` 及新机 IP 对应的 TLS 证书／私钥绝对路径。证书生成与头显信任步骤见[首次输入诊断](../../capabilities/bindu_teleoperation/README.md#vr-首次连通检查)。现场 PICO 准备位与夹爪标定用独立配置路径引用，未标定时不填模拟值、不改写标定状态。源码或依赖清单变更后重新 `build`；检查入口会按构建时的源码指纹拒绝旧产物。

```bash
ROS_DOMAIN_ID=42 python3 -m tools.deployment.x86_real check
# 现场已核实并处于授权实验范围内时，前台启动上层和浏览器；Ctrl+C 收尾。
ROS_DOMAIN_ID=42 python3 -m tools.deployment.x86_real start
```

底层与上层使用相同 ROS 域／兼容 RMW；分机通信另核对 DDS 发现与网卡配置，不能用强制 localhost 掩盖。电脑页面默认 `http://127.0.0.1:8013/`，头显使用现场 HTTPS 地址8012端口。该工具不启动厂商控制器、不使能电机；写权限仍由已审阅的现场配方显式决定。底层本身启动可能回零／清零，持物时遵守[现场启动限制](#现场部署顺序)。

`check` 在启动控制节点前实际加载执行层扩展、QP／Pinocchio 和 ROS 消息原生支持，构造并关闭 QP；核对安装中的 robot／profile／tool／PICO／夹爪配置与当前源码、TLS、端口和重复节点。它会短暂建立只读 ROS 图检查节点，不发送运动指令。底层新鲜反馈、手柄按键、现场准备位／夹爪标定、停机保持与实机性能仍须现场逐项验收；完整操作见[统一入口](../../../运行与验证.md#vr-统一部署与启动)。

## 职责与边界

- 负责消息转换、依赖选择、设备／遥操作装配、本体校验与进程生命周期。
- 启动时明确仿真／真实环境，检查指令和反馈模式一致性；环境选择与设备后端、写权限分别配置。
- 真实模式支持 Snowbot 双臂／腰部位置遥操作装配，默认只读；独立夹爪位置闭环默认未标定禁止写入；实体夹持／虚握确认、完整PICO流程、Pi和导航未完成真机接入。
- 配置选择不替代真实模型标定、设备能力核实和现场验证。

## 配置目录

| 位置 | 维护内容 |
|---|---|
| [config/deployments/](config/deployments) | 用户选择的运行入口，引用下列参数 |
| [config/robots/](config/robots) | 按Snowbot、G1、二代轮式及旧Huawei模型分组的本体、执行、遥操作和工具参数 |
| [config/navigation/](config/navigation) | 通用导航场景、站点、LIO及定位参数 |
| [config/vla/](config/vla) | Pi客户端与回环示例 |
| [config/simulation/](config/simulation) | 通用模拟执行参数 |
| [tests/config/](../../../tests/config) | 验证专用夹具，不安装到运行包 |

Snowbot真机最小配方为`snowbot_real_teleop.json`，使用独立的真实本体、位置运行profile和工具几何配置；默认只读且不启动记录器。

正式运行配置共40份，安装时保持相同目录结构。旧`teleop_snowbot_deployment.json`并入统一`deployments/teleop_robot_deployment.json`；schema 1读取能力仍兼容。旧平铺路径已迁移，升级需重建运行包并更新外部脚本中的配置路径。

## 扫雪遥操作

### 场景选择

| 目的 | 当前入口 | 反馈与验证范围 |
|---|---|---|
| 工具位姿输入、模式切换与暂停／恢复 | [teleop_only.launch.py](launch/teleop_only.launch.py)，默认加载 [teleop_robot_deployment.json](config/deployments/teleop_robot_deployment.json) | ROS 运动学模拟；默认 `tool_pose` 输入，现场 VR 待接入 |
| 整段双臂协调、CAD几何与实时回放 | [run_shovel_physics.sh](../../../tools/run_shovel_physics.sh)，使用 `--kinematic ... --gui` | 日常运动学检查；生产QP／执行器生成输入，Isaac 6.2核对关节和法兰并回放，无承载／接触验收 |
| 短时重力、夹爪接触与载荷 | 同一入口，使用 `--closed-loop` | 默认contact，有限力矩J8与CAD接触；私有IPC复用求解／会话／执行核心；旧弹簧观测须显式选proxy，不启动ROS节点 |
| Snowbot 真机最小遥操作 | 同一launch，选择[snowbot_real_teleop.json](config/deployments/snowbot_real_teleop.json) | VR接收、模式、双臂QP、位置适配共4节点；不启动记录、单臂、显示和任务模块，默认只读；[操作边界](#snowbot-真机最小遥操作) |
| Snowbot 底层位置接口联调 | [snowbot_adapter.launch.py](launch/snowbot_adapter.launch.py) | 独立位置入口，默认真实模式且只读；不包含遥操作节点 |

`runtime.launch.py` 的默认 [runtime_deployment.json](config/deployments/runtime_deployment.json) 选择位置适配入口的仿真模式。运行扫雪工具会话从 `teleop_only.launch.py` 进入；部署时选择完整的`snowbot_vr_real.json`或底层联调配方`snowbot_real_teleop.json`，不能只切换`runtime_mode`。

统一PICO入口为[`tools/run_vr.sh`](../../../tools/run_vr.sh)，选择`snowbot_vr_sim.json`或`snowbot_vr_real.json`。两者使用`position_contract=true`、同一真实位置profile／工具几何和协调器；只在设备IO、初始测量状态及现场标定处区分。状态统一到`teleop/state`，夹爪由同一执行节点提供服务。旧四节点配方保留底层联调用途；具体启动与操作见[统一入口](../../../运行与验证.md#vr-统一部署与启动)。

### 配套配置与流程

真机配方增加`snowbot_grippers_real.json`，默认`calibrated=false`禁止夹爪写入；位置闭环及授权Trigger操作见[真实夹爪接口](../../hardware/bindu_hardware/README.md#snowbot真实夹爪位置闭环)。夹爪服务由同一execution节点提供，原四节点范围不变。底层需配套`bindu_gripper_control`保护入口，实体持物／虚握不由位置到位推断。

| 配置 | 作用 |
|---|---|
| [robot_snowbot.json](config/robots/snowbot/robot_snowbot.json) | 绑定本体模型、执行 profile、单臂与工具配置 |
| [snowbot_shovel_sim.json](config/robots/snowbot/snowbot_shovel_sim.json) | 资源组、动态限制和同步位置执行；联合控制两轴腰部与双臂各七轴 |
| [robot_snowbot_real.json](config/robots/snowbot/robot_snowbot_real.json) | 真机模型／profile／工具索引，不包含单臂配置或模拟初始姿态 |
| [snowbot_real_position.json](config/robots/snowbot/snowbot_real_position.json) | 真实位置运行限制；头部第2轴取源接口与CAD交集±0.79 rad，其余沿用源接口，仍需现场核实 |
| [bimanual_snowbot_real.json](config/robots/snowbot/bimanual_snowbot_real.json) | 真机位置链的名义工具安装／CAD几何／QP与时效参数；不包含演示轨迹，安装尺寸和地面需标定 |
| [snowbot_grippers_real.json](config/robots/snowbot/snowbot_grippers_real.json) | 独立夹爪位置闭环、超时与开合参数；默认未标定，拒绝写入 |
| [bimanual_snowbot_shovel.json](config/robots/snowbot/bimanual_snowbot_shovel.json) | 工具安装、握持约束、姿态策略、几何与输入时效 |
| [snowbot_shovel_physics.json](config/robots/snowbot/snowbot_shovel_physics.json) | 空铲木质质量、接触夹持、Fabric与GUI刷新；保留显式proxy对照参数 |

1. 构建并加载匹配的 ROS、Python 与原生 QP／执行库，检查本体配置；具体命令见[环境准备](../../../运行与验证.md#环境准备与构建)。
2. 启动遥操作装配。默认从零位等待，不自动取铲或运动到准备姿态；工具模式需要有效握姿和新鲜反馈。测试夹具初始化仅由专用模拟入口／验证器显式设置。
3. 持续提供有效工具输入，显式选择 `bimanual_tool` 模式，再开启 `teleop/tool_session`。暂停、恢复和只读显示见[遥操作专用装配与暂停](../../capabilities/bindu_teleoperation/README.md#遥操作专用装配与暂停)；模式 Action 见[Snowbot 双臂遥操作](../../capabilities/bindu_kinematics/README.md#snowbot-双臂与两轴腰部遥操作)。

以上为当前模拟装配步骤。实机目标的底层默认待命与重启条件见[启动方案](../../../软件架构设计.md#832-snowbot-实机启动与准备)；PICO的Squeeze／Trigger术语、实体键表和相对计算见[遥操作映射](../../capabilities/bindu_teleoperation/README.md#pico-按键方案)，空手装夹／持铲重接／作业卸载见[目标操作流程](../../capabilities/bindu_teleoperation/README.md#pico-扫雪操作流程目标方案)，装卸资格、虚握子状态和停止恢复见[执行契约6.1](../../../动作执行契约.md#61-snowbot-工具操作与握持切换)。独立PICO运动学入口已接入：选择[teleop_pico_snowbot_sim.json](config/deployments/teleop_pico_snowbot_sim.json)，`pico_enabled=true`且`tool_input_source=vr`；独立夹爪服务已接入统一位置配方，实体夹持与真实底层恢复仍待验收，旧默认装配不改映射。

当前底层源码仍有启动回位及自动六维传感器清零，`bindu_guard=true`不跳过这两步；必须同步修改底层默认启动路径，不能仅在GUI加按钮。现有看门狗也不允许新适配器直接接管旧会话。整底层进程重启的持载保持须单独验证；完成前持铲仍需支架辅助恢复，不能把目标待命方案视为已有能力。

工具输入与 `tight` 语义见[遥操作模块](../../capabilities/bindu_teleoperation/README.md#扫雪输入与握持)。`tight=false` 只释放右手绕杆轴转动，实体夹爪动作与握持确认仍待接入。

真机 VR 使用接合基准下的相对位姿增量，再生成机器人基座系工具目标。当前 `tool_pose` 接口消费该映射结果，模拟输入也在这一层注入；`vr` 会话已在内部实现相对映射，现场 VR 到统一工具消息的转接仍待接入。接入头显时需落实坐标标定、比例和松握重接基准，详见上面的输入说明。

真机最小配方仅接位姿／Squeeze／Y，须显式管理模式和工具Action；实体操作键停止会话，`tight=false`在真实模式下也会停止。完整边界见[真机最小遥操作](#snowbot-真机最小遥操作)。

### 物理反馈与柔顺

物理脚本默认做参考回放；`--closed-loop` 接实际关节反馈，另加 `--load-feedback` 才启用左腕虚拟六维载荷的握持变形观测。观测结果进入 QP 工具位姿跟踪，双臂载荷另行分析；目前没有双臂内力上限优化，也没有额外的上层导纳／阻抗控制器。底层已有柔顺，仿真参数尚未与实机标定。

实验采用固定底盘、木质刚体空铲与弹簧阻尼握持代理，没有真实指面滑移、木杆弯曲或雪载荷。完整生成输入、GUI 启动、评分与内力定义见[PhysX 受力实验](../../hardware/bindu_hardware/README.md#木质雪铲的-physx-受力实验2026-10-15)。

PICO运行节点：[pico_node.py](bindu_runtime/pico_node.py)；模拟开合参数：[pico_snowbot_sim.json](config/robots/snowbot/pico_snowbot_sim.json)。Squeeze／Trigger及面键先由接收器保留，再由协调器处理操作和控制权，实际夹爪／承载仍不得用模拟状态代替。

## 源码导航

| 入口 | 内容 |
|---|---|
| [setup.py](setup.py) | 全部 ROS 可执行入口与安装资源 |
| [plugins.py](bindu_runtime/plugins.py)、[device_assembly.py](bindu_runtime/device_assembly.py)、[simulation_devices.py](bindu_runtime/simulation_devices.py) | 能力选择、设备与模拟后端装配 |
| [teleop_assembly.py](bindu_runtime/teleop_assembly.py) / [teleop_deployment.py](bindu_runtime/teleop_deployment.py) | 遥操作依赖和部署配置 |
| [robot_setup.py](bindu_runtime/robot_setup.py) / [common.py](bindu_runtime/common.py) | 本体／自由度校验、ROS 转换与公共运行辅助 |
| [execution_node.py](bindu_runtime/execution_node.py) / [snowbot_adapter.py](bindu_runtime/snowbot_adapter.py) | 共享执行节点与底层位置适配 |
| [bindu_runtime/](bindu_runtime) | 遥操作、VR、Pi、导航、感知、任务、记录等节点源码 |
| [launch/](launch) / [config/](config) | 启动装配与配置全集 |

### 常用启动与配置入口

| 用途 | 启动文件 | 配置示例 |
|---|---|---|
| 环境选择与运行装配 | [runtime.launch.py](launch/runtime.launch.py) | [runtime_deployment.json](config/deployments/runtime_deployment.json) |
| 本体与遥操作模式选择 | [robot_teleop.launch.py](launch/robot_teleop.launch.py) | [robot_snowbot.json](config/robots/snowbot/robot_snowbot.json) |
| 旧 Snowbot 模拟入口兼容 | [snowbot_sim.launch.py](launch/snowbot_sim.launch.py) | [snowbot_shovel_sim.json](config/robots/snowbot/snowbot_shovel_sim.json) 与 [bimanual_snowbot_shovel.json](config/robots/snowbot/bimanual_snowbot_shovel.json) 须配套选择 |
| Snowbot 底层适配 | [snowbot_adapter.launch.py](launch/snowbot_adapter.launch.py) | [snowbot_vendor_position.json](config/robots/snowbot/snowbot_vendor_position.json) |
| 导航物理场景 | [navigation_physics.launch.py](launch/navigation_physics.launch.py) | [navigation_room.json](config/navigation/navigation_room.json) |
| 三维建图／地图定位 | [lio.launch.py](launch/lio.launch.py) / [localization.launch.py](launch/localization.launch.py) | [navigation_lio.yaml](config/navigation/navigation_lio.yaml) / [navigation_localization.yaml](config/navigation/navigation_localization.yaml) |

`runtime.launch.py` 读取部署配置并选择具体 launch；[teleop_robot_deployment.json](config/deployments/teleop_robot_deployment.json) 是选择 `robot_teleop.launch.py` 的部署示例。`robot_teleop.launch.py` 自身通过 `robot_config` 读取本体配置。上表配置为场景示例，所需参数及默认值见对应运行说明。

木质雪铲 PhysX 实验使用仓库级 [run_shovel_physics.sh](../../../tools/run_shovel_physics.sh) 和 [snowbot_shovel_physics.json](config/robots/snowbot/snowbot_shovel_physics.json)，具体反馈路径和模型限制见运行说明。

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

本批首轮构建因Conda旧`argparse`覆盖标准库失败，导航首轮因未加载隔离`nav2_msgs 1.3.13`失败；调整环境路径后通过，受影响的G1全身及左右VR/Pi完整启动另行复测通过，未修改运行源码或放宽阈值。Vuer连接断开及部分节点SIGINT收尾仍有异常日志；功能断言通过不代表退出过程完全正常。本批模拟进程已收尾，未运行现场头显、真实Pi模型、真实Nav2/SLAM或真机。

模型来源与修改范围见[架构](../../../软件架构设计.md#854-当前代码组织与替换边界)，生成命令与分发范围见[模型与资产](../../hardware/bindu_description/README.md#机器人模型与资产)。

此前9月18日Ubuntu/Jazzy验证：14包构建、103项模块、9项全身场景及5项相关导航回归通过。TF与FK按同一时间戳的实测样本对照；初轮用较早TF与最终目标比较而失败，修正验证器后通过，精度阈值未放宽。



### Snowbot 真机最小遥操作



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

默认配置为`artifacts/real-teleop/site.json`，现场修改集中在此文件，源码示例保持默认只读。每条命令可用`--help`；`configure/check/start/status/gripper`可用`--config /绝对路径/site.json`指定文件。夹爪位置操作及VR装卸授权通过`gripper`子命令，见[夹爪闭环](../../hardware/bindu_hardware/README.md#snowbot真实夹爪位置闭环)。局域网PICO接入示例（证书须在现场有效且受头显信任；路径不等于证书内容）：

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

脚本回归：`python3 -m tests unit test_snowbot_teleop_tools`。构建、离线检查和合成ROS状态验证均不能代替目标Humble／Orin重建、现场TLS／PICO或真机验收。



### 配方与操作接口

选择[snowbot_real_teleop.json](config/deployments/snowbot_real_teleop.json)，装配仅有`execution`（`snowbot_adapter`）、`teleop_mode`、`teleop_tool`、`vr_input`四个ROS节点，QP子进程由工具节点管理。位置控制覆盖双臂各J1～J7及两轴腰部；头／第8轴作为实测上下文。不启动记录器、单臂、描述／显示、导航、任务、感知或VLA；ROS必要诊断仍保留。包安装依赖尚未裁剪。

先加载匹配的ROS、底层`snowbot_ctl_msgs`和Bindu工作区，运动学依赖按[环境准备](../../../运行与验证.md#环境准备与构建)配置。此命令启动上层只读装配，不启动底层设备：

```bash
ros2 launch bindu_runtime teleop_only.launch.py \
  config:=$(ros2 pkg prefix --share bindu_runtime)/config/deployments/snowbot_real_teleop.json
ros2 topic echo /bindu_snowbot/execution/state
ros2 topic echo /bindu_snowbot/teleop/mode_state
```

配方默认`command_enabled=false`、`host=127.0.0.1`且TLS为空，仅供本机检查。现场在配方中填写实际监听地址、证书／私钥**路径**、底层话题前缀和接收节点身份；头显连接要求见[VR入口](../../capabilities/bindu_teleoperation/README.md#vr-遥操作运行入口)，位置接口及健康／看门狗条件见[底层适配](../../hardware/bindu_hardware/README.md#snowbot-位置控制适配层)。写入实验经现场确认后才在部署配方开启`command_enabled`。

启动默认`SAFE / MODE_NOT_SELECTED`，不回位、不准备、不使能电机、不清零。按用户当前要求，现场底层待命／保持及夹爪接口先作为可用条件准备，正式测试前再核实；已提供的旧底层源码仍包含启动回位／六维清零，不能将这个部署假设当作改造或带载重启验收。

当前真机配方使用`pico_snowbot_v1`相对位姿和Squeeze接合，Y停止并结束会话；**不启动模拟PICO协调器**。Trigger、X、A、B及摇杆点击均会停止活动工具会话；实体开合需先结束臂会话、通过独立夹爪租约显式授权，虚握／准备／确认仍未接通，第8轴模拟角不能用于真机。释放按键并确认停稳后，可以显式新开会话，旧目标不自动续跑。

现场核实实测握姿／安装／支撑后，保持双侧Squeeze释放，先向`teleop/select_mode`发送`SelectTeleopMode`（`mode=bimanual_tool`、空`side`、当前`expected_generation`、`timeout≤15`）；成功后等待新状态／输入，再向`teleop/tool_session`发送`TeleopSession`（非空`task_id`、`resource_group=bimanual_tool`、有效`duration`）。然后两侧各先释放再同时握住Squeeze一秒；以当前实测工具和手柄建立基准。模式选择不等于已经启动会话，软件刚握约束也不代表实体夹持确认。

真机本体索引为[robot_snowbot_real.json](config/robots/snowbot/robot_snowbot_real.json)，引用独立位置运行profile和名义工具几何；没有仿真初始关节或演示轨迹。头部第2轴取源接口±0.791与CAD±0.79 rad的交集，保持启动严格校验；其余限位／速度沿用源接口，工具安装、地面和动态参数需现场标定。

以下检查自行启动替身控制器及真实模式上层，使用隔离本机ROS域，不连接EtherCAT或真实机器人；默认只读及替身写入分别验证：

```bash
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=223 python3 -m tests.integration.validate_real_teleop \
  --output artifacts/snowbot-real-wire-check
# 可选：无记录器的PICO完整仿真回归，含真实WebSocket接收器。
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=224 python3 -m tests integration pico \
  --minimal --websocket-input --output artifacts/pico-minimal-check
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
- `profile`：具名执行组与运行限制；每个关节只能属于一个执行组。工具配置通过组列表选择共享身体及左右臂，自由度从列表计算，不能填一个维度数字代替关节定义。
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

转接层按实际差异实现，不预先要求增加统一网关。需要核对的内容包括关节命名与单位、坐标系、命令类型、时间与有效期、接纳与完成状态、取消/停止及故障反馈。现场缺失的动作能力标为未接入或不支持，不为满足接口形式伪造成功或实测反馈；该能力暂不可用不要求无关能力同时停止建设。接口字段与执行时序仍以[动作执行契约](../../../动作执行契约.md)为准，现场接入需明确能实现的子集及差异。

每条实际运动链路应明确控制权、插值、限幅、设备周期和停止的负责人；这些职责可以由现场系统承担，不要求重复部署 Bindu 对应节点。既有控制器已完成的插值或队列处理不再重复串接，同一资源保留唯一运动写入方。更换执行边界后，须基于实际反馈重新验证跟踪、取消和断流，原仿真结果只覆盖原装配。

2026-10-09 用户确认真机为 Ubuntu 22.04 / ROS 2 Humble，设备接收关节位置命令，具体 SDK 暂未明确。准备流程要求：显式请求后从实测姿态使双臂到达经验证的准备位置；停稳后允许人工放入工具；开爪采用按住按钮请求，释放或开始时请求闭合，确认实体夹持后再捕获工具并进入跟随。操作中请求完全开爪应先暂停并停稳；释放按钮闭合不自动恢复双臂运动，失联不自动开爪。完全开爪与右手轴向虚握为不同操作，不能复用 `tight=false` 表示完全打开。这是现场适配的流程约定，准备轨迹、开合命令/确认、夹持力和按钮失联处理尚无实体驱动实现，不能用示例初始关节角或软件握持状态代替。本轮先落地遥操作专用装配、显式暂停和状态反馈，操作命令与连续工具位姿载荷分开，见[运行入口](../../capabilities/bindu_teleoperation/README.md#遥操作专用装配与暂停)。

**当前实现边界：** `teleop_only.launch.py` 已通过配置复用 Snowbot 模拟装配；`g1_sim.launch.py` 已有遥操作、双臂、VR、Pi、导航等部分开关，各示例 launch 的开关并不一致；`bindu_runtime` 的包依赖仍集中声明，不启动节点不等于已移除安装依赖。当前遥操作会话依赖执行服务和记录器健康状态，`RuntimeNode` 仍拒绝真实设备模式。按需部署是装配原则，任意模块裁剪、最小依赖打包和现场转接均需结合具体部署实现，不能靠关闭节点或修改模拟标志视为完成。



### 8.4 单机首期的模式与配置

借鉴 Galbot 的管理、业务就绪和配置分层，但首期不自建多级 launcher。一个 RuntimeSupervisor 装配配置/依赖；本地执行器强制检查权限，Supervisor 不承担高频采样。

2026-10-14已实现配置级环境选择：`runtime_mode=simulation|real`由启动装配统一传给执行、记录和观察节点，与本体选择及下述业务模式分别表达。公共环境解析位于`bindu_contracts/runtime_mode.py`，具体设备路由仍由运行装配负责；Snowbot可选择运动学／外部仿真或底层接口，真实接口默认只读，未实现的真实遥操作、任务和VLA装配显式拒绝启动。命令、状态、事件和输入的环境一致性与兼容规则集中于[执行契约3.3](../../../动作执行契约.md#33-已实现的仿真与真实环境选择2026-10-14)，配置入口见[运行说明](../../../运行与验证.md#统一仿真与真实模式选择)。模式选择不代表设备已连接、反馈有效或真机已验收。

首期模式：OBSERVE（只读）、TELEOP（示教控制）、AUTO（自主任务）、REPLAY（离线消费，真机执行另经正常合同）。readiness 按当前技能所需能力求交集：单臂实验不等待 ASR/Nav2，传统抓取不等待 VLA，示教必须检查记录能力。模式切换先撤销新输入、完成现有资源收敛、装配依赖，再发布模式可用；模式名更新不是就绪证据。

配置顺序建议：defaults → robot → site → task profile → 显式运行覆盖。启动解析完整依赖并生成 ConfigSnapshot（来源、generation/hash、有效值）。同一任务/重试沿用固定快照；首期不实现任意热更新。URDF、标定、模型或控制拓扑修改，在停止后显式重建 Runtime，禁止后台 watcher 修改正在运行的控制合同。
