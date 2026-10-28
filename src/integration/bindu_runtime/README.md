# bindu_runtime：ROS 运行装配

[项目模块导航](../../../README.md#模块快速导航) · [编译与运行](#编译与运行) · [职责与边界](#职责与边界) · [配置目录](#配置目录) · [扫雪遥操作](#扫雪遥操作) · [源码导航](#源码导航) · [接入与验证](#接入与验证) · [关联模块](#关联模块)

提供 ROS 2 节点、launch 与配置，把公共契约、执行库、设备和能力组合为可启动的运行实例。算法核心仍在各自模块中维护。

## 按任务查阅

| 要修改或核对的内容 | 相关章节 |
|---|---|
| 场景选择、配置与参数接线 | [配置目录](#配置目录)、[扫雪遥操作](#扫雪遥操作) |
| 初次构建、数值依赖与ABI | [编译与运行](#编译与运行) |
| Snowbot机器人部署与日常操作 | [应用部署指南](https://github.com/WXB1870/snowbot/blob/main/运行与验证.md) |
| 旧位置接口专项 | [旧四节点接口](#snowbot-真机最小遥操作) |
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

安装只需在首次准备或依赖变更时执行；每个新终端仍需加载 ROS、上述依赖路径和对应工作区。不要直接复用其他机器的 `.so` 或本地 `artifacts/vr/env.sh`。已有Conda环境须使用与ROS消息匹配的Python和原生库；新部署按应用指南建立固定环境。

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

- PICO 与浏览器操作台：按 [遥操作模块命令](../../capabilities/bindu_teleoperation/README.md#编译与运行) 加载查看器依赖与 TLS，再显式选择Snowbot应用中的`snowbot_vr_sim.json`；模板归属和日常步骤见[应用指南](https://github.com/WXB1870/snowbot/blob/main/运行与验证.md#仿真与浏览器操作台)。
- 三维建图／定位：按 [LIO 模块命令](../../capabilities/bindu_lio/README.md#编译与运行) 构建可选包并提供点云、IMU及标定配置。
- 真实Snowbot：从[应用部署指南](https://github.com/WXB1870/snowbot/blob/main/运行与验证.md)选择目标环境和配对源码；x86工具的职责见下节，Orin支持范围单独记录。

## x86 真机部署（已有适配底层）

实机说明以118最后部署的兼容版`df2645a`为准，使用原单仓布局。其源码获取、编译、现场配置和日常启动由[Snowbot运行指南](https://github.com/WXB1870/snowbot/blob/main/运行与验证.md)维护；现有安装直接看[日常启动](https://github.com/WXB1870/snowbot/blob/main/运行与验证.md#日常启动)。

当前两仓维护源码中的`x86_real prepare --application-source ...`属于拆仓后的装配入口，与该已部署版的命令不同。文档不将当前main或开发分支自动视为118安装版本。姿态优化列为未实现项，保留在`xavier/posture-optimization`。

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

统一PICO入口为[`tools/run_vr.sh`](../../../tools/run_vr.sh)，选择`snowbot_vr_sim.json`或`snowbot_vr_real.json`。两者使用`position_contract=true`、同一真实位置profile／工具几何和协调器；只在设备IO、初始测量状态及现场标定处区分。状态统一到`teleop/state`，夹爪由同一执行节点提供服务。旧四节点配方保留底层联调用途；具体启动与操作见[统一入口](https://github.com/WXB1870/snowbot/blob/main/运行与验证.md#日常启动)。

真实腰臂通过`position_protocol=session_v3`显式选择目标流，须与servo只读参数`position_reference_mode=target_stream`配对（wire 5／reference 2／普通目标配方version 2，显式刚性预测version 3），消息及使用方成套重建。上层只发送16轴联合目标和源身份／期限，servo非实时后台复用`PreparedSegment`生成；仅首次／恢复接管有一次真实接纳握手；正常更新不等待上一目标接纳，PAUSE立即绕过等待，由执行侧从已采用名义参考制动。适配器已移除旧`session_v2`曲线准备、ACK串行依赖及跨层提前量配置；`ExecutionState.reference`来自有效servo采样的q／dq／ddq及采样时刻，缺失时为空。

刚性双臂模式按`rigid_reference_prediction`显式选择version 3几何预测，独立臂、准备与PAUSE保持version 2；须与包含实际模型的servo成套构建，具体边界见[执行说明](../../control/bindu_execution/README.md#扫雪位置执行)。

目标流启动上限由[真实VR配方](../../../../../snowbot/src/snowbot_bringup/config/deployments/snowbot_vr_real.json)维护，逐轴限制由[真实位置profile](../../../../../snowbot/src/snowbot_bringup/config/robots/snowbot/snowbot_real_position.json)维护，发送时按关节取较小值。servo加载[目标流配方](../../../../../snowbot/tools/deployment/snowbot_vendor/position_session/target_stream.yaml)中的接收上限；现场覆盖以实际选定文件为准，接收端仍独立拒绝超限值。参数值只在配置中维护，已有部署覆盖不自动迁移；实体跟踪与停止待现场验证。

目标轴序为左臂J1～J7、右臂J1～J7、腰J1～J2，rad。动态界逐轴取profile与发送端只读启动参数`trajectory_max_velocity`、`trajectory_max_acceleration`及执行侧急动度上限的较小值；参数默认值见实现，servo独立校验接收帽。配置20 ms执行tick／servo名义1 ms采样保持；submit也可触发执行tick，50 Hz不是发送硬限频，实际频率与抖动须现场记录。普通目标最多300 ms，新PAUSE独立固定2 s停止期限，详见[执行时间语义](../../control/bindu_execution/README.md)；沿途不续期。此新路径只支持`execution_mode=position`，与阻抗组合明确拒绝；既有可选阻抗入口的控制和暂停语义保持，其兼容性不属于本轮验证；含可选阻抗的准备器候选不应用目标流补丁，也不接受`target_stream`配置。真实腰臂配方为`session_v3`、`position`、`command_enabled=false`；servo默认仍`follower`／`unconfirmed`，必须显式成套选择新路径。版本、离线证据与现场未完成项见[Snowbot当前进度](../../../../../snowbot/当前进度.md)。

真实`session_v3`配方的`arguments`可设置这两个速度／加速度参数，统一遥操与独立适配器launch均透传；只接受有限正值，其他协议／仿真配方显式设置时拒绝。servo已有同名只读启动参数，必须允许下发的逐轴有效值，超出接收帽仍拒收；不自动远程修改servo参数。同一会话内动态界不得改变。提高发送帽不会自动提高QP使用的profile速度，profile更低时仍受其限制；调参须成套选择启动值和逐轴profile，尤其不要借全局参数提高腰部等不打算调整的轴。急动度、参考超差保护、TTL和PAUSE期限保持。

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

统一PICO位置配方由协调器管理上述Action；旧四节点专项配方不启动该协调器，仍需显式启动Action。按职责查阅：[按键与相对映射](../../capabilities/bindu_teleoperation/README.md#pico-按键方案) · [启动、装卸及重接步骤](https://github.com/WXB1870/snowbot/blob/main/运行与验证.md#pico操作) · [状态契约](../../../动作执行契约.md#61-snowbot-工具操作与握持切换) · [真实底层启动与重启限制](../../../软件架构设计.md#832-snowbot-实机启动与准备)。

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

- 环境切换：[统一仿真与真实模式选择](../../../运行与验证.md#框架运行与环境选择)。
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

旧四节点最小遥操的历史部署与验收说明保存在本地过程记录，不随Git分发。日常部署使用前文[x86真机入口](#x86-真机部署已有适配底层)；旧接口专项工具的显式环境与命令见下节。

### 部署自动化

[`tools/snowbot_teleop.sh`](../../../tools/snowbot_teleop.sh)只保留旧四节点专项配方和检查；日常PICO使用前文x86入口。旧工具不再单独构建：先完成两仓配套构建，再显式选择该安装环境和另一份专项配方。

```bash
export BINDU_TELEOP_WORKSPACE=/绝对路径/已准备的x86部署目录
bash tools/snowbot_teleop.sh configure --config "$BINDU_TELEOP_WORKSPACE/site-interface.json"
bash tools/snowbot_teleop.sh check --config "$BINDU_TELEOP_WORKSPACE/site-interface.json"
# 已有对应旧接口会话时，只读观察：
bash tools/snowbot_teleop.sh status --config "$BINDU_TELEOP_WORKSPACE/site-interface.json" --seconds 3
```

脚本加载选定工作区的`env.sh`，复用ROS、厂商消息、数值依赖和应用安装；不再要求重复填写发行版、venv和依赖目录。配方与日常PICO的`site.json`分别选择，已有文件拒绝覆盖。没有`env.sh`的显式旧安装仍兼容原环境变量加载方式，历史流程保存在本地过程记录。`build`返回迁移提示，不启动构建或设备。

`--help`无需ROS或应用安装。`configure/check/start/status/gripper`继续支持显式`--config`；没有选择工作区时，`configure`必须指定输出文件，配置资源须已安装或通过`BINDU_PACKAGE_PATH`显式选择。局域网输入使用`--host/--cert-file/--key-file`；底层根话题用`--vendor-namespace /`，其他接收节点身份按实际完整名称指定。TLS、消息／本体／限位、唯一写入者与停止保护继续校验，`start`已包含预检；专项配方默认禁写。状态与夹爪操作边界见[设备适配说明](../../hardware/bindu_hardware/README.md#snowbot真实夹爪位置闭环)。

### 旧四节点配方与操作接口

选择[snowbot_real_teleop.json](../../../../../snowbot/src/snowbot_bringup/config/deployments/snowbot_real_teleop.json)，装配仅有`execution`（`snowbot_adapter`）、`teleop_mode`、`teleop_tool`、`vr_input`四个ROS节点，QP子进程由工具节点管理。位置控制覆盖双臂各J1～J7及两轴腰部；头／第8轴作为实测上下文。不启动记录器、单臂、描述／显示、导航、任务、感知或VLA；ROS必要诊断仍保留。包安装依赖尚未裁剪。

先加载匹配的ROS、底层`snowbot_ctl_msgs`和Bindu工作区，运动学依赖按[环境准备](../../../运行与验证.md#环境准备与构建)配置。此命令启动上层只读装配，不启动底层设备：

```bash
ros2 launch bindu_runtime teleop_only.launch.py \
  config:=$(ros2 pkg prefix --share snowbot_bringup)/config/deployments/snowbot_real_teleop.json
ros2 topic echo /bindu_snowbot/execution/state
ros2 topic echo /bindu_snowbot/teleop/mode_state
```

配方默认`command_enabled=false`、`host=127.0.0.1`且TLS为空，仅供本机检查。现场在配方中填写实际监听地址、证书／私钥**路径**、底层话题前缀和接收节点身份；头显连接要求见[VR入口](../../capabilities/bindu_teleoperation/README.md#vr-遥操作运行入口)，位置接口及健康／看门狗条件见[底层适配](../../hardware/bindu_hardware/README.md#snowbot-位置控制适配层)。写入实验经现场确认后才在部署配方开启`command_enabled`。

启动默认`SAFE / MODE_NOT_SELECTED`，不回位、不准备、不使能电机、不清零。按用户当前要求，现场底层待命／保持及夹爪接口先作为可用条件准备，正式测试前再核实；旧底层源码仍包含启动回位／六维清零；待命改造与带载重启仍待实现和验证。

此旧四节点专项配方使用`pico_snowbot_v1`相对位姿和Squeeze接合，Y停止并结束会话；**不启动PICO协调器**。Trigger、X、A、B及摇杆点击均会停止活动工具会话；实体开合需先结束臂会话、通过独立夹爪租约显式授权，虚握／准备／确认不由此入口提供，第8轴模拟角不能用于真机。释放按键并确认停稳后，可以显式新开会话，旧目标不自动续跑。日常统一PICO配方的按键与能力见[遥操作说明](../../capabilities/bindu_teleoperation/README.md#扫雪输入与握持)。

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

2026-10-14已实现配置级环境选择：`runtime_mode=simulation|real`由启动装配统一传给执行、记录和观察节点，与本体选择及下述业务模式分别表达。公共环境解析位于`bindu_contracts/runtime_mode.py`，具体设备路由仍由运行装配负责；Snowbot可选择运动学／外部仿真或底层接口，真实接口默认只读，未实现的真实遥操作、任务和VLA装配显式拒绝启动。命令、状态、事件和输入的环境一致性与兼容规则集中于[执行契约3.3](../../../动作执行契约.md#33-已实现的仿真与真实环境选择2026-10-14)，配置入口见[运行说明](../../../运行与验证.md#框架运行与环境选择)。模式选择后需核对设备连接和反馈有效性，真机验证状态单独记录。

首期模式：OBSERVE（只读）、TELEOP（示教控制）、AUTO（自主任务）、REPLAY（离线消费，真机执行另经正常合同）。readiness 按当前技能所需能力求交集：单臂实验不等待 ASR/Nav2，传统抓取不等待 VLA，示教必须检查记录能力。模式切换先撤销新输入、完成现有资源收敛、装配依赖，再发布模式可用；模式名更新不是就绪证据。

配置顺序建议：defaults → robot → site → task profile → 显式运行覆盖。启动解析完整依赖并生成 ConfigSnapshot（来源、generation/hash、有效值）。同一任务/重试沿用固定快照；首期不实现任意热更新。URDF、标定、模型或控制拓扑修改，在停止后显式重建 Runtime，禁止后台 watcher 修改正在运行的控制合同。
