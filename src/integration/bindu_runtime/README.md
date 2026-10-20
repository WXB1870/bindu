# bindu_runtime：ROS 运行装配

[项目模块导航](../../../README.md#模块快速导航) · [编译与运行](#编译与运行) · [职责与边界](#职责与边界) · [配置目录](#配置目录) · [扫雪遥操作](#扫雪遥操作) · [源码导航](#源码导航) · [接入与验证](#接入与验证) · [关联模块](#关联模块)

提供 ROS 2 节点、launch 与配置，把公共契约、执行库、设备和能力组合为可启动的运行实例。算法核心仍在各自模块中维护。

## 编译与运行

以下命令均在仓库根目录执行，使用与 ROS 消息 ABI 匹配的系统 Python。示例为 Ubuntu 24.04／Jazzy；Ubuntu 22.04／Humble 将 `jazzy` 换为 `humble`，不要在同一终端混用发行版。基础 ROS、colcon、rosdep 和各包依赖按 [环境准备](../../../运行与验证.md#环境准备与构建) 安装。

### 依赖准备

运行包依赖运动学及原生 QP，即使只启动任务示例，首次构建也要准备 C++17、Python 开发头文件、Eigen3、SIMDe，以及匹配的 Pinocchio／ProxSuite。已有可用数值环境可直接加载；若使用仓库固定版本的 wheel，先在新的终端准备隔离目录：

```bash
source /opt/ros/jazzy/setup.bash
python3 -m pip install --target artifacts/teleop-deps -r tools/requirements-teleop.txt \
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
- 真实 Snowbot：沿用 [现场部署顺序](../../../运行与验证.md#snowbot-真机最小遥操作)，准备厂商消息、现场配方与反馈，默认只读；构建和启动上层不等于底层使能或真机验收。

## 职责与边界

- 负责消息转换、依赖选择、设备／遥操作装配、本体校验与进程生命周期。
- 启动时明确仿真／真实环境，检查指令和反馈模式一致性；环境选择与设备后端、写权限分别配置。
- 真实模式支持 Snowbot 双臂／腰部位置遥操作装配，默认只读；独立夹爪位置闭环默认未标定禁止写入；实体夹持／虚握确认、完整PICO流程、Pi和导航未完成真机接入。
- 配置选择不替代真实模型标定、设备能力核实和现场验证。

## 配置目录

| 位置 | 维护内容 |
|---|---|
| [config/deployments/](config/deployments/) | 用户选择的运行入口，引用下列参数 |
| [config/robots/](config/robots/) | 按Snowbot、G1、二代轮式及旧Huawei模型分组的本体、执行、遥操作和工具参数 |
| [config/navigation/](config/navigation/) | 通用导航场景、站点、LIO及定位参数 |
| [config/vla/](config/vla/) | Pi客户端与回环示例 |
| [config/simulation/](config/simulation/) | 通用模拟执行参数 |
| [tests/config/](../../../tests/config/) | 验证专用夹具，不安装到运行包 |

Snowbot真机最小配方为`snowbot_real_teleop.json`，使用独立的真实本体、位置运行profile和工具几何配置；默认只读且不启动记录器。

正式运行配置共40份，安装时保持相同目录结构。旧`teleop_snowbot_deployment.json`并入统一`deployments/teleop_robot_deployment.json`；schema 1读取能力仍兼容。旧平铺路径已迁移，升级需重建运行包并更新外部脚本中的配置路径。

## 扫雪遥操作

### 场景选择

| 目的 | 当前入口 | 反馈与验证范围 |
|---|---|---|
| 工具位姿输入、模式切换与暂停／恢复 | [teleop_only.launch.py](launch/teleop_only.launch.py)，默认加载 [teleop_robot_deployment.json](config/deployments/teleop_robot_deployment.json) | ROS 运动学模拟；默认 `tool_pose` 输入，现场 VR 待接入 |
| 整段双臂协调、CAD几何与实时回放 | [run_shovel_physics.sh](../../../tools/run_shovel_physics.sh)，使用 `--kinematic ... --gui` | 日常运动学检查；生产QP／执行器生成输入，Isaac 6.2核对关节和法兰并回放，无承载／接触验收 |
| 短时重力、夹爪接触与载荷 | 同一入口，使用 `--closed-loop` | 默认contact，有限力矩J8与CAD接触；私有IPC复用求解／会话／执行核心；旧弹簧观测须显式选proxy，不启动ROS节点 |
| Snowbot 真机最小遥操作 | 同一launch，选择[snowbot_real_teleop.json](config/deployments/snowbot_real_teleop.json) | VR接收、模式、双臂QP、位置适配共4节点；不启动记录、单臂、显示和任务模块，默认只读；[操作边界](../../../运行与验证.md#snowbot-真机最小遥操作) |
| Snowbot 底层位置接口联调 | [snowbot_adapter.launch.py](launch/snowbot_adapter.launch.py) | 独立位置入口，默认真实模式且只读；不包含遥操作节点 |

`runtime.launch.py` 的默认 [runtime_deployment.json](config/deployments/runtime_deployment.json) 选择位置适配入口的仿真模式。运行扫雪工具会话从 `teleop_only.launch.py` 进入；部署时选择完整的`snowbot_vr_real.json`或底层联调配方`snowbot_real_teleop.json`，不能只切换`runtime_mode`。

统一PICO入口为[`tools/run_vr.sh`](../../../tools/run_vr.sh)，选择`snowbot_vr_sim.json`或`snowbot_vr_real.json`。两者使用`position_contract=true`、同一真实位置profile／工具几何和协调器；只在设备IO、初始测量状态及现场标定处区分。状态统一到`teleop/state`，夹爪由同一执行节点提供服务。旧四节点配方保留底层联调用途；具体启动与操作见[统一入口](../../../运行与验证.md#vr-统一部署与启动)。

### 配套配置与流程

真机配方增加`snowbot_grippers_real.json`，默认`calibrated=false`禁止夹爪写入；位置闭环及授权Trigger操作见[真实夹爪接口](../../../运行与验证.md#snowbot真实夹爪位置闭环)。夹爪服务由同一execution节点提供，原四节点范围不变。底层需配套`bindu_gripper_control`保护入口，实体持物／虚握不由位置到位推断。

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
3. 持续提供有效工具输入，显式选择 `bimanual_tool` 模式，再开启 `teleop/tool_session`。暂停、恢复和只读显示见[遥操作专用装配与暂停](../../../运行与验证.md#遥操作专用装配与暂停)；模式 Action 见[Snowbot 双臂遥操作](../../../运行与验证.md#snowbot-双臂与两轴腰部遥操作)。

以上为当前模拟装配步骤。实机目标的底层默认待命与重启条件见[启动方案](../../../软件架构设计.md#832-snowbot-实机启动与准备建议流程)；PICO的Squeeze／Trigger术语、实体键表和相对计算见[遥操作映射](../../capabilities/bindu_teleoperation/README.md#pico-按键方案)，空手装夹／持铲重接／作业卸载见[目标操作流程](../../../运行与验证.md#pico-扫雪操作流程目标方案)，装卸资格、虚握子状态和停止恢复见[执行契约6.1](../../../动作执行契约.md#61-snowbot-工具操作与握持切换)。独立PICO运动学入口已接入：选择[teleop_pico_snowbot_sim.json](config/deployments/teleop_pico_snowbot_sim.json)，`pico_enabled=true`且`tool_input_source=vr`；独立夹爪服务已接入统一位置配方，实体夹持与真实底层恢复仍待验收，旧默认装配不改映射。

当前底层源码仍有启动回位及自动六维传感器清零，`bindu_guard=true`不跳过这两步；必须同步修改底层默认启动路径，不能仅在GUI加按钮。现有看门狗也不允许新适配器直接接管旧会话。整底层进程重启的持载保持须单独验证；完成前持铲仍需支架辅助恢复，不能把目标待命方案视为已有能力。

工具输入与 `tight` 语义见[遥操作模块](../../capabilities/bindu_teleoperation/README.md#扫雪输入与握持)。`tight=false` 只释放右手绕杆轴转动，实体夹爪动作与握持确认仍待接入。

真机 VR 使用接合基准下的相对位姿增量，再生成机器人基座系工具目标。当前 `tool_pose` 接口消费该映射结果，模拟输入也在这一层注入；`vr` 会话已在内部实现相对映射，现场 VR 到统一工具消息的转接仍待接入。接入头显时需落实坐标标定、比例和松握重接基准，详见上面的输入说明。

真机最小配方仅接位姿／Squeeze／Y，须显式管理模式和工具Action；实体操作键停止会话，`tight=false`在真实模式下也会停止。完整边界见[真机最小遥操作](../../../运行与验证.md#snowbot-真机最小遥操作)。

### 物理反馈与柔顺

物理脚本默认做参考回放；`--closed-loop` 接实际关节反馈，另加 `--load-feedback` 才启用左腕虚拟六维载荷的握持变形观测。观测结果进入 QP 工具位姿跟踪，双臂载荷另行分析；目前没有双臂内力上限优化，也没有额外的上层导纳／阻抗控制器。底层已有柔顺，仿真参数尚未与实机标定。

实验采用固定底盘、木质刚体空铲与弹簧阻尼握持代理，没有真实指面滑移、木杆弯曲或雪载荷。完整生成输入、GUI 启动、评分与内力定义见[PhysX 受力实验](../../../运行与验证.md#木质雪铲的-physx-受力实验2026-10-15)。

PICO运行节点：[pico_node.py](bindu_runtime/pico_node.py)；模拟开合参数：[pico_snowbot_sim.json](config/robots/snowbot/pico_snowbot_sim.json)。Squeeze／Trigger及面键先由接收器保留，再由协调器处理操作和控制权，实际夹爪／承载仍不得用模拟状态代替。

## 源码导航

| 入口 | 内容 |
|---|---|
| [setup.py](setup.py) | 全部 ROS 可执行入口与安装资源 |
| [plugins.py](bindu_runtime/plugins.py)、[device_assembly.py](bindu_runtime/device_assembly.py)、[simulation_devices.py](bindu_runtime/simulation_devices.py) | 能力选择、设备与模拟后端装配 |
| [teleop_assembly.py](bindu_runtime/teleop_assembly.py) / [teleop_deployment.py](bindu_runtime/teleop_deployment.py) | 遥操作依赖和部署配置 |
| [robot_setup.py](bindu_runtime/robot_setup.py) / [common.py](bindu_runtime/common.py) | 本体／自由度校验、ROS 转换与公共运行辅助 |
| [execution_node.py](bindu_runtime/execution_node.py) / [snowbot_adapter.py](bindu_runtime/snowbot_adapter.py) | 共享执行节点与底层位置适配 |
| [bindu_runtime/](bindu_runtime/) | 遥操作、VR、Pi、导航、感知、任务、记录等节点源码 |
| [launch/](launch/) / [config/](config/) | 启动装配与配置全集 |

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
- 本体与装配检查：[本体配置](../../../tests/test_robot_configuration.py)、[模式](../../../tests/test_runtime_mode.py)、[遥操作装配](../../../tests/test_teleop_deployment.py)。
- ROS 检查：[示例任务](../../../tests/validate_ros.py)、[环境一致性](../../../tests/validate_runtime_mode.py)、[遥操作](../../../tests/validate_teleop.py)、[底层适配](../../../tests/validate_snowbot_adapter.py)。各脚本是否自行启动装配见其参数；[真机位置链合成验证](../../../tests/validate_real_teleop.py)自带替身控制器，不接硬件。

## 关联模块

[公共契约](../../shared/bindu_contracts/README.md) · [ROS 接口](../../shared/bindu_interfaces/README.md) · [共享执行](../../control/bindu_execution/README.md) · [设备适配](../../hardware/bindu_hardware/README.md) · [遥操作](../../capabilities/bindu_teleoperation/README.md) · [全部模块](../../../README.md#模块快速导航)
