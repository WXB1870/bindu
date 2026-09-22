# Bindu

基于 **ROS 2** 的模块化机器人控制系统，面向长程导航与移动操作。首期目标是让轮式人形根据语言指令寻找饮料、导航抓取并持物返回，后续扩展至双足人形。

通过公共接口连接任务、感知、规划、VLA、遥操作和设备控制，使算法与机器人硬件可以独立替换。VLA 在机器人端仅部署客户端，模型推理在远端运行。

## 功能状态

项目处于模拟骨架与能力接入阶段，完整移动抓取和真机控制尚未完成。

| 模块 | 当前状态 |
|---|---|
| 任务与执行 | 已实现模拟任务流程、控制权、在线关节目标、取消与异常处理 |
| Pi 客户端 | 已实现两种历史 ZMQ 模式、动作映射、时效检查和诊断；已通过模拟服务测试 |
| 设备适配 | 已实现模拟底盘、关节和灵巧手驱动；真实设备待接入 |
| 导航 | 已接Nav2仿真；历史FAST-LIO与3D ICP已迁移至ROS 2并通过合成数据验证，Isaac建图/到站/断流恢复分批通过，正常往返受反馈间断阻塞；真实3D传感器待接入 |
| 感知、规划 | 已有模拟实现，真实算法待接入 |
| VR 遥操作与 IK | 已接 v3.4 手柄输入、相对控制和连续 IK；单臂模拟末端/状态可视反馈已通过 ROS/Vuer 验证 |
| 语音交互 | 已预留模块目录 |
| 数据记录 | 已实现异步事件与状态记录；图像同步及完整训练数据管线待实现 |
| 共享执行层插值 | C++ 统一数值内核已接入；在线流、定时多轴轨迹与动作块已通过 Ubuntu ROS/Pi/VR 模拟回归 |

## 工程入口

仓库根目录同时是 ROS 2 工作区，共包含 15 个可构建包。

```text
bindu/
├── src/
│   ├── shared/          # 公共契约、ROS 消息与服务
│   ├── tasks/           # 任务流程与场景状态
│   ├── control/         # 控制权、时序与轨迹执行
│   ├── hardware/        # 设备驱动、反馈汇总与公共本体描述
│   ├── data/            # 异步数据记录
│   ├── integration/     # ROS 节点、配置与 launch
│   └── capabilities/    # IK、VLA、感知、导航、规划、交互、遥操作
├── tests/               # 模块测试与跨进程模拟验证
├── tools/               # 开发与部署工具预留入口
├── README.md
├── AGENTS.md
├── 软件架构设计.md
├── 动作执行契约.md
└── LICENSE
```

`build/`、`install/`、`log/` 由构建生成；`artifacts/` 保存实验结果与临时依赖，均不提交 Git。`tools/requirements-teleop.txt` 保存可选 VR/IK 验证依赖。历史代码、图片、研究计划与过程记录在主开发工作区单独维护。

## 环境要求

- 已验证开发环境：Ubuntu 24.04、ROS 2 Jazzy、Python 3.12、x86_64。
- 构建工具：colcon、rosdep；运行依赖包括 NumPy 和 PyZMQ。
- Jetson Orin / Ubuntu 22.04 的部署兼容性尚未验证。

下列命令假定 ROS 2 Jazzy、colcon 和已初始化的 rosdep 可用。新终端只加载当前使用的工作区，避免混用旧安装环境。

### Conda 兼容性

2026年9月18日，在 Ubuntu 24.04 / x86_64 上验证了 **Conda Python 3.12.14 + 系统 ROS 2 Jazzy**：13包构建、71项模块、16个 ROS、14个 Pi 和9个 VR 场景通过。首轮一个场景在故障注入前因反馈过期结束，原用例连续3次及全组复测通过；不等同长期稳定性或 Orin 验收。系统 ROS、消息库及构建辅助包仍由 apt 提供，这不是独立的全 Conda ROS 发行版。

- 使用单独 Conda 环境，测试采用 setuptools 68.1.2；IK/VR 固定依赖见 `tools/requirements-teleop.txt`。在环境内追加 `/usr/lib/python3/dist-packages` 供 ROS 构建辅助包使用，禁用用户级 site-packages，避免混入个人环境。
- 先加载系统 ROS，再按[VR 入口](#vr-遥操作运行入口)设置 Pinocchio 的 Python 与动态库搜索顺序，依赖根改为 Conda 的 `lib/python3.12/site-packages`。仅执行 `conda activate` 不够：误加载系统旧 Pinocchio 与环境 NumPy 2 的组合曾导致段错误。
- 用 Conda 的 `python /usr/bin/colcon build` 构建，并显式设置 CMake 的 `Python3_EXECUTABLE` / `PYTHON_EXECUTABLE` 为该环境解释器；使用独立源码、build/install，重新编译原生扩展。核对生成的节点入口和实际进程解释器，不能仅检查 shell 是否激活。系统 `ros2` 启动器仍可使用系统 Python。

## 快速开始

### 1. 获取与构建

```bash
git clone https://github.com/WXB1870/bindu.git
cd bindu
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
colcon build --base-paths src --symlink-install
source install/setup.bash
```

### 2. 启动模拟系统

在工作区终端执行：

```bash
export ROS_DOMAIN_ID=116 ROS_LOCALHOST_ONLY=1
ros2 launch bindu_runtime skeleton.launch.py
```

另开一个终端，进入同一工作区并提交模拟取物任务：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=116 ROS_LOCALHOST_ONLY=1
ros2 action send_goal /bindu_sim/tasks/fetch_drink bindu_interfaces/action/FetchDrink \
  "{task_id: demo_1, object_id: drink, strategy: planner}" --feedback
```

`strategy` 支持 `planner` 和 `chunk` 两种模拟策略。一次成功任务后，模拟场景保留持物状态；新的独立实验需重启模拟系统。以上流程不连接真实机器人。

## 测试

在已加载 ROS 与工作区环境的终端执行；测试输出目录应为新的实验批次。

```bash
python3 -m unittest discover -s tests -p 'test_*.py' -v
python3 tests/validate_ros.py --output artifacts/ros-check
python3 tests/validate_pi.py --output artifacts/pi-check
```

集成验证会启动并收尾各自的模拟进程。Pi 验证自带回环服务、合成图像和关节观测，无需模型权重或真实设备。

此前 Python 后端已验证：13 包构建、64 项模块测试（无跳过）、16 个 ROS 集成场景、14 个 Pi 场景及 9 个遥操作场景全部通过；包含动作块时序/版本、受控停止，以及真实 Vuer 服务接收合成输入。尚未做头显现场联调。模拟测试不等于真机或实时性能验收。Pi 首版使用 NumPy 1.26.4；本轮 VR/IK 与回归使用 NumPy 2.3.5，PyZMQ 均为 26.4.0；若需要隔离安装该 PyZMQ 版本，可执行：

```bash
python3 -m pip install --target artifacts/pi-deps pyzmq==26.4.0
export PYTHONPATH="$PWD/artifacts/pi-deps:$PYTHONPATH"
```

## 导航适配验证入口

首批导航适配保留任务的 `navigate(port, context, site)` 接口；选择 `bindu_runtime.navigation:Nav2Navigation` 时，通过 `nav2_msgs/action/NavigateToPose` 调用后端，目标速度经已有租约、revision和执行器到模拟底盘。默认启动仍使用原 `SimNavigation`。

在按快速开始完成依赖安装/构建的 Ubuntu ROS 环境中运行：

```bash
python3 tests/validate_navigation.py --output artifacts/navigation-check
```

该验证器自行启动、收尾隔离命名空间的任务/执行/记录节点和 `tests/navigation_peer.py` 协议替身，覆盖往返、取消、断流、旧目标消息、来源变化、定位失效、错误成功结果和停稳检查。它使用真实 ROS Action 通信，但没有路径规划、障碍物、SLAM或物理底盘模型，不能作为真实 Nav2 导航验收。

2026-09-18 在 Ubuntu/Jazzy、bindu Conda Python 3.12 环境完成13包构建，97项模块及16项导航场景通过；原有16 ROS、14 Pi、11 VR场景回归通过。nav2_msgs使用1.3.13。首轮2项因测试源停止更新而报定位过期；测试源改串行回调后完整通过，超时阈值未放宽。模拟结果不代表现场稳定性。

随后按差速底盘要求校验：修复模拟器将机体前进量直接累加为全局x的问题，改为平面x/y/yaw积分并补齐二维反馈。新增圆弧/倒车、原地转向后前进、二维往返和错误朝向成功检查。此处是无打滑的理想运动学，仍未模拟轮地接触或真实定位；`ExecutionState`新增`base_y`后须重建所有依赖它的ROS包。临时G1模型基准及实机差异见[架构实现边界](软件架构设计.md#854-当前代码组织与替换边界)。

差速校验在独立Ubuntu/Jazzy工作区完成13包重建、100项模块、18项导航和16项通用ROS场景，全部通过；Pi/VR端到端场景沿用上一批证据，本批未重跑。

- 站点与门控参数见 [`navigation_sim.json`](src/integration/bindu_runtime/config/navigation_sim.json)，只包含测试地图版本和测试站点；重复站点ID、非法位姿/参数拒绝加载，不能直接换成未经坐标核对的历史航点。
- `navigation_backend` 指定后端命名空间，接口为 `navigate_to_pose`、`velocity`、`pose`。速度使用 [`NavigationVelocity`](src/shared/bindu_interfaces/msg/NavigationVelocity.msg)，必须在命令产生时附实际Action目标UUID、来源进程实例、单调序号、源时间及机体frame。定位使用 [`NavigationPose`](src/shared/bindu_interfaces/msg/NavigationPose.msg)，保留测量时间、地图版本与全局frame；后续定位/TF适配不得用接收时间刷新旧数据。
- **未修改的 Nav2 `/cmd_vel` 不能直接接入本批入口。** 它缺少目标身份，不能由普通转发节点在接收时补当前目标ID。下文的独立物理导航入口通过每目标独立进程、固定发布者GID和一次性UUID绑定完成该桥接；本节协议替身测试仍不包含真实规划。
- 到点需要后端Action成功、全局位姿误差达标、底盘反馈持续停稳；取消先关速度入口，再停止执行并取消后端。停稳或取消无法确认时明确失败。输入使用best-effort有界队列，Action/执行服务保留可靠通信；丢失输入触发短期有效期和门控超时。
- 底盘独立速度/加速度参数及停止语义见[执行契约](动作执行契约.md#2-输入模式必须显式选择)。真机底盘/雷达尚未接入；Isaac中的真实导航、建图和定位见下文独立入口。动作块适配和手部扩展继续暂缓。

## 临时 G1 全身模拟入口

采用固定版本的 Galbot One Golf 派生[运动学模型](src/hardware/bindu_description/urdf/g1_provisional.urdf)，[执行配置](src/integration/bindu_runtime/config/g1_provisional_sim.json)覆盖19个关节轴，底盘另用机体线速度`v`和角速度`ω`控制：

| 控制组 | 关节顺序 | 数量 |
|---|---|---|
| `leg` | `leg_joint1`、`leg_joint2`，连杆升降 | 2 |
| `waist` | `leg_joint3`，腰部俯仰 | 1 |
| `head` | `head_joint1`、`head_joint2` | 2 |
| `left_arm` | `left_arm_joint1` → `left_arm_joint7` | 7 |
| `right_arm` | `right_arm_joint1` → `right_arm_joint7` | 7 |

角度单位为rad。位置限位逐轴取自原URDF，保留左右臂不对称范围；速度上限暂取原值与0.5 rad/s的较小值，加速度/jerk的2 rad/s²、12 rad/s³为模拟执行配置，不能作实机参数。额外腿腰轴`leg_joint4/5`固定在零位；手爪与轮子子树省略，底盘由差速模拟驱动负责，手臂保留法兰安装坐标。模型只保留运动学/惯性，不含视觉网格或碰撞几何；三轴腿腰映射是临时近似，尚未做实机标定。

```bash
ros2 launch bindu_runtime g1_sim.launch.py
python3 tests/validate_g1.py --output artifacts/g1-check
```

第一条命令启动执行器、记录器、实测状态桥、`robot_state_publisher`和到站导航Action；第二条自行启动独立命名空间做验证。模型通过`robot_description`加载，19轴反馈发布为`joint_states`并驱动TF；`odom→base_link`使用差速模拟反馈，保留源时间，失联不会刷新旧姿态。TF话题隔离在各自命名空间，当前不发布`map→odom`。共享执行器按控制组串行执行，尚无双臂协同或全身IK控制器；本入口不启动依赖手部的取物任务。

G1 能力统一从同一入口选择（需要重建工作区以生成新增的 `NavigateToSite` Action）：

```bash
# 左臂 VR/IK 与显示；右臂改为 side:=right。头显连接另需 host/TLS 配置。
ros2 launch bindu_runtime g1_sim.launch.py vr_enabled:=true side:=left
# 已有 VRInput 源时，仅启用 IK 与显示。
ros2 launch bindu_runtime g1_sim.launch.py teleop_enabled:=true side:=left
# Pi 客户端；另需配置服务端点并提供三路 RGB 与全身关节观测。
ros2 launch bindu_runtime g1_sim.launch.py pi_enabled:=true
```

以上是不同启动方式，不应在同一命名空间重复启动。VR 左右臂分别使用 `teleop_g1_left/right.json`；会话资源为 `left_arm` 或 `right_arm`，一次控制一臂。IK 和显示使用当前腿腰、头部及另一臂反馈，优化变量只有所选臂的7轴；非活动关节缺失/非法或求解期间姿态变化时拒绝下发。末端暂用法兰坐标，不包含手爪TCP标定。Pi 的 `pi_g1.json` 显式映射19轴观测，要求动作关联观测与会话，允许同样的两个单臂资源；不表示远端模型已支持该本体，默认端点仍为本地测试地址。

到站导航无需手部/感知服务，默认开启，可用 `navigation_enabled:=false` 关闭。接入满足前述身份与时间契约的导航后端后，可提交：

```bash
ros2 action send_goal /bindu_g1_sim/navigation/navigate_to_site bindu_interfaces/action/NavigateToSite \
  "{task_id: g1_nav_1, site: pickup}" --feedback
```

Action 仅申请底盘控制权；后端成功、定位误差与停稳条件仍必须同时满足。启动入口不包含导航后端、地图、定位或SLAM，无后端会明确失败。`navigation_sim.json` 中的站点只用于协议模拟。

G1 集成回归入口（均自行启动隔离模拟进程）：

```bash
python3 tests/validate_teleop.py --g1 --output artifacts/g1-vr-left --cases normal body_pose clutch cancel
python3 tests/validate_teleop.py --g1 --side right --output artifacts/g1-vr-right --cases normal body_pose
python3 tests/validate_pi.py --g1 --output artifacts/g1-pi
python3 tests/validate_navigation.py --g1 --output artifacts/g1-navigation
```

2026-09-19 本机 macOS/ARM64、Python 3.12 完成106项模块测试，无跳过；当时因开发机SSH连接超时，新增ROS场景未运行。2026-09-22 在 Ubuntu 24.04 / Jazzy / bindu Conda Python 3.12.14 上完成14包独立重建（含新增Action），106项模块无跳过、9项G1全身、左/右臂VR 11/3项、Pi左臂14项及右臂2种通信模式、18项到站导航全部通过；旧入口16 ROS、14 Pi、18导航及11 VR回归通过。VR包含真实Vuer服务接收合成输入，导航仍使用协议替身。

本批首轮构建因Conda旧`argparse`覆盖标准库失败，导航首轮因未加载隔离`nav2_msgs 1.3.13`失败；调整环境路径后通过，受影响的G1全身及左右VR/Pi完整启动另行复测通过，未修改运行源码或放宽阈值。Vuer连接断开及部分节点SIGINT收尾仍有异常日志；功能断言通过不代表退出过程完全正常。本批模拟进程已收尾，未运行现场头显、真实Pi模型、真实Nav2/SLAM或真机。

模型来源与修改范围见[架构](软件架构设计.md#854-当前代码组织与替换边界)，Apache-2.0许可随描述包分发。需要重生成时，将固定提交的原URDF传给`python3 tools/derive_g1_model.py /path/to/galbot_one_golf.urdf`；工具校验源码SHA256并统一生成模型与profile，避免限位表漂移。

此前9月18日Ubuntu/Jazzy验证：14包构建、103项模块、9项全身场景及5项相关导航回归通过。TF与FK按同一时间戳的实测样本对照；初轮用较早TF与最终目标比较而失败，修正验证器后通过，精度阈值未放宽。

### Isaac Sim 图形物理仿真

`g1_sim.launch.py`默认仍为运动学模拟。可选`device_backend:=external_simulation`通过独立ROS设备桥连接Isaac Sim：PhysX负责重力、轮地接触和驱动响应，执行器读取实际关节位置/速度及底盘位姿，不把下发目标当反馈。当前仅在本机Isaac Sim **6.0.0-rc.22**、Ubuntu24.04/Jazzy、RTX4090测试；脚本使用该RC版本的导入API，其他版本须另行验证。

先按上文重建工作区，生成新增`SimulationCommand/SimulationFeedback`接口，然后准备固定版本资产：

```bash
python3 tools/prepare_g1_physics.py
# 可离线复用该固定提交的上游目录：追加 --source /path/to/galbot_one_golf_description
export BINDU_INSTALL="$PWD/install"
export ISAAC_SIM_PATH="$HOME/isaacsim/_build/linux-x86_64/release"
tools/run_g1_isaac.sh
```

默认打开完整G1图形界面，追加`--headless`才使用无头模式，`--no-ros`仅展示静止本体。启动器隔离Conda/Python动态库并限定ROS localhost，默认域125、命名空间`/bindu_g1_physics`。首次初始化可能较慢，等待终端`BINDU_G1_PHYSICS_READY`后，在另一个已加载ROS/Bindu工作区的终端运行：

```bash
export ROS_DOMAIN_ID=125 ROS_LOCALHOST_ONLY=1
python3 tests/validate_physics.py --output artifacts/g1-physics-check
# 另一次独立实验：先重启Isaac，再验证逆解与动态指令流
python3 tests/validate_physics.py --suite dynamics --output artifacts/g1-physics-dynamics
# 各批次前均须重启Isaac；下面是独立的验证批次
python3 tests/validate_physics.py --suite boundaries --output artifacts/g1-physics-boundaries
python3 tests/validate_physics.py --suite faults --output artifacts/g1-physics-faults
python3 tests/validate_physics.py --suite soak --soak-seconds 600 \
  --soak-amplitude .45 --soak-period 12 --output artifacts/g1-physics-soak
# 真实Vuer WebSocket接收合成手柄输入；右臂改为 --side right，使用新批次目录
python3 tests/validate_physics.py --suite vr --side left --output artifacts/g1-physics-vr-left
```

验证器自行启动和收尾同命名空间的G1执行/记录节点，检测19轴实测反馈、五组关节跟踪、取消后实际停稳、直行/圆弧及执行进程失联保护；图形仿真继续保留。 `--suite dynamics`复用Pinocchio/CasADi独立IK工作进程，验证左右臂末端目标到达、不可达目标不下发、20Hz连续笛卡尔目标、100Hz关节目标换向、取消/TTL断流保持及旧租约/越限/错序拒绝。IK种子和非活动关节均来自PhysX反馈；末端误差为实测关节FK在`base_link`中的推算，不是独立视觉或PhysX连杆位姿测量。该批使用合成目标，不含现场头显、Vuer接收或Pi服务；原始目标、执行参考和物理反馈分别记录在输出目录。每个仿真实例只接受首个执行器身份，重新运行验证器或替换执行器前须重启图形仿真。若手动运行能力入口，不要同时运行验证器：

```bash
ros2 launch bindu_runtime g1_sim.launch.py namespace:=/bindu_g1_physics \
  profile:="$PWD/artifacts/g1-physics-model/g1_physics_sim.json" \
  device_backend:=external_simulation navigation_enabled:=false
```

`boundaries`增加双臂近奇异小步、突变拒绝、工作空间外扩扫描，以及独立PhysX连杆位姿与关节FK的同源时间对照。`simulation/link_poses`为world帧的PoseArray，固定顺序为base_link、左法兰、右法兰；由物理张量直接测量。`faults`通过隔离ROS中继注入反馈丢失、旧反馈重复、命令丢失/延迟及实例身份变化，检查看门狗保持、旧租约拒绝和故障锁定；身份变化为消息注入，不等于真实进程重启。`soak`默认100Hz运行10分钟，肩部±0.45rad、肘部±0.27rad、周期12秒；逐帧输入、参考和反馈以压缩JSONL保存，另留接纳计数、时间对齐跟踪误差与执行器内存趋势，失败批次也保留。

`vr`走真实Vuer WebSocket → CONTROLLER_MOVE解码 → ROS VRInput → 独立IK工作进程 → 租约执行器 → PhysX反馈链路。合成双手柄以名义72Hz发送OpenXR列主序矩阵；活动手柄做20秒三维弧线（x/y/z范围约80/80/120mm）及±0.08rad转腕，加入人为设定的0.15mm位置噪声、±2ms间隔抖动和周期25ms发送延迟，并渐变握持/扳机值。测试松手后手柄迁移214mm/转腕0.5rad再握持、取消、缺失手柄位姿和输入静默，检查实际停稳、重接无跳变及恢复输入不自动续跑。准备姿态单独用有限轨迹设置；开始运动前等待Vuer/IK/观察节点就绪及连续5秒新鲜物理采样。扳机只记录，不控制手部。原始WebSocket包、解码/IK事件/接受命令/执行状态、PhysX反馈及观察曲线均保留；这些输入分布不是头显实测数据。本批左右臂各5项检查通过：原始手柄帧4796/4862，接纳目标3162/3175；首个完整弧线的观察跟踪RMS为3.58/3.84mm、最大7.58/8.64mm（10Hz最新目标对实测关节FK，含时间滞后）。214mm手柄迁移后重新握持的首目标关节差为0，松手后保持漂移约3.4e-5rad；追踪无效和断流均停稳、恢复输入不自动续跑。修复了等待物理停稳后继续检查旧VR帧而误报输入超时的问题；117项模块及旧G1离合/断流/无效追踪3项回归通过。深弯肘准备姿态未满足速度停稳、启动期约330ms反馈发布阻塞及Vuer断连收尾异常的失败记录均保留；改用已验证姿态并等待启动采样稳定，不代表这些问题已解决。

2026-09-22扩展验证：双臂IK边界与独立连杆位姿8项通过，438个同时间样本最大位置/姿态差8.40e-7m/8.96e-7rad；当前IK位置容差15mm，外扩10mm仍可接纳，30/80mm拒绝。大幅度100Hz流连续600秒、60,000/60,000接纳，实测肩部总摆幅0.9003rad、肘部0.5400rad，跟踪RMS0.00195rad、最大0.01021rad，执行器预热后RSS增长0.004MiB。早期长流约66秒失败，诊断发现第2代GC占用235.67ms并阻塞物理推进；就绪前回收/冻结初始化对象后通过完整10分钟，运行期新对象仍正常GC、退出解冻，原超时阈值不变。此结果不能替代30分钟及更长稳定性验证。故障组6项、117项模块及旧ROS取消/反馈丢失/动作块3项通过；同时修复历史停止失败记录污染恢复后新取消/释放服务响应的问题，受理停止仍不等于实测停稳。

模型生成在忽略入库的`artifacts/g1-physics-model`：恢复同一上游版本的本体视觉/碰撞资产，并保留Apache-2.0许可与来源指纹。该Isaac RC的GLB导入曾只生成空节点，工具改用匹配的上游USD视觉网格；同时按URDF重建关节并校验两侧关节坐标。两只驱动轮及低摩擦支撑为合成几何，尺寸、惯性、增益、初始姿态和120Hz物理步长集中在[physics.json](src/hardware/bindu_description/physics.json)，尚未标定。无质量固定坐标框架使用微小数值质量；自碰撞关闭，该基础物理批次未验证手部、抓取接触、相机图像；导航和建图另见下文专项。完整外观和物理设备闭环不等于真机模型验收。 本机RC完整扩展卸载曾崩溃，启动器采用Isaac默认快速退出，先完成Bindu日志和ROS清理；异常路径保留非零返回码。物理步长是积分配置，不代表已达到实时频率。

本批物理验证：117项模块与10项PhysX检查通过；五组关节跟踪最大目标误差0.00050rad，直行0.0837m、圆弧0.0696m/0.1215rad，执行器冻结后实测线速度0.000142m/s。相关运动学回归G1全身9、VR4、Pi14通过；旧ROS首轮15/16、导航17/18，其启动拒绝场景单独复测各通过，启动偶发性尚未解决。以上不代表VR/Pi/导航已经完成物理环境验收。

2026-09-22追加逆解/动态流物理批次：10项检查通过。左右臂末端位移约46mm，到达位置误差0.486/0.512mm；20Hz连续逆解100/100接纳，末端跟踪RMS3.38mm，求解耗时p95为3.08ms；100Hz关节流600/600接纳，实发均频99.99Hz、间隔p95为10.97ms，实测相对参考的关节误差RMS0.00157rad。取消/断流后0.5s保持漂移均小于0.00010rad；不可达、旧租约、越限与错序拒绝通过。这是短时合成目标实验，不代表长期实时性或现场VR体验。

### 真实 Nav2 与 Isaac 房间导航

`navigation_physics.launch.py`独立装配现有任务、租约执行与记录节点；真实Nav2 1.3.13使用NavFn、Regulated Pure Pursuit与路径失效后重规划的行为树。它不导入G1模型或关节名。本体参数集中在[`navigation_g1_fixture.json`](src/integration/bindu_runtime/config/navigation_g1_fixture.json)，房间和站点分别来自`navigation_room.json`、`navigation_room_sites.json`。替换机器人需提供匹配的URDF/物理设备适配、执行profile、关节姿态、碰撞轮廓、坐标系、雷达外参与速度限制；不能只换网格后沿用旧轮廓和标定。

导航前临时G1以站立、双臂收拢姿态初始化，并检查姿态位置/速度反馈；移动期间持续检查姿态。这里尚未验证从任意姿态起身收臂的动作，也不包含手指驱动或自碰撞检查。房间为可复现的USD几何墙体、桌子与障碍物，机器人复用固定版本开源资产，无需下载额外场景。360线水平雷达直接查询PhysX碰撞。本节默认基线使用理想物理里程计、预制地图和恒等`map→odom`；下节的轮式里程计模式关闭这两个预制输出，接入实际建图/定位算法。

在Ubuntu/Jazzy已安装`ros-jazzy-navigation2`和`ros-jazzy-nav2-regulated-pure-pursuit-controller`、完成工作区重建及物理资产准备后：

```bash
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
cmake -S src/capabilities/bindu_navigation/native -B build/nav2-identity
cmake --build build/nav2-identity -j4
export ROS_DOMAIN_ID=126 ROS_LOCALHOST_ONLY=1
export RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS
# 终端1：GUI物理场景；开始独立实验前重启Isaac，避免复用旧执行器身份。
tools/run_g1_isaac.sh --namespace /bindu_navigation \
  --navigation-scene src/integration/bindu_runtime/config/navigation_room.json \
  --navigation-robot src/integration/bindu_runtime/config/navigation_g1_fixture.json \
  --output artifacts/navigation-room-scene
# 终端2：加载相同ROS环境与域，等待物理场景READY后启动。
ros2 launch bindu_runtime navigation_physics.launch.py namespace:=/bindu_navigation \
  profile:="$PWD/artifacts/g1-physics-model/g1_physics_sim.json" \
  navigation_robot:="$PWD/src/integration/bindu_runtime/config/navigation_g1_fixture.json" \
  navigation_config:="$PWD/src/integration/bindu_runtime/config/navigation_room_sites.json" \
  identity_bridge:="$PWD/build/nav2-identity/nav2_velocity_identity" \
  output:="$PWD/artifacts/navigation-room-run" run_id:=navigation_room
# 终端3：同一ROS环境与域。测试结果目录必须是新的。
python3 tests/validate_nav2_physics.py --output artifacts/navigation-room-tests \
  --sessions artifacts/navigation-room-run
```

每个目标消耗独立预热的Nav2进程和命名空间，C++入口逐包校验RMW发布者GID、源时间戳，一次性绑定该次实际Action UUID，退休后不复用进程。未就绪时拒绝目标，准备期间不运动；本版以启动开销换取旧目标隔离。速度仍经公共执行契约，不直接驱动模拟轮子。原生来源/重放/不可重绑检查可用`tests/validate_nav2_identity.py --binary build/nav2-identity/nav2_velocity_identity --output artifacts/nav2-identity-check`单独运行。

2026-09-22分批通过正常往返、取消、动态障碍强制改道、道路堵塞、定位断流、控制器进程退出六类场景；最终改道路径由上侧y≈1.19m切到下侧y≈−1.23m，实际到桌边/返回误差58.0/31.2mm。119项模块、3项原生身份检查与6项旧导航回归通过。结果索引见该批`verification.json`，最终专项为`route-tests/results.json`，轨迹为`trajectory.png`。这是分批证据，不是所有场景一次连续全绿；早期失败均保留。

修复了任务停稳判据未读取profile容差、PhysX休眠保留旧底盘速度、生命周期查询丢响应后无限等待的问题；休眠反馈依据PhysX原生状态置零，原始速度保留，不放宽反馈期限。另行复测旧基础物理入口时，GUI/无头均在低位腿关节目标处因速度未停稳超时，**本轮基础物理10项未全部通过，原因未定位**。站立导航成功不能覆盖该姿态下的失败；自碰撞、任意姿态起身、真实传感器/定位仍未验收。

本机依赖隔离解包在`artifacts/nav2-physics-2026-09-22/deps`，未修改系统Nav2安装；机器专用环境见该批`env.sh`，不能直接复制到新机器。原始扫描、物理反馈、动作与命令、路径、会话配置及失败日志均保留在该批目录；数据忽略入库，尚未远端备份。

### Isaac 内的实际建图与地图定位

[`localization.launch.py`](src/integration/bindu_runtime/launch/localization.launch.py)分别装配 **SLAM Toolbox 2.8.5**（在线异步二维建图）和 **Nav2 AMCL 1.3.13**（重载栅格地图后的粒子滤波定位）。建图阶段由SLAM发布地图和`map→odom`；定位阶段由map_server发布已保存地图、AMCL发布`map→odom`。两种模式应互斥运行。算法参数在[`navigation_localization.yaml`](src/integration/bindu_runtime/config/navigation_localization.yaml)，不包含G1关节或模型路径。

Isaac的[`navigation_sensors.json`](src/integration/bindu_runtime/config/navigation_sensors.json)启用实测轮子转角积分，带人为设置的2%轮径、1%轮距误差和5mm标准差的射线距离噪声；这些是测试扰动，尚未按实机标定。此模式不发布预制地图或恒等`map→odom`。物理世界位姿只用于生成传感器射线、独立评估和执行层运动反馈，不提供给SLAM/AMCL作定位输入。轮转角每次采样必须变化小于π；当前实现处理±π回绕。沿用执行层墙上时钟，算法`use_sim_time=false`，不混用`/clock`。

在上一节环境基础上添加`ros-jazzy-slam-toolbox`、`ros-jazzy-nav2-amcl`和`ros-jazzy-nav2-map-server`，重建`bindu_hardware`与`bindu_runtime`。三个终端加载相同ROS、工作区和域环境，依次执行：

```bash
# 终端1：全新GUI场景，启用轮式里程计和噪声。
tools/run_g1_isaac.sh --namespace /bindu_navigation \
  --navigation-scene src/integration/bindu_runtime/config/navigation_room.json \
  --navigation-robot src/integration/bindu_runtime/config/navigation_g1_fixture.json \
  --navigation-sensors src/integration/bindu_runtime/config/navigation_sensors.json \
  --output artifacts/slam-scene
# 终端2：物理场景READY后，启用扫描新鲜度门控。
ros2 launch bindu_runtime navigation_physics.launch.py namespace:=/bindu_navigation \
  profile:="$PWD/artifacts/g1-physics-model/g1_physics_sim.json" \
  navigation_robot:="$PWD/src/integration/bindu_runtime/config/navigation_g1_fixture.json" \
  navigation_config:="$PWD/src/integration/bindu_runtime/config/navigation_mapping_sites.json" \
  identity_bridge:="$PWD/build/nav2-identity/nav2_velocity_identity" \
  output:="$PWD/artifacts/slam-run" run_id:=slam_room require_scan:=true
# 终端3：自行启动SLAM，采集扫描/保存地图，再切换AMCL和执行导航检查。
python3 tests/validate_localization_physics.py \
  --robot src/integration/bindu_runtime/config/navigation_g1_fixture.json \
  --sites src/integration/bindu_runtime/config/navigation_mapping_sites.json \
  --output artifacts/slam-tests
```

验证器通过公共执行契约发送受控合成遥操作路线，不是自主探索；用SLAM估计位姿引导绕房间一周，保存`room.yaml/room.pgm`与`room_graph.posegraph/.data`。随后从人为偏置的初始位姿启动AMCL，检查往返、前进时雷达断流停车及恢复。`require_scan:=true`防止扫描消失后仅凭轮式TF继续认为定位有效；它停止更新公共定位消息，再由既有定位超时和停止流程收尾，不能把单项超时配置当作总停车延迟。断流可能先触发Nav2的TF_ERROR（102），或先触发公共层NAV_POSE_STALE；验证器只接纳这两种定位失败，另检查断流后的位移与物理停稳。保存地图显式等待最多10秒，算法退出先执行生命周期关闭。

如需单独运行算法，使用`ros2 launch bindu_runtime localization.launch.py namespace:=/bindu_navigation navigation_robot:=绝对配置路径 mode:=mapping`；地图定位改为`mode:=localization map:=已保存地图的绝对YAML路径`并提供初始位姿。建图可用`pose_graph:=已保存图的绝对前缀`恢复；当前配置假定机器人位于原始建图起点，尚未验证任意位置恢复或绑架后全局重定位。不要与完整验证器同时启动第二套算法。

2026-09-22分阶段验证：五个建图航点、位姿图恢复/栅格地图保存、AMCL重载、正常往返及前进断流停车通过。建图定位RMS约31.8mm，AMCL正常往返定位RMS约36.7mm（与独立物理真值同原点、50ms内最近采样比较，未作刚体拟合）；桌边/返回实际到站误差100.6/51.4mm。断流注入时线速度0.062m/s，额外移动75.8mm，约0.622秒后满足150ms窗口的停稳判据。**断流恢复后的近距离回站失败，返回NAV_ACTION_FAILED:105并停稳；完整验证不是全绿。** 123项模块和6项旧导航回归通过。结果、失败批次及数据指纹索引为该批`verification.json`，地图和曲线为`restored-verified-tests/room.yaml`、`mapping_localization.png`。

本机依赖在`artifacts/slam-physics-2026-09-22/deps`隔离解包，环境入口为该批`env.sh`；未修改系统安装，不适用于新机器直接复制。地图、压缩原始扫描/位姿、轨迹、成功与失败批次保存在同目录。传感器为固定高度水平二维射线，不能替代3D雷达/IMU、真实传感器标定或完整碰撞验收；AMCL与Nav2到站成功也不代表精细抓取对位达标。旧低位腿关节停稳回归失败仍待排查。本机曾出现DDS生命周期服务响应超时，导致自动启动失败且需重启算法；失败日志保留，未宣称启动稳定性已解决。验证器等待AMCL处理首帧扫描后才采集收敛结果。

## 历史三维导航迁移入口

2026-09-22按用户要求，以本地`导航-源码.zip`中的FAST-LIO和FAST_LIO_LOCALIZATION为后续主路线。原二维SLAM Toolbox/AMCL保留为已验证基线。算法进程位于[`bindu_lio`](src/capabilities/bindu_lio/)，与站点业务、Nav2和共享执行层隔离；原始ZIP不改。保留迭代滤波、IMU去畸变、局部地图及ikd-Tree和两级点到点ICP，来源指纹／适配范围见[`source_manifest.json`](src/capabilities/bindu_lio/source_manifest.json)，该包按随附GPL-2.0许可维护，原文件声明保留。公共执行模块不导入此算法包。

- `fast_lio`：带时间点云＋IMU → `navigation/odom`及`odom→base_link`、`lio/cloud_registered`；地图服务`lio/save_map`保存PCD，地图坐标为建图实例的odom系，重载时作为配置地图坐标，站点和二维栅格必须一起核对。地图路径预配置，已有文件拒绝覆盖；地图体素数有上限，达到上限锁定失败。
- `lio_localization`：重载PCD，接收`initialpose`后用粗／细两级ICP估计`map→odom`。初值按同一观测的`T_map_base × inverse(T_odom_base)`换算；匹配要求点数、fitness、RMSE和修正幅度同时达标。单工作线程／单待处理扫描，旧初值对应的异步结果丢弃；失败或过期不继续刷新TF。不是地点识别、任意位置重定位或回环优化。
- `lio_scan`：按同时间的LIO位姿和显式外参，把去畸变点云投影为`navigation/scan`。切片内没有回波的方向标为NaN未知，不能当作无遮挡；高度和外参必须按实际传感器设置。可选发布`base_link→lidar`静态TF，已有机器人描述发布该边时关闭。
- `livox_input`：可选订阅已运行的`livox_ros_driver2/CustomMsg`，保留header/timebase与逐点offset_time，按原参考的line/tag规则过滤。需单独安装真正的Livox驱动消息包；本入口不启动设备驱动。该分支尚未与真实Livox驱动联调。

点云输入要求小端FLOAT32的`x/y/z/intensity/time`，`time`为相对header采集起点的秒数，**不允许用接收时间或零偏移伪造逐点时间**。IMU为m/s²（含重力）和rad/s，必须和点云同一ROS时钟；启动时保持静止初始化。外参采用xyz＋xyzw，`imu_from_lidar`将雷达坐标转换到IMU坐标，`imu_from_base`将底盘坐标转换到IMU坐标。配置显式指定所有frame，不静默套用历史MID360/Tracer外参。输入过期、IMU间断、扫描重叠、队列超限或跟踪失效锁定后需重启估计器；节点不发布运动指令，导航侧仍由原有定位时效／停止契约接管。

ROS 2 Jazzy工作区增加PCL/Eigen依赖后构建；ICP的Open3D等可选依赖需独立安装，不混入VR/IK构建环境：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select bindu_lio bindu_runtime --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
# 仅给定位进程使用的可选Python依赖
python3 -m pip install --target artifacts/lio-python -r tools/requirements-navigation.txt
```

启动示例中的传感器配置是**合成测试fixture**，零外参不是实机标定，也不是现有Isaac二维射线输入的适配：

```bash
ros2 launch bindu_runtime lio.launch.py namespace:=/bindu_lio_test \
  sensor_config:="$PWD/src/integration/bindu_runtime/config/navigation_lio_fixture.json" \
  mode:=mapping save_map:="$PWD/artifacts/lio-room.pcd"
# 有输入并完成建图后，另一个终端保存；不会覆盖已有PCD
ros2 service call /bindu_lio_test/lio/save_map std_srvs/srv/Trigger '{}'
```

重载定位时先收尾建图实例，给**定位进程所在终端**增加`PYTHONPATH="$PWD/artifacts/lio-python:$PYTHONPATH"`，再用相同入口的`mode:=localization pcd_map:=/绝对路径/map.pcd`并发送`initialpose`。已有同坐标的二维栅格可用`grid_map:=/绝对路径/map.yaml`启动Nav2 map_server；不可把旧地图随意换名使用。随后原`navigation_physics.launch.py`可消费相同命名空间的`navigation/odom`、`navigation/scan`和TF；算法速度仍必须经过目标身份、租约和执行层。此模式独占map→odom和odom→base，不能并行启动AMCL/SLAM或模拟器旧odom发布者。Isaac三维物理复测入口见下文；各批通过与失败边界单独记录。

验证入口：`tests/validate_lio.py --binary install/bindu_lio/lib/bindu_lio/fast_lio --output artifacts/lio-check`使用时间展开的合成三维房间扫描与IMU，检查动态估计、采集时间、错误字段拒绝、断流及PCD保存；`tests/validate_lio_localization.py --map artifacts/lio-check/map.pcd --output artifacts/lio-localization-check`验证真实ROS点云重载定位、错误匹配和重设初值；`test_lio_localization.py`、`test_lio_ros_adapters.py`覆盖配准和扫描外参。以上不连接真实机器人。

### 三维算法的 Isaac 同场景复测

复用上面的房间、站立收臂模型、`navigation_mapping_sites.json`和Nav2装配，Isaac启动参数改为`--navigation-sensors src/integration/bindu_runtime/config/navigation_lio_isaac.json`。该配置独立描述8个仰角、360个方位、每12个物理步完成一帧的三维射线扫描和IMU；逐束采样使用当时的PhysX姿态，`time`保存实际采集偏移。5mm距离噪声沿用二维基线，IMU另加0.02m/s²和0.002rad/s的高斯噪声，均为未标定的测试假设。

`tools/isaac_lio_sensors.py`只发布原始点云和IMU，不发布机器人位置、地图或TF。IMU通过物理姿态的差分获得，使用与执行器一致的墙上采集时间，并转换重力/角速度到传感器坐标；这不是特定雷达型号的完整误差模型。建图时FAST-LIO按测量时间发布map→odom原点定义；定位时该边仅由ICP发布，避免保留静态恒等变换跨模式冒充有效定位。

新工作区先重建`bindu_lio`和`bindu_runtime`。Open3D运行依赖只在运行时加入`PYTHONPATH`，不要让其附带的setuptools进入colcon构建环境。保持与Isaac相同的ROS域，Nav2按前面的`require_scan:=true`入口启动；第三个终端运行：

```bash
python3 tests/validate_localization_physics.py --backend lio \
  --sensor-config src/integration/bindu_runtime/config/navigation_lio_isaac.json \
  --robot src/integration/bindu_runtime/config/navigation_g1_fixture.json \
  --sites src/integration/bindu_runtime/config/navigation_mapping_sites.json \
  --output artifacts/lio-physics-tests
```

验证器复用二维基线的五个建图航点、初值偏置、往返目标及位置/航向/停稳判据。保存原生PCD，并从带同时间LIO位姿的实际注册点云生成5cm栅格：观测射线标自由空间，高度0.1–0.95m的回波标占据，0.2m点云体素按0.1m半宽保守覆盖；没有读取场景障碍物坐标生成地图。这个切片与体素覆盖是本次fixture的配置，替换传感器外参、体素或场景高度时须同步调整。物理真值只用于传感器合成、设备反馈及独立误差评估。

FAST-LIO断流锁定后要求显式重启，故新测试在确认停车后重启LIO/ICP并重新提供近似初值，再提交回站目标；不会通过扫描恢复自动续跑旧目标。单独复测定位可追加`--phase localization --pcd-map /绝对路径/room.pcd --map /同坐标地图/room.yaml`。原始点云以字段信息和base64数据保存于压缩观测日志；失败批次不覆盖。比较精度时须同时说明二维/三维输入、轮式误差与IMU噪声的差异，不能把差值全部归因于算法。

2026-09-22同场景实测分批完成：FAST-LIO五航点建图到点误差16.7–33.1mm，建图定位RMS15.9mm；PCD与5cm栅格保存、ICP重载初值收敛通过。桌边单程定位RMS25.5mm，到站35.1mm；行进中断雷达触发`NAV_POSE_STALE`，额外移动44.1mm、约0.634秒后满足150ms停稳窗口，显式重启并用最后算法位姿初始化后回站57.5mm。10项LIO模块、2项IMU采样、6项旧导航和3项Fast DDS原生身份检查通过。原始数据、地图、曲线与失败批次保留在`artifacts/lio-physics-2026-09-22/verification.json`索引中。定位单程与建图属于不同阶段，不应混用误差统计。

**完整正常往返仍未通过**：返程准备及后续重测出现约107–320ms的仿真反馈间断，FAST-LIO按50ms IMU间断保护锁定；新增耗时诊断把主要停顿定位到反馈读取／发布阶段，尚未确认更底层原因。启动前连续5秒反馈稳定的门槛不能消除后续故障。隔离CycloneDDS对照可定位，但原生发布者身份校验拒绝合法输入，未绕过校验或切换项目默认中间件。`--navigation-cases fault`可独立复现断流/恢复，`roundtrip`可独立复测正常往返；默认`all`保持完整顺序。这些通过是分批结果，不等于一次端到端全绿，也不证明真实雷达精度或算法必然优于二维基线。

## Pi 客户端运行入口

- 核心代码：[`bindu_vla/pi`](src/capabilities/bindu_vla/bindu_vla/pi/)。
- 配置示例：[`pi_loopback.json`](src/integration/bindu_runtime/config/pi_loopback.json)。
- `pubsub`：机器人发布观测、订阅动作。
- `pull`：机器人回复模型端的观测请求，同时订阅动作。

手工联调时，先配置服务端点、关节映射及观测输入，再启动客户端：

```bash
ros2 launch bindu_runtime skeleton.launch.py pi_enabled:=true
```

在同一 ROS 域、已加载工作区的另一终端中查询就绪状态并发起会话：

```bash
ros2 service call /bindu_sim/vla/pi/ready std_srvs/srv/Trigger '{}'
ros2 action send_goal /bindu_sim/vla/pi/session bindu_interfaces/action/PiSession \
  "{task_id: pi_demo, prompt: 'pick water', resource_group: arm, duration: 5.0}" --feedback
```

缺少新鲜观测时拒绝启动，原因通过 `PI_GOAL_REJECTED` 事件和节点日志给出。会话持续时间结束表示控制会话停止，不代表抓取成功。目前每个会话控制一个关节组，默认使用模拟设备，ACT 未接入。

## VR 遥操作运行入口

核心代码位于 [`bindu_teleoperation`](src/capabilities/bindu_teleoperation/bindu_teleoperation/) 和 [`bindu_kinematics`](src/capabilities/bindu_kinematics/bindu_kinematics/)。本批接入 v3.4 的 `CONTROLLER_MOVE` 输入、OpenXR 坐标转换和相对控制，使用独立进程进行 Pinocchio FK / CasADi 连续 IK，经公共执行服务驱动 **7 轴左臂模拟器**。输入端与机器人驱动分离，后续仍以 VR 为遥操作入口。

示例配置 `teleop_v34.json` 使用 `mapping_mode: "robot_base"`，重新握持时维持空间方向；`operator_yaw_rad` 用于启动前水平朝向校准。旧配置未提供模式时保留 `v34_anchor` 行为；需要旧操作习惯可显式选择该模式并将偏角设为0。坐标和旋转定义见[架构中的单臂映射规则](软件架构设计.md#854-当前代码组织与替换边界)。

可选依赖在 Ubuntu 24.04 / Python 3.12 / x86_64 验证；Orin 尚未验证。先按快速开始构建并加载 ROS 与工作区，再在当前工作区隔离安装依赖。下面的库路径用于避免 ROS 自带 EigenPy 与 Pinocchio wheel 混用，仅影响当前终端：

```bash
python3 -m pip install --target artifacts/teleop-deps -r tools/requirements-teleop.txt
export BINDU_TELEOP_DEPS="$PWD/artifacts/teleop-deps"
export PYTHONPATH="$BINDU_TELEOP_DEPS:$BINDU_TELEOP_DEPS/cmeel.prefix/lib/python3.12/site-packages:$PYTHONPATH"
export LD_LIBRARY_PATH="$BINDU_TELEOP_DEPS/cmeel.prefix/lib:$LD_LIBRARY_PATH"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
export ROS_DOMAIN_ID=119 ROS_LOCALHOST_ONLY=1
python3 tests/validate_teleop.py --output artifacts/teleop-check
```

专项验证启动自己的模拟进程：覆盖启动入口、合成手柄事件经真实 Vuer WebSocket 服务接入、暂停/重接、取消、断流、追踪无效、连接身份改变、不可达目标和记录器退出，包含显示状态与显示进程退出隔离检查。2026-09-18 在 Ubuntu / bindu Conda 环境重建3包，85项模块及最终11个遥操作场景通过；首轮2项在故障注入前等待显示超时，补齐观察节点就绪前置检查后全组通过，保留首轮失败记录。真实头显的浏览器、TLS、跟踪质量与操控感受仍需现场联调。

连接头显时，使用本机有效的 TLS 证书和私钥路径，并让头显能访问输入服务地址。证书不随代码分发；默认仅监听 `127.0.0.1`，下面显式开启局域网输入：

```bash
ros2 launch bindu_runtime teleop.launch.py vr_enabled:=true host:=0.0.0.0 \
  cert_file:=/绝对路径/cert.pem key_file:=/绝对路径/key.pem
```

`vr_enabled:=true` 同时启动只读 `teleop_display` 节点。页面新增：

- 青色线框为请求的末端目标，橙色球为模拟关节反馈经 FK 算出的末端；RGB 轴分别表示末端局部 XYZ。
- 状态面板显示请求的跟随/暂停模式、结束或拒绝原因，以及当前目标与反馈的位姿误差；误差不是 IK 求解残差，也不是实际到达确认。
- 目标、输入或反馈过期时不继续显示有效目标/误差；显示节点超过 0.6 秒未更新时清除两组姿态并显示 `DISPLAY_STALE`。整个 Vuer 服务或网络断开时页面可能停留在最后一帧，须检查连接状态。
- 图形为 `robot_base` 坐标示意，转到 Y 向上的查看坐标并向前平移 1 米；不是机器人与操作者房间的空间配准。模型未加入碰撞几何。

观察节点以约 10 Hz 独立计算反馈 FK，使用 best-effort 订阅和容量为 1 的非阻塞显示队列，不持有执行控制权。字体由服务本地提供，需安装依赖清单中的 Pillow 11.3.0；请使用输入主机的页面，不使用远端托管的 Vuer 页面。单独启动输入节点时，观察节点须使用相同 namespace、profile、run_id 和 teleop_config。

头显访问输入主机的 HTTPS 8012 端口并进入 VR。先确认页面显示 `WAITING` 且橙色反馈末端已出现，再在另一个已加载相同 ROS 域和工作区的终端查询就绪并开启会话。显示是可选观察者，`teleop/ready` 不等待它；观察者晚启动或重启时需新会话恢复完整模式信息：

```bash
ros2 service call /bindu_sim/teleop/ready std_srvs/srv/Trigger '{}'
ros2 action send_goal /bindu_sim/teleop/session bindu_interfaces/action/TeleopSession \
  "{task_id: vr_demo, resource_group: left_arm, duration: 60.0}" --feedback
```

- 每次会话先松开左握持键，再握持超过 1 秒进入跟随；松开暂停，重新握持后以当前实测关节 FK 和当前 VR 姿态重新建立基准。左 B 停止会话，接收端锁存此停止输入，重新连接浏览器后方可再次启动。
- `init` 按键目前明确返回 `VR_INIT_NOT_CONFIGURED`；尚未有核实过的初始姿态，不自动回零。扳机值会记录，尚未控制灵巧手。右臂、双臂/手部并发、图像显示及训练数据同步未接入本批。
- [遥操作配置](src/integration/bindu_runtime/config/teleop_v34.json)与[本体配置](src/integration/bindu_runtime/config/huawei_v34_left_sim.json)定义资源、具名关节、时间限制、比例和模型。旧模型只保留运动学/惯性文本，未包含碰撞几何；非活动关节固定在配置值，不能当作当前本体标定。
- VR 时间戳为输入服务收到事件的时刻，未提供头显采样时间或网络时延估计；轮询不会刷新旧帧时间。输入、模式、IK 残差/耗时、接受目标与实测反馈异步记录，示教图像和完整训练数据集尚未实现。
- 会话到时表示控制结束，不代表抓取成功。运行层继续拒绝真实设备模式；本批没有连接电机、部署真机服务或承诺 IK 达到 100 Hz。

## 共享执行层验证

关节参考统一由 [C++ 内核](src/control/bindu_execution/native/interpolation.hpp)生成；[Python 接口](src/control/bindu_execution/bindu_execution/interpolation.py)只负责对象/时间轴适配，不再保留第二套曲线数学实现。在线目标用自适应策略，定时 P2P、导数轨迹和动作块用时序策略，共用曲线求值、限幅及制动。ROS 执行器继续管理 revision、控制权和实测到达确认，语义见[动作执行契约](动作执行契约.md#2-输入模式必须显式选择)。原实验目录已合入 `native/`，历史源码仍只读。

构建需要 C++17 编译器和与运行环境匹配的 Python 开发头文件。`colcon build` 自动构建原生扩展；模块缺失时直接报导入错误，不静默回退到 Python。更新后须重建并加载工作区，不能复制另一架构或 Python 版本的 `.so`。无 ROS 的本地测试可先构建：

```bash
(cd src/control/bindu_execution && python3 setup.py build_ext --inplace)
```

加载工作区后运行模块和 ROS 回归，`timed_chunks` 包含动作块原始时间轴、旧 revision 拒绝和减速取消：

```bash
python3 -m unittest discover -s tests -p 'test_*.py'
python3 tests/validate_ros.py --case online_targets --case timed_chunks --output artifacts/execution-check
```

本轮本地验证：14组纯 C++ 测试及 ASan/UBSan 通过，覆盖七轴定时 P2P、分轴限幅、带导数轨迹、过期动作块/制动尾段、在线预览一致性、模式接续和运动中停止；另包含6种输入频率、30组非定时单轴 P2P、34,408个随机流采样。68项 Python 调用与执行回归通过，真实 IK 类因缺 Pinocchio/CasADi 跳过；源码分发包在干净目录重新编译和加载也通过验证。9月18日 Ubuntu 原生后端实测：13包构建、71项模块测试（含真实 IK）、14组 C++、16个 ROS、14个 Pi 和9个 VR 场景通过。修复了 colcon symlink-install 下 C++ 源码路径解析；Pi 首轮一次就绪服务回复超时，原用例连续3次及全组复测通过，未改判定条件。该偶发通信问题保留记录，未进行真机或 Orin 验收。

纯 C++ 测试不依赖 ROS、Python 或历史源码：

```bash
mkdir -p artifacts/native-test
c++ -std=c++17 -O2 -Wall -Wextra -pedantic \
  -I src/control/bindu_execution/native \
  src/control/bindu_execution/native/adaptive_stream.cpp \
  src/control/bindu_execution/native/interpolation.cpp \
  tests/test_adaptive_stream.cpp -o artifacts/native-test/test_native
artifacts/native-test/test_native
```

保留历史源码的主工作区可运行以下对照。工具分别测旧 C++、标量自适应、新原生在线/五次曲线策略，输出 CSV/JSON；原生策略耗时包含 Python 绑定开销，七轴项目使用实际在线策略。当前本机离线正弦 RMS 为0.099219 rad，旧内核为0.097335 rad；固定时序策略不会为了降低误差自行改变轨迹时间。测试值不等于机器人定位精度或端到端延迟，内核仍有动态分配，Python/ROS 串行执行壳仍在，不宣称硬实时。

```bash
python3 tools/compare_interpolators.py --legacy-root 历史代码/legs_necks_control \
  --output artifacts/interpolation-comparison
```

## 开发与文档

- [软件架构](软件架构设计.md)：模块职责、替换接口与实现边界。
- [动作执行契约](动作执行契约.md)：目标、控制权、时间轴与停止语义。
- [协作规范](AGENTS.md)：开发验证要求和文件管理规则。

本地开发后提交并推送；目标机在保留自身未提交修改的前提下使用 `git pull --ff-only`，重新构建，并用 `git rev-parse HEAD` 核对版本。已有独立 Git 历史的工作区需先完成迁移。

## License

见 [MIT License](LICENSE)。历史第三方参考资产遵循各自来源许可。
