# Bindu

Bindu 是基于 ROS 2 的模块化机器人控制系统，面向遥操作与移动操作。通过公共接口连接目标输入、运动求解、任务能力和设备控制，使算法与机器人硬件能够独立替换。

当前重点是机器人遥操作：在仿真中完善单臂控制、双臂持工具与腰臂协调，为现场 VR 和真实设备接入建立基础。

## 项目内容

- **遥操作**：单臂末端控制、双臂工具位姿跟踪、握持切换、模式选择与会话管理。
- **运动求解与执行**：单臂 IK、双臂联合 QP、腰部协调，以及共享的位置插值、动态限制、控制权和停止处理。
- **任务与能力**：通过公共接口组织导航、感知、规划和 VLA，支持后续移动抓取任务。
- **反馈与数据**：关联记录目标、执行参考、设备反馈和任务事件，支持异步日志与状态观察。
- **本体与仿真**：通过模型、配置和设备适配器表达机器人差异，支持运动学模拟与 Isaac 仿真验证。

## 模块快速导航

按现有 ROS 包划分为以下 16 个模块。各模块 README 提供职责、源码、配置与验证入口；目录存在不代表已完成真机接入。

| 分类 | 模块 | 主要职责 |
|---|---|---|
| 公共接口 | [bindu_contracts](src/shared/bindu_contracts/README.md) | 与 ROS 无关的数据契约、设备端口和本体配置 |
| 公共接口 | [bindu_interfaces](src/shared/bindu_interfaces/README.md) | ROS 2 消息、服务和动作定义 |
| 执行控制 | [bindu_execution](src/control/bindu_execution/README.md) | 控制权、指令校验、插值、动态限制和停止确认 |
| 本体与设备 | [bindu_hardware](src/hardware/bindu_hardware/README.md) | 设备适配、命令路由、反馈汇总和里程计 |
| 本体与设备 | [bindu_description](src/hardware/bindu_description/README.md) | URDF、网格和模型资产 |
| 运动求解 | [bindu_kinematics](src/capabilities/bindu_kinematics/README.md) | FK、单臂 IK、碰撞检查和双臂求解封装 |
| 运动求解 | [bindu_bimanual_qp](src/capabilities/bindu_bimanual_qp/README.md) | 原生双臂持工具联合 QP 与 C ABI |
| 目标输入 | [bindu_teleoperation](src/capabilities/bindu_teleoperation/README.md) | VR 输入、位姿映射、遥操作会话和模式管理 |
| 目标输入 | [bindu_vla](src/capabilities/bindu_vla/README.md) | 远端 Pi 推理客户端与动作接入 |
| 移动能力 | [bindu_navigation](src/capabilities/bindu_navigation/README.md) | 导航输入校验、Nav2 速度身份检查和扫描适配 |
| 移动能力 | [bindu_lio](src/capabilities/bindu_lio/README.md) | FAST-LIO、三维地图定位与点云适配 |
| 操作能力 | [bindu_perception](src/capabilities/bindu_perception/README.md) | 目标定位接口的模拟实现 |
| 操作能力 | [bindu_planning](src/capabilities/bindu_planning/README.md) | 候选关节计划的模拟策略 |
| 任务编排 | [bindu_tasks](src/tasks/bindu_tasks/README.md) | 取饮料示例流程和场景状态 |
| 数据记录 | [bindu_recording](src/data/bindu_recording/README.md) | 异步结构化记录、采样和记录健康状态 |
| 运行装配 | [bindu_runtime](src/integration/bindu_runtime/README.md) | ROS 节点、launch、配置和模块装配 |

公共说明：[软件架构](软件架构设计.md) · [动作执行契约](动作执行契约.md) · [运行与验证](运行与验证.md) · [配置分类](src/integration/bindu_runtime/README.md#配置目录)。

### 扫雪遥操作阅读路径

先看[运行装配与场景选择](src/integration/bindu_runtime/README.md#扫雪遥操作)，再按“[工具输入与握持](src/capabilities/bindu_teleoperation/README.md#扫雪输入与握持) → [腰臂联合求解与反馈](src/capabilities/bindu_kinematics/README.md#扫雪求解与反馈) → [位置执行](src/control/bindu_execution/README.md#扫雪位置执行) → [Snowbot 位置接入](src/hardware/bindu_hardware/README.md#snowbot-位置接入)”阅读。ROS 运动学遥操作与 Isaac Sim 6.2 物理受力实验各有独立入口，配置和验证范围见运行装配页。

PICO扫雪：[统一部署与启动](运行与验证.md#vr-统一部署与启动) · [按键与相对映射](src/capabilities/bindu_teleoperation/README.md#pico-按键方案) · [操作流程](运行与验证.md#pico-扫雪操作流程目标方案) · [工具操作与握持契约](动作执行契约.md#61-snowbot-工具操作与握持切换)。

真机部署：[Snowbot自动化入口](运行与验证.md#部署自动化)，提供现场配方生成、独立构建、部署检查、前台启动及只读状态查看，默认只读。

## 当前进展

截至 **2026-10-17**，已完成模拟框架及主要遥操作链路、Snowbot 底层位置接口适配，并增加配置级仿真／真实模式选择与消息一致性检查。[Snowbot真机最小遥操作](运行与验证.md#snowbot-真机最小遥操作)只启动VR、双臂及位置执行相关节点，默认只读且不启动记录；新增独立夹爪位置闭环，示例未标定时拒绝夹爪写入；真机运行尚未验收。

| 模块 | 当前进展 |
|---|---|
| 遥操作 | 单臂相对控制、位姿滤波、双臂工具输入和手动模式切换已接入；新增独立PICO映射与准备／装卸／握持协调器，合成输入下的暂停、恢复、取消及断流处理已验证；实体握持未接通 |
| 运动学与工具控制 | 单臂 IK、双臂联合 QP 与自动腰臂协调已实现；支持双手刚性握持及右手绕杆轴转动的虚握，并检查限位与配置范围内的几何约束 |
| 共享执行 | 已接入原生 C++ 插值、动态限制和控制权管理，完善停稳确认、未知速度处理及停止失败接管封锁；完成模拟回归及部分 Isaac 物理执行验证 |
| 本体与设备 | 已增加按本体选择模型、自由度和接口映射的启动校验；Snowbot 专用位置适配与二代只读反馈转接已通过合成 ROS 检查，真机执行、二代完整模型及实体握持尚未验收 |
| 导航 | 已接入 Nav2、二维 SLAM／AMCL 和三维 FAST-LIO／ICP，已有仿真验证；真实雷达与长期运行待验证 |
| VLA | Pi 远端推理客户端已通过模拟服务测试，真实模型服务待联调 |
| 任务、感知与数据 | 任务编排、感知和规划已有模拟实现；异步记录按当前模式标记并拒绝混入另一模式的数据，完整移动抓取和训练数据管线待完成，语音能力预留 |

Snowbot 扫雪示例仅以雪铲位姿和握紧状态驱动双臂与两轴腰部，腰角由联合求解自动分配。48.4 秒合成工具输入的多轮 ROS 理想关节回归中，位置跟踪峰值约 **10–11 mm**，反馈位姿的 CAD 几何检查通过。新增 [Isaac Sim 6.2 木质空铲物理闭环](运行与验证.md#木质雪铲的-physx-受力实验2026-10-15)，实测关节状态接回共享 QP／执行器，选配六维载荷的握持变形观测，并独立记录工具真值、双臂载荷与内力。真实柔顺参数、雪载荷、实体握持和现场 VR 仍待验收。

扫雪能力已抽取为独立 Snowbot 项目，只包含扫雪相关模块；控制核心与 Bindu 同源，同输入下的 4881 个离线执行点关节差为 0，并完成独立包构建与 ROS 模拟验证。

## 已验证平台与目标平台

| 类别 | 平台与环境 | 当前状态 |
|---|---|---|
| 已验证：开发机 | x86_64，Ubuntu 24.04，ROS 2 Jazzy，Python 3.12 | 已完成模块测试、ROS 模拟回归及部分 Isaac 仿真验证 |
| 已验证：Humble 容器 | x86_64，Ubuntu 22.04，ROS 2 Humble，Python 3.10 | 已完成遥操作相关 15 包构建、模块测试与实际 ROS 模拟回归 |
| 目标：Jetson Orin | ARM64；目标系统为 Ubuntu 22.04／ROS 2 Humble | 尚未在 Orin 上验证构建、运行与性能 |
| 目标：i9 mini PC | x86_64；目标系统为 Ubuntu 22.04／ROS 2 Humble | 尚未完成目标机真机部署与实时控制验收 |

两块目标控制器计划分担计算与实时控制，具体职责分配仍待确定。实时内核为后续计划，现场设备、网络和长期调度性能仍需实测。
