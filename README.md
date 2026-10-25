# Bindu

Bindu 是基于 ROS 2 的模块化机器人控制系统，面向遥操作与移动操作。通过公共接口连接目标输入、运动求解、任务能力和设备控制，使算法与机器人硬件能够独立替换。

Bindu维护硬件无关的机器人能力、共享执行契约和运行装配，允许按应用需求修改相关代码。本轮双臂遥操是Snowbot的定制场景；专属操作流程、场景参数和现场资料由文稿中的[snowbot工作区](../../snowbot/README.md)维护，可复用能力与接口在Bindu完善。

## Snowbot 应用入口

Snowbot 型号实现、模型与运行配方集中在独立的 [Snowbot 工作区](../../snowbot/README.md)：`snowbot_bringup` 管装配与配置，`snowbot_description` 管模型，`snowbot_hardware` 管厂商协议，`snowbot_teleop` 管扫雪操作流程。Bindu 保留公共契约、求解、执行和通用节点；原 Snowbot Python 模块仅保留兼容转发。运行 Snowbot 需要配套取得两个仓库，旧单仓库安装须按[部署说明](src/integration/bindu_runtime/README.md#x86-真机部署已有适配底层)重建，不能把本机目录移动视为部署完成。

## 项目结构

```text
bindu/
├── src/                        # ROS 2 功能包；模块内 README 提供构建与运行命令
│   ├── shared/                 # 公共契约、消息与服务
│   ├── control/                # 共享执行与控制权
│   ├── hardware/               # 设备适配、模型与网格
│   ├── capabilities/           # 遥操作、运动学、导航、感知、规划与 VLA
│   ├── tasks/                  # 任务编排
│   ├── data/                   # 数据记录
│   └── integration/            # ROS 节点、launch 与运行配置
├── tools/
│   ├── teleop/                 # VR 输入、操作台与反馈观察
│   ├── simulation/             # Isaac 场景、物理后端与控制进程
│   ├── models/                 # 本体模型及仿真资产准备
│   ├── deployment/             # 通用部署配方与环境验证
│   ├── analysis/               # 离线轨迹、工作空间与实验分析
│   ├── requirements/           # 可选 Python 依赖清单
│   ├── config/                 # 工具使用的通信配置
│   ├── web/                    # 操作台静态页面
│   └── *.sh                    # 常用启动入口
├── tests/                      # 单元、集成、物理、回放与公共夹具
├── README.md                   # 项目入口与模块导航
├── 运行与验证.md                # 环境、测试和运行说明入口
└── build/、install/、log/、artifacts/  # 本机构建及实验产物，不提交
```

常用启动命令保留在 `tools/*.sh`；Python 工具从仓库根目录以 `python3 -m tools.<分类>.<模块>` 调用，例如 `python3 -m tools.models.prepare_snowbot_description --help`。原始参考资产和本地过程记录保持原位。

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

## 文档阅读与维护

| 需要了解的内容 | 维护位置 |
|---|---|
| 模块职责、装配边界及尚未实现的设计 | [软件架构](软件架构设计.md)，具体实现见各模块 README |
| 动作身份、时间、控制权、反馈与停止语义 | [动作执行契约](动作执行契约.md) |
| 环境、构建、运行和验证步骤 | [运行与验证](运行与验证.md)；配置选择见[运行装配](src/integration/bindu_runtime/README.md#配置目录) |
| Snowbot／118现场配方、问题与验收 | [Snowbot工作区](../../snowbot/README.md)及[底层驱动问题记录](../../snowbot/底层驱动问题记录.md) |

主工作区另有本地的`项目上下文.md`、`当前进度.md`、`决策记录.md`与`过程记录/`，分别保存稳定条件、当前摘要、重要取舍和历史证据；历史选型、来源对照、源码盘点及架构图册位于本地`参考文档/`，按主题检索；这些本地材料不随Git分发，新克隆按本文、工程文档与源码使用。Snowbot链接指向独立本地工作区，未取得该目录时不影响阅读Bindu公共工程说明。

### 按任务定位

局部修改从对应模块的职责、源码和验证入口开始；仅在影响跨模块接口时扩读相关契约与上下游。文档链接提供查阅入口，不构成逐篇全文必读清单。

| 本次任务 | 直接入口 |
|---|---|
| 手柄按键、位姿映射、滤波、握持与会话 | [遥操作](src/capabilities/bindu_teleoperation/README.md#按任务查阅) |
| 插值、期限、控制权与停止 | [执行模块](src/control/bindu_execution/README.md#扫雪位置执行)、[执行契约](动作执行契约.md#6-执行状态与停止)相关章节 |
| 模型、设备接口与反馈 | [本体模型](src/hardware/bindu_description/README.md#机器人模型与资产)、[设备适配](src/hardware/bindu_hardware/README.md#snowbot-位置接入) |
| 场景配置、部署或验证 | [运行装配](src/integration/bindu_runtime/README.md#按任务查阅)、[运行与验证](运行与验证.md#按需查阅) |
| 导航与其他能力 | 从上方模块导航选择本次涉及的模块 |

## 当前状态与验证入口

已接入单臂、普通双臂与工具耦合遥操作、共享执行、Snowbot位置设备接口及可选PhysX反馈；实体握持、真实设备标定和完整真机运行尚未验收。功能边界与配置选择见[运行装配](src/integration/bindu_runtime/README.md)，输入滤波及插值的实际路径见[位置执行说明](src/control/bindu_execution/README.md#当前snowbot-vr链路的滤波与插值)。

测试按单元、集成、物理、回放与公共夹具分类；命令和前提统一见[分组测试](运行与验证.md#测试)，平台与部署限制见[环境准备](运行与验证.md#环境准备与构建)。历史批次结果保留在本地过程记录和实验产物中。
