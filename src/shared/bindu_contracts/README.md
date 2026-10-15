# bindu_contracts：公共契约与端口

[项目模块导航](../../../README.md#模块快速导航) · [职责与边界](#职责与边界) · [源码导航](#源码导航) · [接入与验证](#接入与验证) · [关联模块](#关联模块)

定义算法、执行器与设备适配之间共享的数据和接口。它是 Python 库，不依赖 ROS，也不创建节点或连接设备。

## 职责与边界

- 表达运动命令、关节计划、观测、设备反馈和事件，统一关节顺序、单位、时间与身份信息。
- 通过本体配置表达资源组、关节限位、动态限制及执行时序；通过端口隔离具体规划、导航和设备实现。
- 提供仿真／真实模式及遥操作输入契约。模式字段描述运行环境，控制权和设备写权限另行管理。
- 本模块定义契约；指令接纳、物理执行与任务状态由使用方负责。

## 源码导航

| 入口 | 内容 |
|---|---|
| [contracts.py](bindu_contracts/contracts.py) | Motion、Feedback、Observation、JointPlan 等数据类型 |
| [profile.py](bindu_contracts/profile.py) | Profile、本体资源组、关节顺序与限制 |
| [ports.py](bindu_contracts/ports.py) | GraspStrategy、NavigationProvider、ExecutionIO 端口 |
| [devices.py](bindu_contracts/devices.py) | 关节／底盘驱动接口及带时间戳的读数；缺失速度保留为未知 |
| [teleoperation.py](bindu_contracts/teleoperation.py) | VR 与工具位姿输入 |
| [runtime_mode.py](bindu_contracts/runtime_mode.py) | 运行模式规范化与一致性检查 |

## 接入与验证

新增算法或设备时先满足端口和本体配置，再由运行装配选择实现。ROS 消息转换位于 [runtime/common.py](../../integration/bindu_runtime/bindu_runtime/common.py)，消息定义由 `bindu_interfaces` 维护。

- 契约与停止语义：[动作执行契约](../../../动作执行契约.md)。
- 行为检查：[系统契约](../../../tests/test_system_contracts.py)、[本体配置](../../../tests/test_robot_configuration.py)、[运行模式](../../../tests/test_runtime_mode.py)。
- 构建与测试方式：[运行与验证](../../../运行与验证.md)。

## 关联模块

[ROS 接口](../bindu_interfaces/README.md) · [共享执行](../../control/bindu_execution/README.md) · [设备适配](../../hardware/bindu_hardware/README.md) · [运行装配](../../integration/bindu_runtime/README.md)
