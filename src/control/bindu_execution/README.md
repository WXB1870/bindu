# bindu_execution：共享执行与停止管理

[项目模块导航](../../../README.md#模块快速导航) · [职责与边界](#职责与边界) · [扫雪位置执行](#扫雪位置执行) · [源码导航](#源码导航) · [接入与验证](#接入与验证) · [关联模块](#关联模块)

为遥操作、规划和 VLA 提供统一执行入口。Python 管理命令与状态，原生 C++ 实现插值和在线轨迹计算。

## 职责与边界

- 管理租约、实例／版本身份、资源占用、命令有效期、关节顺序和动态限制。
- 保持单一执行时间线，处理定时轨迹与在线跟随，并依据新鲜设备反馈确认到达或停稳。
- 对已由底层承担平滑的位置接口使用委托执行路径，避免重复叠加插值；停止后仍需实际反馈确认。
- 通过 `ExecutionIO` 访问设备；本库不导入具体硬件、ROS 节点、GUI 或算法包。

## 扫雪位置执行

- 当前 ROS 运动学扫雪与 Isaac 物理闭环复用 `Executor`；[snowbot_shovel_sim.json](../../integration/bindu_runtime/config/snowbot_shovel_sim.json) 选择 `joint_target_interpolation=synchronized`，对腰部和双臂联合目标做同步位置执行。
- 独立 Snowbot 位置适配入口使用 `DelegatedPositionExecutor`，直接交目标给底层控制完成平滑与制动。它仅接纳非定时位置目标，不承诺上述曲线的导数或到达时间；当前尚未与真机工具会话联通。
- 换握、暂停和取消依赖实测停稳确认。反馈过期或速度未知时不能报告已停稳，停止超时会锁存未确认状态并阻止新接管。

Snowbot目标准备／装卸、请求—实体确认—QP生效、虚握重锁当前转角和停止恢复规则维护于[工具操作契约](../../../动作执行契约.md#61-snowbot-工具操作与握持切换目标契约)；PICO协调器已在运动学仿真复用上述接口，准备和模拟第8轴使用短参考段并监督时效；实体夹爪与真实恢复仍未实现。Squeeze表示运动离合，Trigger表示实体开合请求，完整映射见[遥操作模块](../../capabilities/bindu_teleoperation/README.md#pico-按键方案)。

两种执行路径由装配选择；底层已有柔顺，上层没有在此叠加柔顺控制。场景与入口见[扫雪运行装配](../../integration/bindu_runtime/README.md#扫雪遥操作)，完整时间／停止规则见执行契约。

## 源码导航

| 入口 | 内容 |
|---|---|
| [executor.py](bindu_execution/executor.py) | Executor、控制权、指令接纳、状态机与反馈检查 |
| [interpolation.py](bindu_execution/interpolation.py) | 原生数学内核的 Python 接口 |
| [delegated_position.py](bindu_execution/delegated_position.py) | 底层位置控制的委托执行路径 |
| [native/interpolation.cpp](native/interpolation.cpp) | 定时轨迹插值与限制检查 |
| [native/adaptive_stream.cpp](native/adaptive_stream.cpp) | 在线自适应位置流 |
| [setup.py](setup.py) | C++17 扩展构建；运行需要已编译的原生模块 |

## 接入与验证

ROS 服务与状态发布由 [execution_node.py](../../integration/bindu_runtime/bindu_runtime/execution_node.py) 适配；驱动实现通过 [设备装配](../../integration/bindu_runtime/bindu_runtime/device_assembly.py) 注入。

- 时间、取消、断流和停稳规则：[动作执行契约](../../../动作执行契约.md)。
- 单元检查：[插值](../../../tests/test_interpolation.py)、[故障序列](../../../tests/test_execution_fault_sequences.py)、[系统契约](../../../tests/test_system_contracts.py)。
- 原生在线流检查：[test_adaptive_stream.cpp](../../../tests/test_adaptive_stream.cpp)；ROS 停止检查：[validate_execution_stop.py](../../../tests/validate_execution_stop.py)。
- 构建和运行：[共享执行层验证](../../../运行与验证.md#共享执行层验证)。

## 关联模块

[公共契约](../../shared/bindu_contracts/README.md) · [设备适配](../../hardware/bindu_hardware/README.md) · [运行装配](../../integration/bindu_runtime/README.md)
