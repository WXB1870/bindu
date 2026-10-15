# bindu_hardware：设备适配与反馈汇总

[项目模块导航](../../../README.md#模块快速导航) · [职责与边界](#职责与边界) · [源码导航](#源码导航) · [接入与验证](#接入与验证) · [关联模块](#关联模块)

把公共设备端口接到模拟驱动、外部物理仿真或厂家接口，并向执行层提供统一反馈。

## 职责与边界

- 分开维护设备驱动、命令路由和反馈汇总；`RobotIO` 负责组合这些组件。
- 保留反馈来源、源时间、仿真标记与未知速度，汇总停止失败，支持底盘里程计和关节位置映射。
- Snowbot 厂家位置适配与二代 SOEM 只读反馈分别维护；配置映射需要与实机关节顺序和单位一致。
- 模拟驱动用于协议和行为验证，不代表物理仿真；真实设备运动与标定仍需现场验收。

## 源码导航

| 入口 | 内容 |
|---|---|
| [robot_io/system.py](bindu_hardware/robot_io/system.py) | RobotIO 组合与 ExecutionIO 实现 |
| [robot_io/routing.py](bindu_hardware/robot_io/routing.py) / [feedback.py](bindu_hardware/robot_io/feedback.py) | 命令分发／反馈汇总 |
| [robot_io/odometry.py](bindu_hardware/robot_io/odometry.py) | 轮式底盘里程计 |
| [drivers/](bindu_hardware/drivers/) | 模拟关节、手、底盘及外部物理仿真驱动 |
| [snowbot.py](bindu_hardware/snowbot.py) / [position_mapping.py](bindu_hardware/position_mapping.py) | 厂家位置接口与配置映射 |
| [soem_feedback.py](bindu_hardware/soem_feedback.py) | 二代 SOEM 反馈转换 |

## 接入与验证

设备选择在 [device_assembly.py](../../integration/bindu_runtime/bindu_runtime/device_assembly.py) 完成。Snowbot ROS 适配入口为 [snowbot_adapter.py](../../integration/bindu_runtime/bindu_runtime/snowbot_adapter.py)，二代反馈入口为 [soem_joint_feedback.py](../../integration/bindu_runtime/bindu_runtime/soem_joint_feedback.py)。

- 配置示例：[Snowbot 厂家位置](../../integration/bindu_runtime/config/snowbot_vendor_position.json)、[二代位置映射](../../integration/bindu_runtime/config/position_mapping_wheel_gen2.json)。
- 行为检查：[物理反馈](../../../tests/test_physics.py)、[里程计](../../../tests/test_odometry.py)、[Snowbot 适配](../../../tests/test_snowbot_adapter.py)。
- 接入步骤和限制：[Snowbot 厂家位置控制适配层](../../../运行与验证.md#snowbot-厂家位置控制适配层)。

## 关联模块

[公共契约](../../shared/bindu_contracts/README.md) · [共享执行](../../control/bindu_execution/README.md) · [本体模型](../bindu_description/README.md) · [运行装配](../../integration/bindu_runtime/README.md)
