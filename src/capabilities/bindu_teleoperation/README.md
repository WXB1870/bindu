# bindu_teleoperation：遥操作输入与会话

[项目模块导航](../../../README.md#模块快速导航) · [职责与边界](#职责与边界) · [源码导航](#源码导航) · [接入与验证](#接入与验证) · [关联模块](#关联模块)

处理 VR 输入、坐标映射、位姿滤波、单臂／双臂工具会话与模式切换。ROS 节点由运行装配模块提供。

## 职责与边界

- 把手柄位姿转为机器人坐标下的相对目标，维护握持、松握重接、暂停和重新对齐语义。
- 区分原始 VR 输入与已在机器人基座坐标系中的工具位姿输入，检查时间、序号与会话身份。
- 单臂委托 IK、双臂委托 QP；轨迹平滑和设备写入交给共享执行与设备层。
- 现场头显、网络与真实握持仍需联调；合成输入或本机 WebSocket 验证不代表现场 VR 已验收。

## 源码导航

| 入口 | 内容 |
|---|---|
| [session.py](bindu_teleoperation/session.py) | 单臂相对控制会话 |
| [bimanual_session.py](bindu_teleoperation/bimanual_session.py) | 双臂／工具会话、暂停与已接纳参考 |
| [modes.py](bindu_teleoperation/modes.py) | 手动模式选择与会话切换检查 |
| [config.py](bindu_teleoperation/config.py) | 输入映射与控制配置加载 |
| [vr/](bindu_teleoperation/vr/) | VR 接收、坐标映射、One Euro 滤波和显示 |
| [feedback.py](bindu_teleoperation/feedback.py) | 供显示使用的只读反馈 |

## 接入与验证

ROS 入口：[单臂节点](../../integration/bindu_runtime/bindu_runtime/teleop_node.py)、[工具节点](../../integration/bindu_runtime/bindu_runtime/bimanual_tool_node.py)、[VR 输入节点](../../integration/bindu_runtime/bindu_runtime/vr_input_node.py)。装配与模式配置从 [bindu_runtime](../../integration/bindu_runtime/README.md) 进入。

- 输入与依赖：[VR 遥操作运行入口](../../../运行与验证.md#vr-遥操作运行入口)。
- 模式与自由度：[手动选择本体与自由度](../../../运行与验证.md#手动选择本体与自由度)。
- 单元检查：[会话](../../../tests/test_teleoperation.py)、[模式](../../../tests/test_teleop_modes.py)、[One Euro](../../../tests/test_one_euro.py)。
- ROS 场景：[validate_teleop.py](../../../tests/validate_teleop.py)、[validate_bimanual.py](../../../tests/validate_bimanual.py)，依赖对应模拟装配。

## 关联模块

[运动学](../bindu_kinematics/README.md) · [公共输入契约](../../shared/bindu_contracts/README.md) · [共享执行](../../control/bindu_execution/README.md) · [运行装配](../../integration/bindu_runtime/README.md)
