# bindu_navigation：导航适配与输入检查

[项目模块导航](../../../README.md#模块快速导航) · [职责与边界](#职责与边界) · [源码导航](#源码导航) · [接入与验证](#接入与验证) · [关联模块](#关联模块)

维护导航配置、站点与每次目标的输入检查，并提供 Nav2 速度身份检查和 Airy 扫描转换。Nav2 会话的 ROS 装配位于 `bindu_runtime`。

## 职责与边界

- 根据目标身份、发布进程、序号、时间戳和坐标系检查速度输入，避免旧会话或新进程的速度混入当前控制。
- 校验地图位姿、到站范围与停稳反馈；导航速度经控制权和共享执行入口送到底盘。
- 分别维护协议模拟基线与实际 Nav2／传感器适配。Nav2、SLAM、AMCL 是装配使用的外部能力。
- 已有仿真验证；真实雷达、外参、底盘参数和长期运行仍需现场验证。

## 源码导航

| 入口 | 内容 |
|---|---|
| [navigation.py](bindu_navigation/navigation.py) | NavigationConfig、站点、NavigationGate 与 SessionVelocityFence |
| [simulated.py](bindu_navigation/simulated.py) | 导航协议模拟实现 |
| [native/nav2_velocity_identity.cpp](native/nav2_velocity_identity.cpp) | 原生 Nav2 速度来源身份检查 |
| [airy_scan.py](bindu_navigation/airy_scan.py) / [scripts/airy_scan_bridge](scripts/airy_scan_bridge) | Airy 点云到导航扫描的转换与节点入口 |
| [CMakeLists.txt](CMakeLists.txt) | Python 与原生组件的安装和依赖 |

## 接入与验证

ROS 会话入口：[runtime/nav2_session.py](../../integration/bindu_runtime/bindu_runtime/nav2_session.py)、[runtime/navigation.py](../../integration/bindu_runtime/bindu_runtime/navigation.py)。传感器参数示例：[navigation_sensors.json](../../integration/bindu_runtime/config/navigation_sensors.json)。

- 单元检查：[导航](../../../tests/test_navigation.py)、[Airy 扫描](../../../tests/test_airy_scan.py)。
- 集成检查：[导航协议](../../../tests/validate_navigation.py)、[Nav2 身份](../../../tests/validate_nav2_identity.py)、[Nav2 物理场景](../../../tests/validate_nav2_physics.py)。各入口需要相应 ROS／仿真环境。
- 配置和运行：[导航适配验证入口](../../../运行与验证.md#导航适配验证入口)、[真实 Nav2 与 Isaac 房间导航](../../../运行与验证.md#真实-nav2-与-isaac-房间导航)、[Airy 预适配](../../../运行与验证.md#二代轮式-airy-导航传感器预适配)。

## 关联模块

[三维定位](../bindu_lio/README.md) · [共享执行](../../control/bindu_execution/README.md) · [设备与里程计](../../hardware/bindu_hardware/README.md) · [运行装配](../../integration/bindu_runtime/README.md)
