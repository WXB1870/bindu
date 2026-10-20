# bindu_navigation：导航适配与输入检查

[项目模块导航](../../../README.md#模块快速导航) · [编译与运行](#编译与运行) · [职责与边界](#职责与边界) · [源码导航](#源码导航) · [接入与验证](#接入与验证) · [关联模块](#关联模块)

维护导航配置、站点与每次目标的输入检查，并提供 Nav2 速度身份检查和 Airy 扫描转换。Nav2 会话的 ROS 装配位于 `bindu_runtime`。

## 编译与运行

在仓库根目录执行；先准备 ROS 2、colcon、C++17 和 [包声明的 ROS 依赖](../../../运行与验证.md#环境准备与构建)。Humble 环境将 `jazzy` 换为 `humble`。

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-up-to bindu_navigation --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE="$(command -v python3)" -DPYTHON_EXECUTABLE="$(command -v python3)"
source install/local_setup.bash
ros2 pkg executables bindu_navigation
python3 -m unittest discover -s tests -p 'test_airy_scan.py' -v
```

可执行列表应包含 `nav2_velocity_identity` 与 `airy_scan_bridge`。前者需由 Nav2 会话绑定目标身份后使用，后者需有效传感器参数与 TF；不能仅启动程序就认为导航可用。

要运行含任务／执行链的协议模拟，先准备 [运行装配依赖](../../integration/bindu_runtime/README.md#编译与运行)，再执行：

```bash
colcon build --base-paths src --packages-up-to bindu_runtime --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE="$(command -v python3)" -DPYTHON_EXECUTABLE="$(command -v python3)"
source install/local_setup.bash
python3 tests/validate_navigation.py \
  --output "artifacts/navigation-check-$(date +%Y%m%d-%H%M%S)"
```

验证器自行启动并收尾模拟节点和导航协议替身，不使用真实底盘或实际 Nav2 规划。实际 Nav2／Isaac 运行需额外安装导航与仿真依赖，按 [房间导航步骤](../../../运行与验证.md#真实-nav2-与-isaac-房间导航) 启动；真实雷达和外参按 [Airy 接入](../../../运行与验证.md#二代轮式-airy-导航传感器预适配) 配置。

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

ROS 会话入口：[runtime/nav2_session.py](../../integration/bindu_runtime/bindu_runtime/nav2_session.py)、[runtime/navigation.py](../../integration/bindu_runtime/bindu_runtime/navigation.py)。传感器参数示例：[navigation_sensors.json](../../integration/bindu_runtime/config/navigation/navigation_sensors.json)。

- 单元检查：[导航](../../../tests/test_navigation.py)、[Airy 扫描](../../../tests/test_airy_scan.py)。
- 集成检查：[导航协议](../../../tests/validate_navigation.py)、[Nav2 身份](../../../tests/validate_nav2_identity.py)、[Nav2 物理场景](../../../tests/validate_nav2_physics.py)。各入口需要相应 ROS／仿真环境。
- 配置和运行：[导航适配验证入口](../../../运行与验证.md#导航适配验证入口)、[真实 Nav2 与 Isaac 房间导航](../../../运行与验证.md#真实-nav2-与-isaac-房间导航)、[Airy 预适配](../../../运行与验证.md#二代轮式-airy-导航传感器预适配)。

## 关联模块

[三维定位](../bindu_lio/README.md) · [共享执行](../../control/bindu_execution/README.md) · [设备与里程计](../../hardware/bindu_hardware/README.md) · [运行装配](../../integration/bindu_runtime/README.md)
