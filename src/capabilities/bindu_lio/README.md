# bindu_lio：三维里程计与地图定位

[项目模块导航](../../../README.md#模块快速导航) · [职责与边界](#职责与边界) · [源码导航](#源码导航) · [接入与验证](#接入与验证) · [关联模块](#关联模块)

集成 FAST-LIO 雷达／IMU 估计、已有地图的 ICP 定位及导航扫描转换。算法进程与控制执行分开运行。

## 职责与边界

- 维护雷达／IMU 输入、去畸变点云、里程计、地图保存与定位适配。
- 地图定位采用带初始位姿的两级 ICP，并检查配准质量和位姿变化；不提供任意位置的全局重定位。
- 需要正确的点时间、IMU 轴向、传感器外参与时间同步。Livox 转换入口不代替实体雷达驱动。
- 不直接发底盘命令。已有 Isaac 场景验证，真实传感器和长期运行尚未完成验收。

## 源码导航

| 入口 | 内容 |
|---|---|
| [src/lio_node.cpp](src/lio_node.cpp) | FAST-LIO 的 ROS 2 节点适配，安装为 `fast_lio` |
| [vendor/fast_lio/](vendor/fast_lio/) | 保留的 FAST-LIO 估计与 ikd-Tree 实现 |
| [localization.py](bindu_lio/localization.py) / [lio_localization_node.py](bindu_lio/lio_localization_node.py) | ICP 配准与 `lio_localization` 节点 |
| [lio_scan_node.py](bindu_lio/lio_scan_node.py) | 去畸变点云转二维导航扫描，入口 `lio_scan` |
| [livox_input_node.py](bindu_lio/livox_input_node.py) | Livox 消息转换，入口 `livox_input` |
| [source_manifest.json](source_manifest.json) / [上游许可](vendor/fast_lio/LICENSE) | 迁移来源、指纹与 GPL 许可；vendor 内容按上游许可维护 |

## 接入与验证

启动入口：[lio.launch.py](../../integration/bindu_runtime/launch/lio.launch.py)、[localization.launch.py](../../integration/bindu_runtime/launch/localization.launch.py)。构建涉及 PCL／Eigen 等依赖，ICP 定位使用 Open3D；Livox 消息依赖按实际输入选择。

- 单元检查：[配准](../../../tests/test_lio_localization.py)、[ROS 适配](../../../tests/test_lio_ros_adapters.py)。
- 场景检查：[LIO](../../../tests/validate_lio.py)、[地图定位](../../../tests/validate_lio_localization.py)、[定位物理场景](../../../tests/validate_localization_physics.py)。
- 来源、依赖、建图／地图重载与限制：[历史三维导航迁移入口](../../../运行与验证.md#历史三维导航迁移入口)。

## 关联模块

[导航](../bindu_navigation/README.md) · [设备与里程计](../../hardware/bindu_hardware/README.md) · [运行装配](../../integration/bindu_runtime/README.md)
