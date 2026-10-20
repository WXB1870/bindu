# bindu_lio：三维里程计与地图定位

[项目模块导航](../../../README.md#模块快速导航) · [编译与运行](#编译与运行) · [职责与边界](#职责与边界) · [源码导航](#源码导航) · [接入与验证](#接入与验证) · [关联模块](#关联模块)

集成 FAST-LIO 雷达／IMU 估计、已有地图的 ICP 定位及导航扫描转换。算法进程与控制执行分开运行。

## 编译与运行

在仓库根目录执行；需要 ROS 2、colcon、C++17、PCL、Eigen3、OpenMP 及 `package.xml` 中的 ROS 依赖，见 [环境准备](../../../运行与验证.md#环境准备与构建)。Humble 环境将 `jazzy` 换为 `humble`。

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-up-to bindu_lio --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE="$(command -v python3)" -DPYTHON_EXECUTABLE="$(command -v python3)"
source install/local_setup.bash
python3 tests/validate_lio.py \
  --binary "$PWD/install/bindu_lio/lib/bindu_lio/fast_lio" \
  --output "artifacts/lio-check-$(date +%Y%m%d-%H%M%S)"
```

上述验证自带合成点云／IMU并收尾进程，不连接真实雷达。常规建图由运行包提供 launch；先准备 [运行装配依赖](../../integration/bindu_runtime/README.md#依赖准备)，再执行：

```bash
colcon build --base-paths src --packages-up-to bindu_lio bindu_runtime --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE="$(command -v python3)" -DPYTHON_EXECUTABLE="$(command -v python3)"
source install/local_setup.bash
ros2 launch bindu_runtime lio.launch.py namespace:=/bindu_lio_test \
  sensor_config:="$PWD/tests/config/navigation/navigation_lio_fixture.json" \
  mode:=mapping save_map:="$PWD/artifacts/lio-room.pcd"
```

该配置仅为合成输入夹具，launch 不产生传感器数据，也不启动雷达驱动；无输入时不会完成建图。现场必须替换话题、坐标系、外参和点时间配置。有输入并完成建图后，另一已加载相同环境／ROS 域的终端可调用 `ros2 service call /bindu_lio_test/lio/save_map std_srvs/srv/Trigger '{}'` 保存 PCD，已有文件拒绝覆盖；Ctrl+C 收尾。

重载地图定位还需在独立运行环境安装 `python3 -m pip install --target artifacts/lio-python -r tools/requirements-navigation.txt`，仅在定位终端加入 `export PYTHONPATH="$PWD/artifacts/lio-python${PYTHONPATH:+:$PYTHONPATH}"`。随后使用 `mode:=localization pcd_map:=/绝对路径/map.pcd` 并提供初始位姿；完整步骤见 [三维导航入口](../../../运行与验证.md#历史三维导航迁移入口)，不要将 Open3D 依赖目录混入 colcon 构建环境。

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
