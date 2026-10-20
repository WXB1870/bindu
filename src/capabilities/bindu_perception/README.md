# bindu_perception：目标定位模拟接口

[项目模块导航](../../../README.md#模块快速导航) · [编译与运行](#编译与运行) · [职责与边界](#职责与边界) · [源码导航](#源码导航) · [接入与验证](#接入与验证) · [关联模块](#关联模块)

目前提供取饮料流程使用的合成目标定位，不包含真实相机、目标检测或物体位姿估计算法。

## 编译与运行

在仓库根目录执行；需要 ROS 2、colcon 及 [基础依赖](../../../运行与验证.md#环境准备与构建)。Humble 环境将 `jazzy` 换为 `humble`。

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-up-to bindu_perception --executor sequential
source install/local_setup.bash
python3 -c "from bindu_perception.simulated import SimObjectLocator; print(SimObjectLocator().locate('drink'))"
```

上例打印合成目标的 `found`、`frame` 和 `position`。本包没有独立 ROS 可执行程序；定位服务在 `bindu_runtime`。先准备 [运行装配依赖](../../integration/bindu_runtime/README.md#编译与运行)，再启动：

```bash
colcon build --base-paths src --packages-up-to bindu_runtime --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE="$(command -v python3)" -DPYTHON_EXECUTABLE="$(command -v python3)"
source install/local_setup.bash
ros2 run bindu_runtime perception --ros-args -r __ns:=/bindu_sim -p runtime_mode:=simulation
```

另一终端在仓库根目录加载相同 ROS 与工作区后查询：

```bash
ros2 service call /bindu_sim/perception/locate bindu_interfaces/srv/LocateObject \
  '{schema_version: 1, request_id: readme_demo, task_id: demo, object_id: drink}'
```

预期返回 `found: true`、`simulated: true`；Ctrl+C 结束服务。本例不读取真实相机。

## 职责与边界

- 按目标标识返回是否找到、坐标系和位置，供任务编排验证观测路径。
- 当前 `SimObjectLocator` 为已知 `drink` 返回固定地图坐标，不读取图像或点云。
- 感知结果仅作为观测；物体是否抓住、任务是否成功由后续证据与任务状态管理。

## 源码导航

| 入口 | 内容 |
|---|---|
| [simulated.py](bindu_perception/simulated.py) | ObjectEstimate 与 SimObjectLocator |
| [ROS 适配](../../integration/bindu_runtime/bindu_runtime/perception_node.py) | 定位服务与观测消息封装 |
| [LocateObject.srv](../../shared/bindu_interfaces/srv/LocateObject.srv) / [ObjectObservation.msg](../../shared/bindu_interfaces/msg/ObjectObservation.msg) | 跨进程定位接口 |

## 接入与验证

替换定位算法时保留目标标识、坐标系与观测时间，避免把旧观测作为当前结果；由 [运行装配](../../integration/bindu_runtime/README.md) 连接具体实现。

- 模拟能力检查：[test_capabilities.py](../../../tests/test_capabilities.py)。
- 示例任务的 ROS 集成检查：[validate_ros.py](../../../tests/validate_ros.py)。
- 环境准备和检查方式：[运行与验证](../../../运行与验证.md)。这些检查不构成真实感知精度验收。

## 关联模块

[任务编排](../../tasks/bindu_tasks/README.md) · [ROS 接口](../../shared/bindu_interfaces/README.md) · [运行装配](../../integration/bindu_runtime/README.md)
