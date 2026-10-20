# bindu_tasks：任务编排与场景状态

[项目模块导航](../../../README.md#模块快速导航) · [编译与运行](#编译与运行) · [职责与边界](#职责与边界) · [源码导航](#源码导航) · [接入与验证](#接入与验证) · [关联模块](#关联模块)

通过能力端口组织任务流程。当前取饮料示例用于验证导航、观测、候选计划和共享执行的组合，不代表完整真机抓取已实现。

## 编译与运行

在仓库根目录执行；需要 ROS 2、colcon 及 [基础依赖](../../../运行与验证.md#环境准备与构建)。Humble 环境将 `jazzy` 换为 `humble`。

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-up-to bindu_tasks --executor sequential
source install/local_setup.bash
python3 -c "from bindu_tasks.scene import Scene; print('held_object:', Scene().held_object)"
```

本包提供流程库，ROS Action 节点是 `bindu_runtime task`。完整取饮料模拟需感知、规划、执行、设备及记录一起运行；先准备 [运行装配依赖](../../integration/bindu_runtime/README.md#编译与运行)，再在终端一启动：

```bash
colcon build --base-paths src --packages-up-to bindu_runtime --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE="$(command -v python3)" -DPYTHON_EXECUTABLE="$(command -v python3)"
source install/local_setup.bash
ros2 launch bindu_runtime skeleton.launch.py runtime_mode:=simulation recording_mode:=normal
```

终端二在仓库根目录加载相同 ROS 与工作区，保持 ROS 域一致。先查询就绪，返回 `success: true` 后再提交模拟任务：

```bash
ros2 service call /bindu_sim/tasks/ready std_srvs/srv/Trigger '{}'
ros2 action send_goal /bindu_sim/tasks/fetch_drink bindu_interfaces/action/FetchDrink \
  '{task_id: readme_demo, object_id: drink, strategy: planner}' --feedback
```

启动本身不会自动执行任务，Action 会使模拟机器人运行取饮料流程；结果带 `simulated` 标记。结束后在启动终端按 Ctrl+C 收尾。真实感知、抓取和持物确认尚未接通。

## 职责与边界

- 组织前往取物点、定位、接近、抓取和返回流程，并维护观测与持物状态。
- 不直接连接硬件或 ROS 网络；ROS 动作与执行调用由运行节点适配。
- 当前接近／抓取使用固定测试目标，抓取确认依赖模拟条件；仅关节到位不能作为真机抓住物体的证据。

## 源码导航

| 入口 | 内容 |
|---|---|
| [fetch_drink.py](bindu_tasks/fetch_drink.py) | 取饮料示例流程与能力调用 |
| [scene.py](bindu_tasks/scene.py) | 场景观测和 held_object 状态 |
| [task_node.py](../../integration/bindu_runtime/bindu_runtime/task_node.py) | ROS 动作、依赖连接与任务执行适配 |
| [FetchDrink.action](../../shared/bindu_interfaces/action/FetchDrink.action) | 任务请求、结果和进度反馈 |

## 接入与验证

更换能力时通过公共端口与运行装配接入，并保持观测时间／坐标系、控制权、取消和持物语义一致。新增真实抓取流程需要独立的抓取确认依据。

- 行为检查：[模拟能力](../../../tests/test_capabilities.py)、[系统契约](../../../tests/test_system_contracts.py)。
- 示例任务 ROS 检查：[validate_ros.py](../../../tests/validate_ros.py)。
- 执行约束：[动作执行契约](../../../动作执行契约.md)；运行环境：[运行与验证](../../../运行与验证.md)。

## 关联模块

[导航](../../capabilities/bindu_navigation/README.md) · [感知](../../capabilities/bindu_perception/README.md) · [规划](../../capabilities/bindu_planning/README.md) · [运行装配](../../integration/bindu_runtime/README.md)
