# bindu_interfaces：ROS 2 通信接口

[项目模块导航](../../../README.md#模块快速导航) · [编译与运行](#编译与运行) · [职责与边界](#职责与边界) · [源码导航](#源码导航) · [接入与验证](#接入与验证) · [关联模块](#关联模块)

使用 rosidl 定义 ROS 2 消息、服务和动作，供节点之间交换命令、状态与会话信息。此包采用 ament_cmake 构建。

## 编译与运行

在仓库根目录执行，前提为已安装 ROS 2、colcon 和 [接口生成依赖](../../../运行与验证.md#环境准备与构建)。Humble 环境将 `jazzy` 换为 `humble`；每个新终端都需加载 ROS 与工作区。

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-up-to bindu_interfaces --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE="$(command -v python3)" -DPYTHON_EXECUTABLE="$(command -v python3)"
source install/local_setup.bash
ros2 interface show bindu_interfaces/msg/MotionCommand
ros2 interface show bindu_interfaces/action/TeleopSession
```

预期打印生成后的字段定义。本包只生成通信类型，没有独立节点；修改接口后，还需重新构建使用它的包，例如 `colcon build --base-paths src --packages-up-to bindu_runtime --executor sequential`，再重新加载工作区。

## 职责与边界

- 将公共契约映射为 ROS 通信类型，包括时间戳、配置身份、命令身份、租约及运行模式。
- 区分命令提交结果、执行状态和设备反馈；接口响应不能单独证明机器人真实到达。
- 只维护通信定义。业务规则在对应能力、执行库和运行节点中实现。

## 源码导航

| 入口 | 内容 |
|---|---|
| [msg/](msg) | 全部消息定义 |
| [MotionCommand.msg](msg/MotionCommand.msg) | 运动指令、资源组、时间语义与租约身份 |
| [ExecutionState.msg](msg/ExecutionState.msg) | 执行参考、实际反馈、来源、运行模式和停止失败 |
| [VRInput.msg](msg/VRInput.msg) | VR位姿、映射标识、物理键状态和累计边沿；保留旧`grip`字段 |
| [PicoContext.srv](srv/PicoContext.srv) | 绑定来源与状态代次的人工持物／外部支撑声明 |
| [RecorderHealth.msg](msg/RecorderHealth.msg) | 写入健康及独立的采集开关状态 |
| [srv/](srv) | 控制权、运动提交、执行控制、目标定位与故障注入服务 |
| [action/](action) | 任务、导航、Pi、遥操作会话和模式选择动作 |
| [CMakeLists.txt](CMakeLists.txt) | 接口生成与依赖声明 |

## 接入与验证

修改字段或语义时，同时检查 [公共契约](../bindu_contracts/README.md) 与 [ROS 转换](../../integration/bindu_runtime/bindu_runtime/common.py)，重新构建接口及受影响的发布／订阅节点。

- 字段约束与执行语义：[动作执行契约](../../../动作执行契约.md)。
- ROS 集成检查：[validate_ros.py](../../../tests/integration/validate_ros.py)、[模式一致性](../../../tests/integration/validate_runtime_mode.py)。这些脚本需要已构建、已启动的对应测试环境。
- 构建、环境加载与验证命令：[运行与验证](../../../运行与验证.md)。

## 关联模块

[公共契约](../bindu_contracts/README.md) · [共享执行](../../control/bindu_execution/README.md) · [运行装配](../../integration/bindu_runtime/README.md)
