# bindu_recording：异步结构化记录

[项目模块导航](../../../README.md#模块快速导航) · [编译与运行](#编译与运行) · [职责与边界](#职责与边界) · [源码导航](#源码导航) · [接入与验证](#接入与验证) · [关联模块](#关联模块)

把收到的控制、反馈和事件交给有界队列与后台写线程，生成压缩 JSONL、清单和汇总，避免在控制回调中直接写磁盘。

## 编译与运行

在仓库根目录执行；需要 ROS 2、colcon 及 [基础依赖](../../../运行与验证.md#环境准备与构建)。Humble 环境将 `jazzy` 换为 `humble`。

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-up-to bindu_recording --executor sequential
source install/local_setup.bash
python3 -c "from bindu_recording.recorder import AsyncRecorder; print(AsyncRecorder.__name__)"
```

本包为记录库，ROS 订阅节点是 `bindu_runtime recorder`。先准备 [运行装配依赖](../../integration/bindu_runtime/README.md#编译与运行)，再在需要采集的话题命名空间启动：

```bash
colcon build --base-paths src --packages-up-to bindu_runtime --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE="$(command -v python3)" -DPYTHON_EXECUTABLE="$(command -v python3)"
source install/local_setup.bash
ros2 run bindu_runtime recorder --ros-args -r __ns:=/bindu_sim \
  -p runtime_mode:=simulation -p recording_mode:=normal \
  -p run_id:="recording_demo_$(date +%Y%m%d-%H%M%S)" -p output:="$PWD/artifacts/recording"
```

另一已加载相同环境的终端用 `ros2 topic echo /bindu_sim/recorder/health --once` 查看健康；Ctrl+C 会排空队列并写入汇总。目录为 `artifacts/recording/<run_id>/`，包含 `manifest.json`、`episode.jsonl.gz` 和 `summary.json`。无上游消息时数据可为空；采集现有会话须与其 namespace、profile、run_id、runtime_mode 一致。已有 launch 若已启动记录器，不要再重复启动。

## 职责与边界

- 支持正常、简洁、无三档；简洁模式对高频反馈／VR 采样并保留关键状态变化，主动采样与意外丢弃分开统计。
- 记录写入、丢弃、错误与收尾状态；日志需要按任务、命令和来源身份关联解释。
- 覆盖范围是记录器实际收到的消息，不能用“丢弃为零”推断上游从未丢数据。
- 当前不是完整训练数据处理、质检或回放系统；模式一致性检查由 ROS 记录节点完成。

## 源码导航

| 入口 | 内容 |
|---|---|
| [recorder.py](bindu_recording/recorder.py) | AsyncRecorder、队列、采样、后台压缩写入与健康汇总 |
| [recorder_node.py](../../integration/bindu_runtime/bindu_runtime/recorder_node.py) | ROS 订阅、消息转换、运行模式检查和退出收尾 |
| [RecorderHealth.msg](../../shared/bindu_interfaces/msg/RecorderHealth.msg) | 跨进程记录健康状态 |

## 接入与验证

启动参数与记录目录由 [运行装配](../../integration/bindu_runtime/README.md) 配置。新增记录类型时确认消息可序列化，并检查队列满、磁盘失败及退出收尾行为。

- 记录档位、产物与限制：[日志记录级别](../../../运行与验证.md#日志记录级别)。
- 行为检查：[test_system_contracts.py](../../../tests/test_system_contracts.py) 中的记录器场景。
- ROS 记录集成检查：[validate_ros.py](../../../tests/validate_ros.py)；模式检查：[validate_runtime_mode.py](../../../tests/validate_runtime_mode.py)。

## 关联模块

[公共契约](../../shared/bindu_contracts/README.md) · [ROS 接口](../../shared/bindu_interfaces/README.md) · [运行装配](../../integration/bindu_runtime/README.md)
