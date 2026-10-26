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
- 记录范围为实际收到的消息，“丢弃为零”仅描述记录器本地队列；上游完整性需结合源端序号核对。
- 当前不是完整训练数据处理、质检或回放系统；模式一致性检查由 ROS 记录节点完成。

腰臂轨迹沿用现有记录链路：`execution/accepted`完整保存`MotionCommand`的目标、关节顺序、原时间和有效期；`execution/state`完整保存执行参考、实测、来源时间及命令／任务身份；`events`完整保存`RuntimeEvent`。节点按消息字段自动序列化，无需为轨迹或可选阻抗重复定义一套日志消息。`ExecutionState.reference`记录执行层发布的参考；servo当前生效参考与最终硬件输出由底层对应字段记录。

Snowbot补充PDO观察由非实时显示进程中的`PdoObserver`接收，经已有`FaultWindow`与`DiagnosticWriter`保存故障及停止前后窗口。总线健康时，执行状态或事件中的`STOPPING`、显式暂停请求和`STOP_UNCONFIRMED`也触发采集。窗口保留同一消息的`boot_id`／`pdo_sample`、会话／命令序号、实测、固定保持回执，以及可选的`wrench`／`sensor_valid_mask`／`sensor_data_no`／`sensor_age`；缺失字段保持缺失，数值不由日志端补齐或推算。`received_ros`／`received_monotonic`是观察进程的接收时间，消息`header.stamp`（保存为`stamp`）是发布时刻，`pdo_age`及各臂`sensor_age`保留源端报告的采样年龄。窗口落盘外层时间记录本次接收，可能晚于原接收时间；设备采样时间使用源时间字段。力样本完整性与现场标定仍需核对传感器采样号、有效位和标定数据。

真实Snowbot显示进程启用诊断日志时，另由只读`WrenchObserver`订阅`/force_sensor/left/wrench`与`/force_sensor/right/wrench`，复用后台队列分别保存`wrench-left.jsonl`、`wrench-right.jsonl`；`wrench-capture.jsonl`保存采集说明，现有诊断快照的`wrench_capture`记录双侧到达计数及接收／发布时效。默认各侧最多100 Hz，frame、非有限值或发布时效状态变化额外保留；主动抽样计入`sampled_out`，队列丢失计入`dropped`，不保证上游全量或短暂力峰完整捕获。日志保留`force[x,y,z]`、`torque[x,y,z]`、原消息`publication_stamp_ns`／`frame_id`和回调接收时间；不变换坐标、不做去皮或力控，也不与PDO按近邻时间拼接。旧底层话题在传感器缺失时可能发布六个0，且缺少采样号／有效位／采样年龄，因此`sensor_valid`／`sensor_data_no`／`sensor_age_s`保持null，标定、单位尺度和发布者身份标记未验证；`finite`、`publication_fresh`及`recently_received`描述数值与话题投递；真实力采样有效性和零外力状态保持未确认。缺侧和陈旧状态明确显示，不补零；新增日志不参与运动、准入或安全判定。

观察端还可按原值复制消息存在时的`reference_mode`／`segment_id`／`sample_time_ns`、`reference_position/speed/acceleration[16]`及`output_position/speed/torque/kp/kd[16]`／`output_valid`，仅随上述选中窗口落盘。数组顺序、单位、源时钟和“最终输出”的实际含义以servo消息及写入点为准；这些数据仅在消息字段与servo接入完成后提供，缺失时保留为空。`q_goal`由原接纳命令记录；接入侧须另在现有接纳事件或段元数据中建立`command_id`与`session/command_sequence/segment_id`、租约／epoch和原期限的映射，观察端不按时间接近推断对应关系。

停止期间，适配器在现有执行循环旁路发布`STOP_CHECK`，沿用`runtime-events.jsonl`保存。保持回执检查与实际接纳共用判据，记录首个未通过原因、请求／观察的boot与会话序号、PDO号、使能掩码、反馈时间、误差／速度与阈值、停稳窗口及原停止预算余量。回执未通过时仍记录保持点误差，`ack_reason`保留实际回执校验失败原因。原因变化立即发布，停止等待中的重复原因每0.5秒最多一次，最终故障不周期刷屏；诊断不授予控制权、不续期、不改变停止或使能策略。

这些队列只供ROS回调和非实时观察侧使用；Python的`put_nowait`仍包含锁，不能从PDO实时线程调用。servo只旁路复制固定大小快照，发布和日志在非实时侧完成；不因1000 Hz轨迹采样默认开启全程1000 Hz JSON记录。

操作台诊断在入队／ROS序列化前采样：VR原始／投影输入按流和左右手分别最多10 Hz，执行／遥操／夹爪／模式状态及连续IK／QP诊断最多10 Hz，健康摘要1 Hz；按键边沿、接合／释放阈值、有效性及状态变化额外保留，已接纳命令与会话映射事件完整保留。PDO故障／停止窗口默认100 Hz（状态变化额外保留），前10秒／后30秒、前窗容量1200帧；监视仍逐帧检查，抽样不会改变控制或健康判定。新命令的暂停重新开启完整后窗，同一次停止的重复状态／事件不延长窗口；重叠窗口按event及源PDO身份关联，原接收时间不变，退出时报告窗口是否完整。后台约每秒刷新，`sampled_out`与意外`dropped`分开报告，PDO抽样计数另见`log_sampled_out`。这些采样日志用于日常排查，不保证完整原始VR回放或逐周期分析；本节策略独立于`recording_mode`。

真实Snowbot观察器还只读接收`/rosout`中`servo_node`／`dual_arm_control_node`／`robot_upperMonitor`及其子logger的WARN以上记录，保存到`rosout-log-events.jsonl`。订阅使用100深度、best-effort／volatile，不请求历史、不作为保护输入；记录范围为实际收到的消息，上游完整性另行核对。相同logger／级别／调用点／原文首次立即入队，重复每5秒汇总观测次数与首末时间；不同电机、错误码及原文数值不合并，128键有界缓存，退出前结算，日志静默不判定故障恢复。独立终端的普通`printf`／`std::cerr`仍须另行保存，既往未保存内容无法补录。

VR健康采样按固定阶段分别保留1 Hz摘要，浏览器计时、服务端循环、ROS发布间隔/耗时与可选`input_pipeline`不共用一个采样槽；异常/恢复和来源变化额外保留。阶段键有界，未知阶段合并到一个槽，源身份仅作为状态变化记录，重连时缓存大小保持有界。普通报告与其他阶段的慢发布证据使用独立采样槽；该规则只影响诊断采样，不改变控制阈值。分段时间定义及开启方式见[输入诊断](../../capabilities/bindu_teleoperation/README.md#vr-首次连通检查)。

统一入口默认开启`--diagnostics`，显式`--no-diagnostics`才关闭，与示教采集开关独立。`control.log`／`display.log`／可选`physics.log`沿用文本格式，由同一有界后台写入器缓冲、约每秒刷新，并按128MiB分卷。管道读取持续进行，已识别的ROS WARN以上重复行按上述策略合并为`[LOG_REPEAT]`摘要，异常栈及其他文本原样保留；`process-log-health.json`分别报告合并数、队列丢弃与写入错误，不与观察器的`diagnostics-health.json`混用。网页`events.jsonl`保持原格式，也使用有界异步写入，状态见`web-log-health.json`及API的`event_log`字段；关闭诊断时只保留内存近期事件。聚合只减少本记录侧开销，不减少驱动自身打印／ROS发布或原日志落盘；突然终止可能损失缓冲内容。

需要完整输入时使用统一入口的`--record-inputs`，范围、丢失检查及纯离线回放命令见[VR完整输入记录与离线回放](../../capabilities/bindu_teleoperation/README.md#vr-完整输入记录与离线回放)。

## 源码导航

| 入口 | 内容 |
|---|---|
| [recorder.py](bindu_recording/recorder.py) | AsyncRecorder、队列、采样、后台压缩写入与健康汇总 |
| [recorder_node.py](../../integration/bindu_runtime/bindu_runtime/recorder_node.py) | ROS 订阅、消息转换、运行模式检查和退出收尾 |
| [RecorderHealth.msg](../../shared/bindu_interfaces/msg/RecorderHealth.msg) | 跨进程记录健康状态 |

## 接入与验证

启动参数与记录目录由 [运行装配](../../integration/bindu_runtime/README.md) 配置。新增记录类型时确认消息可序列化，并检查队列满、磁盘失败及退出收尾行为。

- 记录档位、产物与限制：[日志记录级别](#日志记录级别)。
- 行为检查：[test_system_contracts.py](../../../tests/unit/test_system_contracts.py) 中的记录器场景。
- ROS 记录集成检查：[validate_ros.py](../../../tests/integration/validate_ros.py)；模式检查：[validate_runtime_mode.py](../../../tests/integration/validate_runtime_mode.py)。

## 关联模块

[公共契约](../../shared/bindu_contracts/README.md) · [ROS 接口](../../shared/bindu_interfaces/README.md) · [运行装配](../../integration/bindu_runtime/README.md)

## 运行与专项验证

以下命令和项目路径均相对于仓库根目录。带日期内容仅代表当时的专项实验，现行控制规则以动作执行契约和统一部署为准。

### 日志记录级别

运行入口统一支持 `recording_mode:=normal|compact|off`，默认 `compact`（简洁）。当前开发阶段显式使用 `normal`（正常），各验证脚本已显式设置；日志级别专项测试除外。开发启动示例：

```bash
ros2 launch bindu_runtime g1_sim.launch.py recording_mode:=normal
```

| 级别 | 内容 |
|---|---|
| 正常 `normal` | 保留全部收到的指令、执行参考／实际反馈、VR 输入与诊断事件 |
| 简洁 `compact` | 保留全部已接纳指令及关键／异常事件；轨迹和 VR 输入约 10 Hz，状态变化额外保留；省略重复接纳、替换及成功 IK 诊断 |
| 无 `off` | 不订阅记录数据、不启动写入线程、不创建 episode 文件；按用户显式配置允许控制继续运行 |

正常和简洁都保存为 `episode.jsonl.gz`，JSON 结构不变；读取时用 `gzip.open(path, 'rt')`。压缩与磁盘 I/O 在记录器后台线程完成，最多每0.5秒刷新一次；正常退出收尾，异常退出可能损失尚未刷出的数据。`manifest.json`注明级别和数据文件；`summary.json`将主动采样省略的`sampled_out`与意外丢弃的`dropped`分开统计。简洁模式不用于分析高频抖动或完整VR输入回放，`writer_complete`表示所选内容保存完整，源消息接收完整性另行核对。

`off`模式的健康消息`ready`表示关闭模式已就绪，此时不采集数据。此设置只控制结构化记录器；ROS自身控制台日志、验证器原始包／测量数据以及地图等专项产物独立保存。已有实验数据不自动删除，也不自动录制视频／图像。
