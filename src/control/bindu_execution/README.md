# bindu_execution：共享执行与停止管理

[项目模块导航](../../../README.md#模块快速导航) · [编译与运行](#编译与运行) · [职责与边界](#职责与边界) · [扫雪位置执行](#扫雪位置执行) · [源码导航](#源码导航) · [接入与验证](#接入与验证) · [关联模块](#关联模块)

为遥操作、规划和 VLA 提供统一执行入口。Python 管理命令与状态，原生 C++ 实现插值和在线轨迹计算。

## 编译与运行

在仓库根目录执行；需要 ROS 2、colcon、C++17 编译器及匹配 Python 的开发头文件，见 [环境准备](../../../运行与验证.md#环境准备与构建)。Humble 环境将 `jazzy` 换为 `humble`。

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-up-to bindu_execution --executor sequential
source install/local_setup.bash
python3 - <<'PY'
from bindu_execution.interpolation import Curve, State
curve = Curve.between(0.0, 1.0, State.rest((0.0,)), State.rest((0.1,)))
print(curve.sample(0.5))
PY
```

上例只运行原生插值计算，预期中点位置为 `0.05`。本包是执行库，ROS 节点名为 `bindu_runtime execution`。需要模拟设备与状态发布时，先按 [运行装配](../../integration/bindu_runtime/README.md#编译与运行) 准备其依赖，再执行：

```bash
colcon build --base-paths src --packages-up-to bindu_runtime --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE="$(command -v python3)" -DPYTHON_EXECUTABLE="$(command -v python3)"
source install/local_setup.bash
ros2 launch bindu_runtime skeleton.launch.py runtime_mode:=simulation recording_mode:=normal
```

该装配同时启动模拟执行、感知、任务和记录节点，不会自动提交任务。其他终端需加载相同环境，可用 `ros2 topic echo /bindu_sim/execution/state --once` 查看反馈；Ctrl+C 收尾。

## 职责与边界

- 管理租约、实例／版本身份、资源占用、命令有效期、关节顺序和动态限制。
- 保持单一执行时间线，处理定时轨迹与在线跟随，并依据新鲜设备反馈确认到达或停稳。
- 对已由底层承担平滑的位置接口使用委托执行路径，避免重复叠加插值；停止后仍需实际反馈确认。
- 通过 `ExecutionIO` 访问设备；本库不导入具体硬件、ROS 节点、GUI 或算法包。

## 扫雪位置执行

- 当前统一Snowbot VR位置契约使用`DelegatedPositionExecutor`，直接交目标给选定位置设备；准备、普通双臂及扫雪共用该接口。上层仅接纳非定时`joint_target`；具体设备协议决定底层参考生成方式，不能仅凭上层命令类型推断底层算法。滤波和插值的实际路径见[下表](#当前snowbot-vr链路的滤波与插值)。
- 旧`tool_pose`运动学装配仍可使用通用`Executor`及`snowbot_shovel_sim.json`的同步插值；该路径与当前统一位置契约分别由装配选择，不能按配置名称推断正在使用的算法。
- 换握、暂停和取消依赖新鲜实测反馈确认停稳。反馈过期或速度未知不能报告已停稳，停止超时会锁存未确认状态并阻止新接管。

PICO协调器管理准备、夹爪请求与控制权交接；旧模拟入口使用短参考段，统一位置配方使用直接位置目标及独立夹爪服务。操作资格、确认和停止规则统一见[工具操作契约](../../../动作执行契约.md#61-snowbot-工具操作与握持切换)。真实设备接口已适配，实体夹持、现场标定及真机效果仍待验收。

真实位置设备的协议分支由运行装配显式选择：

| 协议 | 曲线与参考职责 |
|---|---|
| `legacy_guard`／`session_v1` | 委托已有底层的位置跟随，不从通用执行器推定曲线导数或到达时间 |
| `session_v3` | 上层只发送16轴联合目标及原身份／期限，与servo的`position_reference_mode=target_stream`配对；servo后台复用`PreparedSegment`生成曲线，原周期统一采样，普通发送和PAUSE不等待接纳回执 |

配置、动态限制及可选阻抗的配对条件在[运行装配](../../integration/bindu_runtime/README.md#扫雪遥操作)维护。两种执行路径由装配选择；上层不重复叠加底层已有的柔顺控制。场景与入口见[扫雪运行装配](../../integration/bindu_runtime/README.md#扫雪遥操作)，完整时间／停止规则见执行契约。

## 源码导航

| 入口 | 内容 |
|---|---|
| [executor.py](bindu_execution/executor.py) | Executor、控制权、指令接纳、状态机与反馈检查 |
| [interpolation.py](bindu_execution/interpolation.py) | 原生数学内核的 Python 接口 |
| [delegated_position.py](bindu_execution/delegated_position.py) | 底层位置控制的委托执行路径 |
| [native/interpolation.cpp](native/interpolation.cpp) | 定时轨迹插值与限制检查 |
| [native/CMakeLists.txt](native/CMakeLists.txt) | 不依赖ROS／Python的`bindu::execution_native`静态库导出 |
| [native/adaptive_stream.cpp](native/adaptive_stream.cpp) | 在线自适应位置流 |
| [setup.py](setup.py) | C++17 扩展构建；运行需要已编译的原生模块 |

## 接入与验证

ROS 服务与状态发布由 [execution_node.py](../../integration/bindu_runtime/bindu_runtime/execution_node.py) 适配；驱动实现通过 [设备装配](../../integration/bindu_runtime/bindu_runtime/device_assembly.py) 注入。

- 时间、取消、断流和停稳规则：[动作执行契约](../../../动作执行契约.md)。
- 单元检查：[插值](../../../tests/unit/test_interpolation.py)、[故障序列](../../../tests/unit/test_execution_fault_sequences.py)、[系统契约](../../../tests/unit/test_system_contracts.py)。
- 原生在线流检查：[test_adaptive_stream.cpp](../../../tests/native/test_adaptive_stream.cpp)；ROS 停止检查：[validate_execution_stop.py](../../../tests/integration/validate_execution_stop.py)。
- 构建和运行：[共享执行层验证](#共享执行层验证)。

## 关联模块

[公共契约](../../shared/bindu_contracts/README.md) · [设备适配](../../hardware/bindu_hardware/README.md) · [运行装配](../../integration/bindu_runtime/README.md)

## 运行与专项验证

以下命令和项目路径均相对于仓库根目录。带日期内容仅代表当时的专项实验，现行控制规则以动作执行契约和统一部署为准。

### 共享执行层验证

2026-10-14新增停止／反馈边界检查：异步停止期间保留`STOPPING`，停止失败／超时锁存并拒绝新接管；缺速度不作为零速，上层暂停／停止等待执行终态。完整语义与恢复规则维护于[执行契约第6节](../../../动作执行契约.md#6-执行状态与停止)。首次停止采用固定的ROS／单调时间预算，SIGINT／SIGTERM保留通信直到握手完成或期限到达。`command_enabled`、profile、后端及底层路由为启动配置，修改须停机重启；运行中停止使用既有`execution/control`／撤销租约，不通过修改写权限参数实现。

以下验证启动本仓库执行节点及合成外部设备反馈，不启动Isaac、底层驱动或硬件；每批用新的输出目录：

```bash
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=209 python3 -m tests.integration.validate_execution_stop \
  --output artifacts/execution-stop-check
```

覆盖异步底盘取消／旧租约、停止前旧采样拒绝、实际节点SIGINT／SIGTERM握手、未停稳失败退出和ROS时钟冻结后的单调期限。停止验收直接运行安装后的节点程序，`ros2 run`包装进程的SIGTERM退出码不能代替节点停止结果。初始位置参考为空，只有已成功发送的目标才出现在`reference`；混合仿真／真实反馈、畸形速度维度／数据和设备时间戳回退均拒绝。

本批证据在`artifacts/control-fixes-2026-10-14/`；多轮检查保留修复前的失败、旧用例即时接管预期及ROS包装进程测试问题，不覆盖原数据。Jazzy／Humble的模块、重复停止、模式隔离和Snowbot接口结果由本批`verification.json`索引；仅x86_64软件／合成反馈验证，不能替代真机、ARM64、载荷或实测通信延迟验收。

后续全面检查见`artifacts/comprehensive-qa-2026-10-14/verification.json`：扩大到16包完整构建、全部Python模块、C++与内存／未定义行为检查、100组故障交错、ROS链路及Isaac基础物理验证。复现并修正Snowbot单调停止预算遗漏、真实执行可切换仿真时钟、Python标准测试未接入及LIO默认未优化构建；任务补齐反馈时效门槛、就绪查询和拒绝诊断。旧插值用例和物理验证器同步按独立停稳确认等待，不放宽阈值。首轮QP超时及两轮导航启动失败保留，预检后的复测通过仍不代表长时间负载／DDS调度原因已全部解决；底层全包在Jazzy仍有Python3.10头文件硬编码／KDL依赖问题，真机、ARM64和底层完整目标平台构建不在通过范围。

关节参考统一由 [C++ 内核](native/interpolation.hpp)生成；[Python 接口](bindu_execution/interpolation.py)只负责对象/时间轴适配，不再保留第二套曲线数学实现。在线目标用自适应策略，定时 P2P、导数轨迹和动作块用时序策略，共用曲线求值、限幅及制动。ROS 执行器继续管理 revision、控制权和实测到达确认，语义见[动作执行契约](../../../动作执行契约.md#2-输入模式必须显式选择)。原实验目录已合入 `native/`，历史源码仍只读。

`PreparedSegment`为联合轨迹提供预备与周期采样分离的接口：非实时侧调用`PreparedSegment::target(limits, initial, goal, start, horizon, stop_timeout)`或`stop(limits, initial, start, timeout)`，复用`fit_target`／`fit_stop`的共同时间曲线及整段位置、速度、加速度和jerk限制。静止P2P保持共同路径进度；非零初速和非共线换目标从当前生效参考的`q/v/a`接续，允许计划曲线转弯，不通过逐轴截断改变曲线。首次接管／恢复才从新鲜实测播种。此接口不承诺闭链误差或jerk连续，也不改变原有`Online`策略。

连续位置目标另用`PreparedSegment::stream_target(...)`：非实时侧复用`AdaptiveStream`生成30 ms受限前缀，按所有轴的急动度相位边界拆成共同时间片段，再接原`fit_target`的有限静止尾段。接近静止终点时直接使用原收敛曲线，避免重复目标一直推迟捕获；前缀无法认证时，原整段规划仍须独立通过全部检查。周期侧继续使用同一不可变预备段采样器，不运行在线生成器、不分配或释放内存。此策略允许各轴进度不同，不保证刚性握持或碰撞约束；工具模式须核对生成曲线上的实际几何。它供servo目标流本地规划调用，不通过旧整段轨迹wire导出／重建这些短片段；原`target`、`stop`及旧整段轨迹入口保持原义。

周期仅调用`segment.sample_into(now, q, v, a, j, capacity) noexcept`；四个互不重叠的`double`缓冲须预分配且容量不小于`axes()`。失败不写输出；端点后保持最终位置并返回零`v/a/j`。最多256轴、1024片段，以二分选择片段和固定6元素栈上求值，采样不规划、不分配或释放内存。段创建、验证和对象回收留在非实时侧，调用方须保证采样期间对象存活，不能在周期内销毁最后一个引用。`sample()`及Python对象／缓冲接口仍有分配或解释器开销，不能据此宣称硬实时。

`describe()`导出每个真实片段的`(start, duration, source_duration, source_elapsed, controls[axis])`；位置控制点采用Bernstein基，每轴4个（cubic）或6个（quintic）。`source_duration==0`时按`elapsed/duration`求值，否则按`(elapsed+source_elapsed)/source_duration`保留原曲线裁剪区间。`from_description(description, limits)`只在非实时侧重建导数、核验片段时间／`q/v/a`接续、动态限制及最终静止条件，不拟合、重排或重定时。轴序、源身份、原期限、时钟异常和真实停止确认由调用方管理；取样、派生帧和心跳都不能续期。到达采样端点也不能报告机器人实际到达。Python同名类提供`target`／`stop`／`describe`／`from_description`／`sample_into`。

`session_v3`是纯位置目标流协议（wire 5、reference 2、目标配方version 2）。上层退休曲线发送及跨层提前量配置；旧`session_v2`配方拒绝加载。目标轴序为左臂J1～J7、右臂J1～J7、腰部J1～J2，单位rad；动态界逐轴取profile与发送端只读启动参数`trajectory_max_velocity`（默认0.25 rad/s）、`trajectory_max_acceleration`（默认0.5 rad/s²）及固定162 rad/s³的较小值。消息同时保留session、源command／lease／epoch／profile身份、sequence、源时间及原始期限；中层只透传，派生帧、接纳、采样和心跳不续期。普通目标原始TTL最多300 ms（旧follower／trajectory及阻抗普通目标仍为200 ms）；纯目标流PAUSE作为独立停止命令，由源端一次生成固定2 s原始期限，须在该期限内完成制动参考并转入固定保持，不续期旧目标或授权恢复运动。旧follower／trajectory及阻抗PAUSE仍最多200 ms；新停止命令须与更新后的三层配对。

servo在非实时后台从已采用名义轨迹的未来`q/v/a`快照调用`tracking_target`生成共同时间的联合曲线。速度趋势由相邻已提交目标及原始源时间差估计，避免投递／接纳抖动进入导数；采用40 ms导数平滑并限制在逐轴速度帽的80%，为追赶保留余量。这不是上层QP直接提供的速度，也不再滤波目标位置。普通接续将趋势叠加200 ms响应尺度的位置误差修正；非零趋势下，误差修正最多将其降至一半，避免参考超前后被位置修正拉成反向终点。终点位置由当前及终点速度／加速度的Hermite积分确定，较长连接缩小终点速度，终点距原目标最多为逐轴速度帽对应的200 ms位移；随后接回原目标的有限静止尾段。整段解析认证位置／速度／加速度／急动度，保持C2；这不保证任何初始状态下整段速度绝不反向，真实输入反向仍正常规划。运动连接从100 ms规划时域起搜索可行曲线（不是先等待100 ms）；动态或有限返回尾段不可行时仍回退原`target`。不再以未来40 ms位置误差比较将已可行的运动趋势切回静止终点。首次目标、重复位置、超过100 ms或不足5 ms的源间隔、PAUSE与重新接管清除趋势。消息、原有效期和停止判据不变；周期线程只验证版本、期限及接续条件后切换，拒绝或迟到结果不更新趋势历史。

普通在途生成不因更新取消，待处理普通目标只保留最新一份；PAUSE及控制权／故障边界使旧结果失效，仍调用原`stop`。首次从失能接管用新鲜实测播种；已带电暂停后的重新接管延续原固定参考，避免跟随偏差变成参考跳变，正常更新不逐帧用实测重置。周期采样不规划、不分配或释放内存；共同时间不承诺任意换目标的共同标量进度、闭链几何或jerk连续。旧整段轨迹入口和`stream_target`公共接口仍保留。此处描述本机源码，现场仍须核对选定版本是否已构建并加载。

纯位置PAUSE在原期限内完成制动参考后，由servo复用本地固定参考保持；不再接收运动点也不因2 s到期撤使能。原source时间与期限保持不变，仅继续新鲜的本地采样，普通运动断流及反馈／Fault／撤权／采样超时保护保留。Bindu仍先核对同一PAUSE的有效保持回执，再要求100 ms内新鲜、持续推进的实测满足原速度界及固定实测锚点的位置变化界；名义到位误差单独诊断，不作为target_stream暂停停稳的前提。回执缺失、速度超界或窗口内漂移不能报告停稳，原停止总预算不延长。下发保持、实测停稳与运动目标到达分别判定。

真机target_stream在明确开放写入后，通过正常租约／接管／PAUSE链路完成一次实测原位带电保持；就绪前不接收跟随租约，失败不自动再次使能。Squeeze只控制跟随。目标生产者断流时，执行端在原目标期限前60 ms请求PAUSE：300 ms上限对应源年龄240 ms，显式更短的TTL仍按原值提前制动，启动原位保持目标仍为200 ms。源时间不重置，重复tick／派生帧不续期；本地控制线程停滞仍受底层watchdog约束，不能保证在设备或控制故障时继续带电。

首次／恢复接管保留一次接纳握手，避免op1在中层或DDS的latest缓存被普通目标覆盖；等待期间上层只保留最新目标，原接管期限不变。接纳后普通目标不逐段等待回执；PAUSE立即绕过该等待。接纳／拒绝通过`/bindu/position/servo_receipt`或周期状态观察，配置20 ms执行tick，submit也可触发发送，50 Hz不是硬限频。真实生效以servo采样身份与时刻判断，参考终点不表示真实到达；正常暂停、期限失效、撤权、Fault和STOP_UNCONFIRMED分别保留。目标配置周期及servo名义1 ms采样不变，本地准备预算和离线对比证据在[Snowbot运行说明](../../../../../snowbot/运行与验证.md#执行侧联合目标流)。真实手柄到运动、调度尾峰及持载停止仍待现场验收。

本机[时序对比工具](../../../tools/analysis/compare_session_latency.py)使用相同输入与动态限制比较冻结的旧方案和新目标流，仅覆盖相应本机软件／内存设备替身。结果不能代替ROS/DDS和硬件验收。

刚性双臂预测与[离线回放](../../../tools/analysis/replay_geometric_prediction.py)共用可选的[rigid_prediction.hpp](native/rigid_prediction.hpp)。它将趋势速度投影至相对位姿雅可比的零空间，非线性修正预测末点以保持已接纳目标的六维相对位姿，再约束末速度与含曲率项的末加速度。各轴共用预测增益及100 ms起的连接时域，普通趋势基线复用同一`tracking_target`。保留解析动态限制、有限返回尾段和原采样几何门槛；可行候选不再被40 ms位置误差评分切回静止终点，几何或动态候选失败仍退回无趋势联合曲线。`A`、仅切空间投影与自适应增益保留作离线消融，不是生产策略开关。

真实`session_v3`中，仅配方`rigid_reference_prediction=true`且当前为刚性双臂工具模式时，PICO工具节点在`MotionCommand`显式设置`rigid_reference`；执行器保留原校验，适配器编码为目标配方version 3。普通／独立臂、准备与启动保持仍用version 2，PAUSE也始终使用原version 2停止配方。底层准备器按核对后的实际URDF和左右末帧生成模型头并记录SHA；未构建模型的servo拒绝version 3，不静默改用普通趋势。握持参考与趋势只随已提交段保存，模式边界重新捕获、PAUSE清除；原身份／期限、单后台生成、C2切换和周期采样不变。

工具需要已有的C++17编译器和Eigen头文件，不自动安装依赖。输入沿用原生回放格式，末尾追加一次握持捕获的关节位置；`--names`为同轴序JSON数组，单位rad／m与秒。示例只执行本机参考计算：

```bash
python3 -m tools.analysis.replay_geometric_prediction \
  --urdf path/to/model.urdf --names path/to/names.json \
  --left-frame left_tool --right-frame right_tool \
  --output artifacts/geometric-prediction --input path/to/replay.input
python3 -m unittest tests.unit.test_geometric_prediction
```

预测末点服从六维相对约束，但连接曲线仍只做有限采样几何检查；无趋势回退也不是连续闭链证书。当前不支持轴向五维握持、完整工具／CAD准入或真实闭环保证。后台`TARGET_PREDICTION`记录源session／sequence、生成结果、增益、回退原因和准备耗时，`TARGET_PREDICTION_MODEL`记录模型SHA；这些INFO事件由既有只读观察器保存，不能当作已采用回执。是否实际采用仍以servo参考身份／采样时刻核对。当前仅完成本机软件接入，真实ROS/DDS构建、运行时调度和真机效果未验收。

原生消费者可`add_subdirectory(path/to/native)`并链接`bindu::execution_native`；独立安装后也可`find_package(bindu_execution_native CONFIG REQUIRED)`。Python包会将同一份源码与CMake入口安装到`share/bindu_execution/native`，避免servo另写插值器。独立构建不需要ROS、Python或设备SDK：

```bash
cmake -S src/control/bindu_execution/native -B artifacts/execution-native-build
cmake --build artifacts/execution-native-build
```

构建需要 C++17 编译器和与运行环境匹配的 Python 开发头文件。`colcon build` 自动构建原生扩展；模块缺失时直接报导入错误，不静默回退到 Python。更新后须重建并加载工作区，不能复制另一架构或 Python 版本的 `.so`。无 ROS 的本地测试可先构建：

```bash
(cd src/control/bindu_execution && python3 setup.py build_ext --inplace)
```

加载工作区后运行模块和 ROS 回归，`timed_chunks` 包含动作块原始时间轴、旧 revision 拒绝和减速取消：

```bash
python3 -m tests unit all
python3 -m tests.integration.validate_ros --case online_targets --case timed_chunks --output artifacts/execution-check
```

本轮本地验证：14组纯 C++ 测试及 ASan/UBSan 通过，覆盖七轴定时 P2P、分轴限幅、带导数轨迹、过期动作块/制动尾段、在线预览一致性、模式接续和运动中停止；另包含6种输入频率、30组非定时单轴 P2P、34,408个随机流采样。68项 Python 调用与执行回归通过，真实 IK 类因缺 Pinocchio/CasADi 跳过；源码分发包在干净目录重新编译和加载也通过验证。9月18日 Ubuntu 原生后端实测：13包构建、71项模块测试（含真实 IK）、14组 C++、16个 ROS、14个 Pi 和9个 VR 场景通过。修复了 colcon symlink-install 下 C++ 源码路径解析；Pi 首轮一次就绪服务回复超时，原用例连续3次及全组复测通过，未改判定条件。该偶发通信问题保留记录，未进行真机或 Orin 验收。

纯 C++ 测试不依赖 ROS、Python 或历史源码：

```bash
mkdir -p artifacts/native-test
c++ -std=c++17 -O2 -Wall -Wextra -pedantic \
  -I src/control/bindu_execution/native \
  src/control/bindu_execution/native/adaptive_stream.cpp \
  src/control/bindu_execution/native/interpolation.cpp \
  tests/native/test_adaptive_stream.cpp -o artifacts/native-test/test_native
artifacts/native-test/test_native
```

`tests/native/test_prepared_segment.cpp`用同样编译命令替换测试源即可运行，覆盖16轴共同动态限制、静止与非零初速、连续非共线换目标、停止参考接续、wire多片段及source时间往返、非法缓冲和采样零动态分配。1kHz时间戳序列验证数学接口，不代表操作系统调度或真实设备周期验收。

保留历史源码的主工作区可运行以下对照。工具分别测旧 C++、标量自适应、新原生在线/五次曲线策略，输出 CSV/JSON；原生策略耗时包含 Python 绑定开销，七轴项目使用实际在线策略。当前本机离线正弦 RMS 为0.099219 rad，旧内核为0.097335 rad；固定时序策略不会为了降低误差自行改变轨迹时间。测试值不等于机器人定位精度或端到端延迟，内核仍有动态分配，Python/ROS 串行执行壳仍在，不宣称硬实时。

```bash
python3 -m tools.analysis.compare_interpolators --legacy-root 历史代码/legs_necks_control \
  --output artifacts/interpolation-comparison
```



## 当前Snowbot VR链路的滤波与插值

| 环节 | 当前位置契约的实际路径 |
|---|---|
| 手柄位姿滤波 | 共用Snowbot配方已启用One Euro；普通双臂分别滤波手柄，扫雪滤波合成中点／姿态。按键与时效使用原始输入，参数及重置规则见[遥操作模块](../../capabilities/bindu_teleoperation/README.md#vr-位姿-one-euro-滤波) |
| 共享位置执行 | `DelegatedPositionExecutor.prepare_path`保留直接位置目标，不调用通用执行器的原生自适应／S曲线生成；profile中的`joint_target_interpolation=adaptive`不能据此证明此路径使用了自适应插值 |
| 当前PhysX设备 | 腰与双臂共用一个限速向量进度，生成一致的位置／速度参考；实际关节由物理驱动跟随，头部与夹爪独立 |
| 真实设备 | `legacy_guard/session_v1`委托底层跟随；`session_v3`由servo后台生成联合曲线并统一采样，实际配置和同步性能仍需按现场记录与实测核对 |

原有插值路径仍由通用`Executor`使用；`PreparedSegment`供`session_v3`执行侧非实时生成复用，见[snowbot_position_session.py](../../integration/bindu_runtime/bindu_runtime/snowbot_position_session.py)。软件接入与现场已加载版本、真实周期及停止验收分别核对。输入滤波保持独立。
