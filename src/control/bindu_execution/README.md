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

- 当前统一Snowbot VR位置契约使用`DelegatedPositionExecutor`，直接交目标给选定位置设备；准备、普通双臂及扫雪共用该接口。它仅接纳非定时`joint_target`，不承诺通用曲线的导数或到达时间。滤波和插值的实际路径见[下表](#当前snowbot-vr链路的滤波与插值)。
- 旧`tool_pose`运动学装配仍可使用通用`Executor`及`snowbot_shovel_sim.json`的同步插值；该路径与当前统一位置契约分别由装配选择，不能按配置名称推断正在使用的算法。
- 换握、暂停和取消依赖新鲜实测反馈确认停稳。反馈过期或速度未知不能报告已停稳，停止超时会锁存未确认状态并阻止新接管。

PICO协调器管理准备、夹爪请求与控制权交接；旧模拟入口使用短参考段，统一位置配方使用直接位置目标及独立夹爪服务。操作资格、确认和停止规则统一见[工具操作契约](../../../动作执行契约.md#61-snowbot-工具操作与握持切换)。真实设备接口已适配，实体夹持、现场标定及真机效果仍待验收。

两种执行路径由装配选择；上层不重复叠加底层已有的柔顺控制。场景与入口见[扫雪运行装配](../../integration/bindu_runtime/README.md#扫雪遥操作)，完整时间／停止规则见执行契约。

## 源码导航

| 入口 | 内容 |
|---|---|
| [executor.py](bindu_execution/executor.py) | Executor、控制权、指令接纳、状态机与反馈检查 |
| [interpolation.py](bindu_execution/interpolation.py) | 原生数学内核的 Python 接口 |
| [delegated_position.py](bindu_execution/delegated_position.py) | 底层位置控制的委托执行路径 |
| [native/interpolation.cpp](native/interpolation.cpp) | 定时轨迹插值与限制检查 |
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

保留历史源码的主工作区可运行以下对照。工具分别测旧 C++、标量自适应、新原生在线/五次曲线策略，输出 CSV/JSON；原生策略耗时包含 Python 绑定开销，七轴项目使用实际在线策略。当前本机离线正弦 RMS 为0.099219 rad，旧内核为0.097335 rad；固定时序策略不会为了降低误差自行改变轨迹时间。测试值不等于机器人定位精度或端到端延迟，内核仍有动态分配，Python/ROS 串行执行壳仍在，不宣称硬实时。

```bash
python3 tools/compare_interpolators.py --legacy-root 历史代码/legs_necks_control \
  --output artifacts/interpolation-comparison
```



## 当前Snowbot VR链路的滤波与插值

| 环节 | 当前位置契约的实际路径 |
|---|---|
| 手柄位姿滤波 | 普通双臂与扫雪分别使用`IndependentArmsSession`和`BimanualSession`，未实例化`OneEuroPose`；One Euro实现供配置启用的独立单臂`TeleopSession`使用 |
| 共享位置执行 | `DelegatedPositionExecutor.prepare_path`保留直接位置目标，不调用通用执行器的原生自适应／S曲线生成；profile中的`joint_target_interpolation=adaptive`不能据此证明此路径使用了自适应插值 |
| 当前PhysX设备 | 腰与双臂共用一个限速向量进度，生成一致的位置／速度参考；实际关节由物理驱动跟随，头部与夹爪独立 |
| 真实设备 | 上层同样委托位置设备负责插值／保持；底层实际使用的曲线、动态限制和同步性能需按现场源码、配置与实测核对 |

原生插值代码仍由通用`Executor`使用。本轮仅整理入口与说明，未改变任何运行滤波、限速或插值配置。
