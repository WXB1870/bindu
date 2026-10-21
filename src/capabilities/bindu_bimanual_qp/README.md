# bindu_bimanual_qp：原生双臂工具 QP

[项目模块导航](../../../README.md#模块快速导航) · [编译与运行](#编译与运行) · [职责与边界](#职责与边界) · [源码导航](#源码导航) · [扫雪接口与库版本](#扫雪接口与库版本) · [接入与验证](#接入与验证) · [关联模块](#关联模块)

基于 Pinocchio、ProxSuite 和 Eigen 的 C++17 联合求解库，输出 `libbindu_bimanual_qp.so`。通过 C ABI 提供双臂与腰部持工具控制，以及普通双臂的独立末端目标。

普通模式使用两个末端任务，不施加刚性工具闭合约束；共享腰部和未接合臂的速度固定为0，仍使用同一组关节位置／速度边界。新增`bq_capture_independent`、`bq_flange_poses`、`bq_solve_independent`为ABI v1的增量能力，旧工具接口与布局不变。调用者负责控制权、反馈时效、组合路径碰撞和离合交接，本库不发送设备命令。

## 编译与运行

在仓库根目录执行。需要 ROS 2／colcon、C++17、Eigen3、SIMDe，以及版本匹配的 Pinocchio／ProxSuite；wheel 安装方式及 `CMAKE_PREFIX_PATH`、`LD_LIBRARY_PATH` 设置见 [运行装配依赖准备](../../integration/bindu_runtime/README.md#依赖准备)。Humble 环境将 `jazzy` 换为 `humble`，并在加载 ROS 后设置数值依赖路径。

```bash
source /opt/ros/jazzy/setup.bash
# 若使用 wheel，在此按上述依赖准备加载其环境。
colcon build --base-paths src --packages-up-to bindu_bimanual_qp --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE="$(command -v python3)" -DPYTHON_EXECUTABLE="$(command -v python3)"
source install/local_setup.bash
python3 - <<'PY'
import ctypes
from pathlib import Path
from ament_index_python.packages import get_package_prefix
library = Path(get_package_prefix('bindu_bimanual_qp')) / 'lib/libbindu_bimanual_qp.so'
print('ABI:', ctypes.CDLL(str(library)).bq_abi_version())
PY
```

当前预期输出 `ABI: 1`。本包是共享库，没有 `ros2 run` 节点；由 [BimanualQP](../bindu_kinematics/README.md#编译与运行) 加载。原生求解回归可单独运行：

```bash
colcon test --packages-select bindu_bimanual_qp --event-handlers console_direct+
colcon test-result --test-result-base build/bindu_bimanual_qp --verbose
```

测试目标需构建时保持 `BUILD_TESTING` 开启并安装 ament_cmake_gtest；它使用测试模型，不发送设备命令。

## 职责与边界

- 联合处理工具位姿跟踪、握持约束、关节限制、接近限位时的速度约束和姿态目标。物理软限位按距硬边界的距离收缩向外速度，斜率为`min(damper_gain, vmax/limit_margin)`；软带内仍可继续跟踪，硬边界向外速度为0。实测与接纳参考同时约束，积分不得越过硬限位；配置的姿态包络和参考余量另行保留。
- 支持双手刚性握持与右手绕杆轴转动的虚握：右握点三维位置和杆轴方向仍受约束，不释放沿杆平移；具体资源布局和几何由调用方配置。
- 输出候选关节结果，已接纳参考由调用方提交；不承担设备写入、控制权、ROS 通信或执行调度。
- 当前不是双臂内力优化器，未将实机允许力／力矩作为经过标定的控制约束。

## 源码导航

| 入口 | 内容 |
|---|---|
| [include/bindu_bimanual_qp/c_api.h](include/bindu_bimanual_qp/c_api.h) | 对外 C ABI、句柄、输入输出与状态码 |
| [include/dual_arm_tool_qp/core.hpp](include/dual_arm_tool_qp/core.hpp) | 原生求解器内部 C++ 接口 |
| [src/core.cpp](src/core.cpp) | QP 目标、约束与求解 |
| [src/c_api.cpp](src/c_api.cpp) | C ABI 参数检查与封装 |
| [CMakeLists.txt](CMakeLists.txt) | 依赖、共享库安装与原生测试目标 |
| [LICENSE](LICENSE) | 本模块许可 |

## 扫雪接口与库版本

上层通常通过 Python 的 `BimanualQP` 使用本库。下列 C 函数属于 Python 调用已编译 `.so` 的二进制接口（ABI）；使用新功能时，Python 封装和动态库需要匹配。

| 原生入口 | 扫雪用途 |
|---|---|
| `bq_capture_axial` | 捕获允许右手绕杆转动的握持约束 |
| `bq_solve_tracking` | 跟踪误差反馈与工具速度前馈 |
| `bq_posture_policy` | 自动腰臂协调的姿态偏好与边界 |
| `bq_posture_reference_margin` | 为参考速度约束预留姿态边界余量 |
| `bq_shared_motion_weight` | 为腰等共享组增加运动代价，优先使用手臂 |

当前基础 ABI 版本为 1；这些能力以新增函数扩展，保留原结构布局和旧调用。只检查版本号不足以确认具备新功能，Python 还检查所需函数是否存在；当前雪铲配置启用了上述能力，应构建同一版本源码中的原生库与 Python 封装。

## 接入与验证

Python 调用入口为 [bindu_kinematics/bimanual.py](../bindu_kinematics/bindu_kinematics/bimanual.py)，对外函数声明见上表对应的 C 头文件。

- 构建、库加载与参数：[双臂 QP 参数与 ABI](#双臂qp参数与abi)。
- 原生测试：[test/test_core.cpp](test/test_core.cpp)，CMake 目标 `test_bimanual_core`。
- Python／ABI 检查：[test_bimanual_qp.py](../../../tests/unit/test_bimanual_qp.py)。

## 关联模块

[运动学封装](../bindu_kinematics/README.md) · [遥操作会话](../bindu_teleoperation/README.md) · [共享执行](../../control/bindu_execution/README.md)

## 运行与专项验证

以下命令和项目路径均相对于仓库根目录。带日期内容仅代表当时的专项实验，现行控制规则以动作执行契约和统一部署为准。

### 双臂QP参数与ABI

以下QP字段位于`bimanual_*.json`顶层，缺省值由原生`Config`提供，Python不另维护默认值：

| 参数 | 默认值 | 含义 |
| --- | ---: | --- |
| `tool_gain` / `relative_gain` | 2 / 2 | 工具跟踪／闭合误差恢复增益 |
| `posture_gain` / `posture_weight` | 0.3 / 0.001 | 捕获姿态回归增益／权重 |
| `shared_target_gain` / `shared_target_weight` | 3 / 0 | 独立共享关节姿态目标增益／权重 |
| `regularization` | 0.0001 | 速度正则项 |
| `shared_motion_weight` | 0 | 共享组额外的速度平方代价；越大越少使用共享关节，不设置固定腰角 |
| `max_linear_speed` / `max_angular_speed` | 0.15 / 0.5 | 工具目标线／角速度上限，m/s、rad/s |
| `max_relative_error` | 0.15 | 6D相对位姿误差范数门槛，平移与转角的混合范数 |
| `limit_margin` / `damper_gain` | 0.05 / 2 | 软限位余量／速度阻尼增益，余量采用关节自身单位 |
| `equality_tolerance` | 0.0005 | 速度闭合等式复核容差 |

所有字段必须有限；权重、姿态增益与margin可为零，其余必须为正；两侧margin之和必须小于对应有效关节范围。已有配置对速度和闭合误差设有更严格值，以上不是硬件标定值。可选`shared_targets`为共享组全部关节名到目标位置的映射，须配合正的`shared_target_weight`；未配置时腰部仍参与工具跟踪。Python `set_shared_targets(None)`清除该独立姿态目标。

当前位置契约的Snowbot配置将`shared_motion_weight`设为0.1，两轴腰部的速度正则二次项因此从0.0011增至0.1011；双臂不变。它是运动分配偏好，腰部仍可参与跟踪，普通双臂模式继续将腰部速度固定为0。配置不改变关节速度上限或握持约束。该字段通过新增的`bq_shared_motion_weight`函数在每次捕获后设置，C结构布局仍为ABI v1；旧库缺此能力时明确要求重建。

积分与保护字段：`max_integration_interval`缺省0.1 s且不可超过0.1 s；`max_reference_error`缺省0.15，按各关节位置单位检查参考与实测偏差，超过后要求停止并重新捕获；`sample_period`是离线采样步长，运行节点传入实际候选采样间隔。`solve_timeout_us`按墙钟约束单次候选求解，从输入校验、实测握姿准备到原生调用、限位／几何复核、结果打包和前馈样本准备；现有配置保留30 ms。返回的`solve_us`指标仍仅为QP内部累计计时，不能当作完整调用或指令往返延迟。worker往返另由`worker_timeout`约束，缺省为`solve_timeout_us/1e6 + 0.2`秒，初始化由`worker_startup_timeout`约束（缺省30秒）。这些门限不等于已验证真机延迟。

Python封装的`tracking_feedback`显式选择跟踪误差来源，适用于持工具与普通双臂；缺省`measured`保持现有位置跟踪，实测工具偏差会继续产生速度并推进已接纳关节参考。设置`"tracking_feedback": "reference"`时必须同时设置`"reference_kinematics": true`：受控关节的FK／Jacobian和任务误差都由已接纳参考计算，未控制关节仍使用实测上下文。目标固定且名义位姿已到达时，不因实测受力偏移继续追偿；尚未完成的名义运动及姿态偏好仍可推进，候选仍只在执行器接纳后提交。可选`tool_feedback`继续校验标定／差异边界，但在该模式不用于任务追踪。

名义模式保留实测关节／姿态限位、参考偏差、握持、路径碰撞和超时保护，以及整组参考领先量限制；允许的柔顺偏移仍受这些门槛约束，不自动放宽或重新捕获。现有纯位置配方保留缺省`measured`；选择名义模式不代表已接通阻抗执行，真实后端仍需统一名义参考、坐标与控制权／停止契约并由现场验收。

超时诊断为`BIMANUAL_QP_TIMEOUT:native|validation:wall_us=…:cpu_us=…:native_us=…:qp_us=…:geometry_us=…`：`native`检查覆盖输入准备至原生返回，`validation`检查覆盖后续复核、输出与前馈样本准备。`last_timing`保留同一阶段的墙钟、当前线程CPU及分段时间；墙钟明显高于线程CPU提示等待／调度，同线程GC则也消耗CPU，仍须结合GC或线程探针确认。超时使捕获失效，迟到候选不会返回成功或推进前馈样本；检查是调用返回后的拒绝门槛，不能抢占正在执行的原生函数，父进程另以worker期限处理卡住的子进程。

2026-10-14专项在保留原生函数与几何方法的对照中复现一次64.5 ms调用，其中同线程二代GC占62.1 ms；另一分段对照确认停顿发生在几何复核，QP内部仅12.3 µs。NumPy `ctypes.data_as()`转换留下的循环引用会促发扫描；现改为直接地址转换并显式保持数组生命周期。独立spawn的QP worker在初始化期限内先收集再冻结常驻模型／导入图，后续GC保持开启，父进程GC不变，正常退出显式释放原生句柄。完整测试中的旧超时没有分段探针，不能据此证明该次历史异常的唯一原因。对照、回归与失败日志见`artifacts/qp-timeout-2026-10-14/verification.json`；普通系统负载、GIL竞争与Orin长期延时仍需实测，这些改动不构成硬实时保证。

原生包安装C头文件`bindu_bimanual_qp/c_api.h`，当前ABI版本1。升级时须同时重建原生包和Python适配；旧库缺少`bq_abi_version()`会明确报告`BIMANUAL_ABI_MISSING`。原生头文件定义参数顺序、数组长度、返回码和所有权；空句柄返回失败，不支持重复销毁或任意悬空指针。Python求解须显式传入`reference`和`dt`，调用者只在接纳后推进参考；完整时序见[执行契约](../../../动作执行契约.md#2-输入模式必须显式选择)。
