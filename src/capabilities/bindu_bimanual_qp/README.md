# bindu_bimanual_qp：原生双臂工具 QP

[项目模块导航](../../../README.md#模块快速导航) · [职责与边界](#职责与边界) · [源码导航](#源码导航) · [扫雪接口与库版本](#扫雪接口与库版本) · [接入与验证](#接入与验证) · [关联模块](#关联模块)

基于 Pinocchio、ProxSuite 和 Eigen 的 C++17 联合求解库，输出 `libbindu_bimanual_qp.so`。通过 C ABI 为上层提供双臂与腰部持工具控制。

## 职责与边界

- 联合处理工具位姿跟踪、握持约束、关节限制、接近限位时的速度约束和姿态目标。
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

当前基础 ABI 版本为 1；这些能力以新增函数扩展，保留原结构布局和旧调用。只检查版本号不足以确认具备新功能，Python 还检查所需函数是否存在；当前雪铲配置启用了上述能力，应构建同一版本源码中的原生库与 Python 封装。

## 接入与验证

Python 调用入口为 [bindu_kinematics/bimanual.py](../bindu_kinematics/bindu_kinematics/bimanual.py)，对外函数声明见上表对应的 C 头文件。

- 构建、库加载与参数：[双臂 QP 参数与 ABI](../../../运行与验证.md#双臂qp参数与abi)。
- 原生测试：[test/test_core.cpp](test/test_core.cpp)，CMake 目标 `test_bimanual_core`。
- Python／ABI 检查：[test_bimanual_qp.py](../../../tests/test_bimanual_qp.py)。

## 关联模块

[运动学封装](../bindu_kinematics/README.md) · [遥操作会话](../bindu_teleoperation/README.md) · [共享执行](../../control/bindu_execution/README.md)
