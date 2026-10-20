# bindu_planning：候选关节计划模拟策略

[项目模块导航](../../../README.md#模块快速导航) · [编译与运行](#编译与运行) · [职责与边界](#职责与边界) · [源码导航](#源码导航) · [接入与验证](#接入与验证) · [关联模块](#关联模块)

目前为示例任务生成单终点关节计划，用于验证策略替换与共享执行路径。尚未集成通用抓取规划或环境避障规划器。

## 编译与运行

在仓库根目录执行；需要 ROS 2、colcon 及 [基础依赖](../../../运行与验证.md#环境准备与构建)。Humble 环境将 `jazzy` 换为 `humble`。

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-up-to bindu_planning --executor sequential
source install/local_setup.bash
python3 - <<'PY'
import asyncio
from bindu_contracts.profile import Profile
from bindu_planning.simulated import PlannerStrategy
profile = Profile.load('src/integration/bindu_runtime/config/simulation/wheel_sim.json')
plan = asyncio.run(PlannerStrategy().plan(profile, 'arm', (0.0, 0.0), (0.1, 0.1)))
print(plan)
PY
```

上例只生成 `JointPlan`，不提交设备命令。本包没有独立 ROS 节点；要运行计划到模拟执行的完整流程，按 [任务模块启动命令](../../tasks/bindu_tasks/README.md#编译与运行) 启动 `skeleton.launch.py`，并在任务请求中使用 `strategy: planner`。

## 职责与边界

- 实现公共 `GraspStrategy` 端口，依据本体资源组和目标关节值生成 `JointPlan`。
- 根据配置中的速度、加速度和加加速度限制估计计划时长；输出仍需执行层校验。
- 不执行 IK、环境碰撞搜索或设备写入。单臂 IK 与双臂 QP 位于运动学模块。

## 源码导航

| 入口 | 内容 |
|---|---|
| [simulated.py](bindu_planning/simulated.py) | PlannerStrategy 与候选计划生成 |
| [公共策略端口](../../shared/bindu_contracts/bindu_contracts/ports.py) | GraspStrategy.plan 的输入输出 |
| [计划数据契约](../../shared/bindu_contracts/bindu_contracts/contracts.py) | JointPlan 等共享类型 |

## 接入与验证

替换策略时保持关节命名、顺序、资源组和时间语义，由 [plugins.py](../../integration/bindu_runtime/bindu_runtime/plugins.py) 等装配入口选择实现。计划生成成功不代表设备已执行或抓取成功。

- 行为检查：[模拟能力](../../../tests/unit/test_capabilities.py)、[系统契约](../../../tests/unit/test_system_contracts.py)。
- 示例任务 ROS 检查：[validate_ros.py](../../../tests/integration/validate_ros.py)。
- 执行规则：[动作执行契约](../../../动作执行契约.md)；环境与验证命令：[运行与验证](../../../运行与验证.md)。

## 关联模块

[公共契约](../../shared/bindu_contracts/README.md) · [运动学](../bindu_kinematics/README.md) · [共享执行](../../control/bindu_execution/README.md) · [任务编排](../../tasks/bindu_tasks/README.md)
