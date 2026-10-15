# bindu_kinematics：运动学与求解封装

[项目模块导航](../../../README.md#模块快速导航) · [职责与边界](#职责与边界) · [源码导航](#源码导航) · [接入与验证](#接入与验证) · [关联模块](#关联模块)

提供模型驱动的正运动学、单臂 IK、几何检查及双臂工具 QP 的 Python 接口。求解结果作为候选关节目标交给共享执行层。

## 职责与边界

- 从模型和配置读取链路、资源组与限制，避免在公共求解器中固定机器人型号。
- 单臂路径使用 Pinocchio／CasADi；双臂持工具通过原生 QP 联合分配腰部与双臂运动。
- 使用已接纳参考、实际关节反馈和显式周期维护连续性；求解器不直接写设备。
- 碰撞检查仅覆盖启用的代理或 CAD 范围。可选工具位姿观测不等于已实现真机六维力控制或上层柔顺控制。

## 源码导航

| 入口 | 内容 |
|---|---|
| [model.py](bindu_kinematics/model.py) | 本体运动学模型与正运动学 |
| [pinocchio_casadi.py](bindu_kinematics/pinocchio_casadi.py) / [worker.py](bindu_kinematics/worker.py) | 单臂 IK 与隔离工作进程 |
| [collision.py](bindu_kinematics/collision.py) | 简化碰撞几何与检查 |
| [bimanual.py](bindu_kinematics/bimanual.py) / [bimanual_worker.py](bindu_kinematics/bimanual_worker.py) | 双臂 QP 的 C ABI 封装与隔离工作进程 |
| [tool_geometry.py](bindu_kinematics/tool_geometry.py) | 工具／机器人 CAD 几何查询 |

## 接入与验证

双臂配置入口：[Snowbot 雪铲](../../integration/bindu_runtime/config/bimanual_snowbot_shovel.json)。IK 与 QP 各有可选依赖；使用对应能力前，按 [运行与验证](../../../运行与验证.md) 安装并构建。

- 原生边界与参数：[双臂 QP 参数与 ABI](../../../运行与验证.md#双臂qp参数与abi)。
- 行为检查：[模型](../../../tests/test_g1_model.py)、[IK 碰撞](../../../tests/test_ik_collision.py)、[双臂 QP](../../../tests/test_bimanual_qp.py)、[雪铲运动](../../../tests/test_shovel_motion.py)。
- 物理闭环入口：[木质雪铲的 PhysX 受力实验](../../../运行与验证.md#木质雪铲的-physx-受力实验2026-10-15)；该实验由仓库工具装配，不是本包的通用力传感器接口。

## 关联模块

[原生双臂 QP](../bindu_bimanual_qp/README.md) · [遥操作](../bindu_teleoperation/README.md) · [共享执行](../../control/bindu_execution/README.md) · [本体模型](../../hardware/bindu_description/README.md)
