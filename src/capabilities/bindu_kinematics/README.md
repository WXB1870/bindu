# bindu_kinematics：运动学与求解封装

[项目模块导航](../../../README.md#模块快速导航) · [编译与运行](#编译与运行) · [职责与边界](#职责与边界) · [扫雪求解与反馈](#扫雪求解与反馈) · [源码导航](#源码导航) · [接入与验证](#接入与验证) · [关联模块](#关联模块)

提供模型驱动的正运动学、单臂 IK、几何检查及双臂工具 QP 的 Python 接口。求解结果作为候选关节目标交给共享执行层。

## 编译与运行

在仓库根目录执行。先安装并加载 [数值依赖](../../integration/bindu_runtime/README.md#依赖准备)：FK／单臂 IK 需要 Pinocchio、NumPy／CasADi，双臂路径还需原生 QP 的 ProxSuite／Eigen3／SIMDe。Humble 环境将 `jazzy` 换为 `humble`，并在加载 ROS 后设置数值依赖路径。

```bash
source /opt/ros/jazzy/setup.bash
# 若使用 wheel，在此加载上述数值依赖环境。
colcon build --base-paths src --packages-up-to bindu_kinematics bindu_description --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE="$(command -v python3)" -DPYTHON_EXECUTABLE="$(command -v python3)"
source install/local_setup.bash
python3 - <<'PY'
import pinocchio as pin
import casadi
from bindu_kinematics.bimanual import BimanualQP
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
urdf = Path(get_package_share_directory('bindu_description')) / 'urdf/snowbot_control.urdf'
model = pin.buildModelFromUrdf(str(urdf))
print('model nq:', model.nq, 'QP wrapper:', BimanualQP.__name__)
PY
```

预期能加载 Snowbot 控制模型及求解封装；此检查不执行设备运动，也不代表完整 IK／QP 验收。本包没有独立 ROS 节点，持续求解由 [遥操作装配](../bindu_teleoperation/README.md#编译与运行) 启动工作进程；单独更新原生接口时同步重建 `bindu_bimanual_qp`。

## 职责与边界

- 从模型和配置读取链路、资源组与限制，避免在公共求解器中固定机器人型号。
- 单臂路径使用 Pinocchio／CasADi；双臂持工具通过原生 QP 联合分配腰部与双臂运动。
- 使用已接纳参考、实际关节反馈和显式周期维护连续性；求解器不直接写设备。
- 碰撞检查仅覆盖启用的代理或 CAD 范围。可选工具位姿观测不等于已实现真机六维力控制或上层柔顺控制。

## 扫雪求解与反馈

[雪铲配置](../../integration/bindu_runtime/config/robots/snowbot/bimanual_snowbot_shovel.json) 选择 `waist + left_arm + right_arm`，对应两轴腰部与双臂各七轴。控制链为工具目标 → `BimanualQP` 候选关节位置 → 共享执行器接纳；头部和两侧第八轴保留为上下文，不随工具目标自动控制。

- `reference_kinematics=true` 时，在已接纳参考处计算受控关节的 FK／Jacobian；`tracking_feedback`缺省使用实测跟踪，可显式选择`reference`生成名义运动，将保护范围内的受力偏移留给下层处理，配置语义见[QP参数](../bindu_bimanual_qp/README.md#双臂qp参数与abi)。保护仍检查实际姿态，参考只在执行器接纳后推进。
- `measured_grasp_policy` 默认 `enforce`，保留实测握持超差保护；仅在已声明未共同夹持实体工具的临时测试中选择 `unheld_test`。它只跳过实测握持几何误差判据，原生双臂耦合、实际关节限位、参考跟踪、碰撞和停止保护继续执行；接管事件记录所选策略。
- 工具速度前馈只使用已处理输入；换握／重捕获清空历史。姿态参考余量由 `posture_reference_margin_rad` 配置，实际姿态边界仍独立检查。
- 在线几何检查覆盖配置内的机器人、工具、地面与箱体，并沿候选关节段离散采样；它用于拒绝危险候选，不提供主动绕障或连续碰撞保证。
- 工具几何可配置`demo.shaft_contact_endpoint_tolerance_m`，默认0。接触区端点在该余量内越出模型杆身时，碰撞查询只豁免接触区与杆身的交集；杆长、杆径和其他身体部位的完整杆件检查保持。超过余量、无交集或非法数值仍报`INVALID_SHAFT_CONTACT_BAND`，不放宽工具／身体净距或目标有效期。

Isaac 物理实验在 [shovel_control.py](../../../tools/simulation/shovel_control.py) 中用左腕虚拟六维载荷估计握持变形，再把工具位姿观测交给 `solve(..., tool_feedback=...)`。工具真值只用于评分，右腕载荷目前用于记录和分析。该通路尚未接入 ROS 遥操作节点或真机六维传感器，也未实现双臂内力上限优化；上层没有额外叠加导纳／阻抗控制器。

## 源码导航

| 入口 | 内容 |
|---|---|
| [model.py](bindu_kinematics/model.py) | 本体运动学模型与正运动学 |
| [pinocchio_casadi.py](bindu_kinematics/pinocchio_casadi.py) / [worker.py](bindu_kinematics/worker.py) | 单臂 IK 与隔离工作进程 |
| [collision.py](bindu_kinematics/collision.py) | 简化碰撞几何与检查 |
| [bimanual.py](bindu_kinematics/bimanual.py) / [bimanual_worker.py](bindu_kinematics/bimanual_worker.py) | 双臂 QP 的 C ABI 封装与隔离工作进程 |
| [tool_geometry.py](bindu_kinematics/tool_geometry.py) | 工具／机器人 CAD 几何查询 |

双臂 worker 的 `worker_timeout` / `worker_startup_timeout` 是父进程收到结果的单调时间预算，包含队列和调度等待；不能解读为纯 QP 计算耗时。轮询先核对期限，再消费结果。超时、子进程退出或初始化致命错误锁存故障并撤销 ready，迟到结果不会恢复就绪或允许再次提交；恢复须显式替换 worker，并沿上层既有停止确认、反馈校验与重新捕获流程重入。期限内的普通求解拒绝仍作为本次操作错误交给调用方处理，不自动重试。纯内存时序回归见 [test_bimanual_worker.py](../../../tests/unit/test_bimanual_worker.py)，不启动 ROS 或求解进程。

工具会话生成候选时使用60%跟踪预算，提交复核仍用80%，并复核最新反馈、执行身份及租约。`session_v3`以同租约新鲜servo参考检查硬跟踪误差，积分仍从已接纳目标延续；制动中已有目标差只准保持／缩小，其他轴分别使用上述生成／提交余量。其他路径沿用目标跟踪判据，完整规则见[执行契约](../../../动作执行契约.md#2-输入模式必须显式选择)。不合格候选丢弃且不推进参考，连续丢弃不刷新TTL或原参考期限；诊断区分执行参考、目标差及逐轴候选预算。

工具会话等待`hold`停稳期间仍维护原租约续租，避免超过2秒租约有效期后失去同代停止凭证；仅原执行实例、原租约／代次、匹配profile／环境及新鲜健康反馈允许续租。续租不延长原停止期限，不发送运动目标；撤权、换实例、故障、反馈失鲜或停止未确认仍拒绝，确认停稳后再按既有流程撤权。

真实Snowbot配方的握持位置误差上限为60 mm、旋转上限0.05 rad（约2.86°），QP与几何校验总预算80 ms；杆体对机器人的最小净距仍为20 mm。2026-10-25按用户指定，左右link7／link8握持豁免半长从110增至150 mm；接触区中心沿用实测捕获定位，宽度与实体杆长取交集，区外净距及本体检查不变。实测关节越界容差单独设为2°，目标边界不扩，详见[执行契约](../../../动作执行契约.md#2-输入模式必须显式选择)。在线仅判通过／拒绝时，AABB下界达到对应门槛即可跳过精确距离查询；完整报告仍计算最小距离。预算也覆盖测量到旧参考的基线校验，未完成校验不会作为安全参考返回。

## 接入与验证

双臂配置入口：[Snowbot 雪铲](../../integration/bindu_runtime/config/robots/snowbot/bimanual_snowbot_shovel.json)。IK 与 QP 各有可选依赖；使用对应能力前，按 [运行与验证](../../../运行与验证.md) 安装并构建。

- 原生边界与参数：[双臂 QP 参数与 ABI](../bindu_bimanual_qp/README.md#双臂qp参数与abi)。
- 行为检查：[模型](../../../tests/unit/test_g1_model.py)、[IK 碰撞](../../../tests/unit/test_ik_collision.py)、[双臂 QP](../../../tests/unit/test_bimanual_qp.py)、[雪铲运动](../../../tests/unit/test_shovel_motion.py)。
- 物理闭环入口：[木质雪铲的 PhysX 受力实验](../../hardware/bindu_hardware/README.md#木质雪铲的-physx-受力实验2026-10-15)；该实验由仓库工具装配，不是本包的通用力传感器接口。

## 关联模块

[原生双臂 QP](../bindu_bimanual_qp/README.md) · [遥操作](../bindu_teleoperation/README.md) · [共享执行](../../control/bindu_execution/README.md) · [本体模型](../../hardware/bindu_description/README.md)

## 运行与专项验证

以下命令和项目路径均相对于仓库根目录。带日期内容仅代表当时的专项实验，现行控制规则以动作执行契约和统一部署为准。

### Snowbot 双臂与两轴腰部遥操作

三份 URDF 的用途、共享网格及生成方式见[模型与资产](../../hardware/bindu_description/README.md#机器人模型与资产)。控制模型的 20 个标量关节按 [`snowbot_sim.json`](../../integration/bindu_runtime/config/robots/snowbot/snowbot_sim.json) 命名。腰部是回转 `body_joint_1`、俯仰 `body_joint_2`，两臂各前 7 轴进入扫雪 QP；第 8 轴保留反馈但不由扫雪会话驱动。模型没有真实工具与雪面接触参数，当前 [`bimanual_snowbot_sim.json`](../../integration/bindu_runtime/config/robots/snowbot/bimanual_snowbot_sim.json) 的工具 frame 仅是左右力传感器中点的虚拟横杆。

构建需ROS 2 Jazzy、Pinocchio、Eigen3及ProxSuite CMake开发包；原生QP在`bindu_bimanual_qp`单独包中，未安装依赖时该可选包无法构建。单配置生成器未指定构建类型时默认Release，显式Debug/RelWithDebInfo保持原选择。完成构建并加载工作区后，可启动模拟节点：

```bash
ros2 launch bindu_runtime snowbot_sim.launch.py recording_mode:=normal vr_enabled:=true side:=left
```

`vr_enabled:=true`使用同一Vuer输入连接的左右手柄；现场连接仍需实际host/TLS/坐标核对。已有`VRInput`发布者时用`vr_enabled:=false`，避免同命名空间重复发布。启动装配包括执行、记录、具名TF、单臂IK、双臂QP及模式管理；底盘是运动学模拟，不会自动行走或启动真机。初始模式为单臂；后续GUI调用下列Action并订阅`teleop/mode_state`即可提交显式切换，示例中的代次1适用于新启动实例：

```bash
ros2 action send_goal /bindu_snowbot_sim/teleop/select_mode bindu_interfaces/action/SelectTeleopMode \
  "{mode: bimanual_tool, side: '', expected_generation: 1, timeout: 6.0}" --feedback
ros2 action send_goal /bindu_snowbot_sim/teleop/tool_session bindu_interfaces/action/TeleopSession \
  "{task_id: snowbot_demo, resource_group: bimanual_tool, duration: 30.0}" --feedback
```

切换须等待上一会话租约释放和新鲜静止反馈、记录器就绪、双手同源同序且松握的有效输入，以及模型/工具几何预检；成功后代次递增，需重新握持建立工具基准。切回单臂时用`mode: single_arm, side: left, expected_generation: 2`，然后向`teleop/session`提交`resource_group: left_arm`。旧代次、资源冲突、输入失效和取消停稳未确认会拒绝；模式不会随抓握自动切换。单臂配置可通过launch的`side:=right`选择右侧，但同一个实例只启动所选侧的单臂IK。

离线与ROS验收入口：

```bash
python3 -m unittest tests.test_snowbot_description tests.test_bimanual_qp tests.test_teleop_modes
python3 -m tests.integration.validate_bimanual --snowbot --output artifacts/snowbot-bimanual-check
python3 -m tests.integration.validate_bimanual --snowbot --switch-while-active --output artifacts/snowbot-active-switch
python3 -m tests.integration.validate_bimanual --snowbot --expect-geometry-rejection --output artifacts/snowbot-invalid-tool
```

第二条使用合成成对VR、运动学设备和真实ROS Action，检查腰部+双臂联合指令、取消、手动切单臂后IK跟随与再次取消；第三条额外检查单臂运动中手动切回工具模式前先停稳并撤销旧租约；第四条故意给出错误的工具握持间距，要求模式进入SAFE且零条运动指令。模式状态50ms发布，执行会话用独立0.5秒心跳门槛；关节反馈仍要求0.2秒内新鲜样本，二者不可混同。这些测试不会触发真实扫雪。10月7日规范复核时，旧验证脚本将合成输入发布与高频状态采集放在同一回调循环，三次完整回归分别因反馈、双手输入或模式心跳过期而安全结束；一次模式状态年龄为0.503秒。合成输入改由独立测试节点发布后，Snowbot完整闭环、运动中手动切换和无腰部组的通用双臂模型均通过，发布间隔最大约21毫秒。模式管理器保留最近完整成对帧；错误工具几何立即进入SAFE且零条命令。以上仍是无接触运动学批次，消息调度的长期稳定性尚未验收。真实工具握持、碰撞与雪面接触、轮地/关节物理、头显现场输入及实机两腰轴能力均未验收。G1可选双臂物理检查`tests/integration/validate_bimanual.py --g1-model artifacts/waist-yaw-2026-10-01/model`曾接纳联合命令，但取消时约0.02rad/s残余速度未达0.01rad/s停稳阈值，报告`STOP_FEEDBACK_TIMEOUT`，当前不标物理通过。
