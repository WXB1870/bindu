# bindu_teleoperation：遥操作输入与会话

[项目模块导航](../../../README.md#模块快速导航) · [职责与边界](#职责与边界) · [扫雪输入与握持](#扫雪输入与握持) · [PICO 按键方案](#pico-按键方案) · [源码导航](#源码导航) · [接入与验证](#接入与验证) · [关联模块](#关联模块)

处理 VR 输入、坐标映射、位姿滤波、单臂／双臂工具会话与模式切换。ROS 节点由运行装配模块提供。

## 职责与边界

- 把手柄位姿转为机器人坐标下的相对目标，维护握持、松握重接、暂停和重新对齐语义。
- 区分原始 VR 输入与已在机器人基座坐标系中的工具位姿输入，检查时间、序号与会话身份。
- 单臂委托 IK、双臂委托 QP；轨迹平滑和设备写入交给共享执行与设备层。
- 现场头显、网络与真实握持仍需联调；合成输入或本机 WebSocket 验证不代表现场 VR 已验收。

## 扫雪输入与握持

真机 VR 手柄采用相对映射：接合时记录手柄基准和当前实测工具姿态，后续把手柄的位置／旋转增量经坐标标定与比例映射，叠加到工具基准上。松握停止跟随，重新接合时重建基准。例如比例为 1 时，手柄沿标定后的前向移动 5 cm，工具目标相对接合姿态前移 5 cm。

输入链路为：**VR 手柄位姿 → 接合基准下的相对增量 → 机器人基座系工具目标 → QP**。这里的基座系工具目标是相对映射的计算结果；“绝对工具位姿”描述该结果的坐标表达。

当前扫雪装配默认 `tool_input_source=tool_pose`，在命名空间内订阅 `teleop/tool/input` 的 [ToolPoseInput](../../shared/bindu_interfaces/msg/ToolPoseInput.msg)。该接口接收已经映射好的目标 `pose` 与右手握持请求 `tight`；两轴腰部和双臂关节由 QP 自动分配。

| 输入 | 当前 Snowbot 约定 |
|---|---|
| `pose` / `frame_id` | 相对映射后的工具目标，以基座系 `car_base_link` 下的行优先 4×4 SE(3) 表示，平移单位米；也可由模拟输入生成器提供 |
| `tight=true` | 左右手均采用刚性握持约束 |
| `tight=false` | 左手持续紧握；右握点三维位置和杆轴方向保持约束，仅释放绕杆轴转动，不允许沿杆滑动 |
| `source_id` / `seq` / `stamp` / `runtime_mode` | 来源、顺序、时效与环境检查；`valid`、`stop` 表达输入有效性和停止 |

`tight`表达软件约束请求，实体夹爪指令及握持确认尚未接入；当前真实位置链收到`tight=false`会停止。换握先等待已接纳提交和实测停稳，再捕获当前握姿；绝对工具目标保持原坐标语义。

现有成对 VR 路径通过 `tool_input_source=vr` 消费 `VRInput`，由 `BimanualSession` 内部完成上述相对映射。在雪铲配置下，以两手位置中点的变化控制平移、左手柄的旋转变化控制姿态、右扳机请求虚握。`ToolPoseSession` 则消费上游映射好的工具目标；面向统一工具消息的现场 VR 转接仍待接入，头显与实体夹持尚未验收。操作与按键见[右手虚握说明](../../../运行与验证.md#vr-手柄控制右手虚握2026-10-09)。

模式选择使用 `teleop/select_mode`，工具模式名为 `bimanual_tool`；随后显式启动 `teleop/tool_session`。暂停／恢复使用 `teleop/tool_pause`：服务响应表示请求接纳，停稳确认由会话反馈给出；恢复需要请求之后的新输入。启动、准备握姿和完整流程从[扫雪运行入口](../../integration/bindu_runtime/README.md#扫雪遥操作)进入。

[真机最小位置链](../../../运行与验证.md#snowbot-真机最小遥操作)仍需显式管理上述Action，仅支持位姿／Squeeze／Y；其他实体操作待适配。

## PICO 按键方案

**程序状态：独立PICO配置已接入运动学仿真；实体夹爪、载荷／滑移确认和真实底层恢复尚未接入。** 统一称呼 **Squeeze（侧握键）** 和 **Trigger（食指扳机）**。本文完整维护按键及位姿映射；资格／执行规则见[动作执行契约6.1](../../../动作执行契约.md#61-snowbot-工具操作与握持切换目标契约)，使用顺序见[PICO扫雪操作流程](../../../运行与验证.md#pico-扫雪操作流程目标方案)。

### 实体按键映射

| 输入 | 操作 | 目标意图 |
|---|---|---|
| 左Trigger | 按住／有效释放 | 打开／闭合左爪，右爪不联动；仅已授权装卸有效 |
| 右Trigger | 按住／有效释放 | 打开／闭合右爪，左爪不联动；仅已授权装卸有效 |
| 双侧Squeeze | 两侧先各自释放，再同时握住1秒 | 接合相对跟随，捕获当前实测工具和VR基准 |
| 任一侧Squeeze | 释放 | 停止跟随，保持实体夹持；重新接合重建基准 |
| 左X | 按住／有效释放 | 请求右手绕杆虚握／恢复紧握；工具先停稳，实体模式确认及当前握姿捕获后才能继续 |
| 左Y | 新按下立即生效 | 软件停止并锁定跟随；不退出底层程序、不回位、不清零，保持夹持 |
| 右A | 按住1秒后持续按住 | 明确空手时双臂联合到准备位；松开停止，持铲或状态未知时不开放 |
| 右B | 新按下 | 确认面板当前一步，如支撑就绪、装夹完成、故障恢复；一次只消费一步，不直接开始跟随 |
| 左摇杆点击 | 新按下 | 请求装夹／卸载，停稳及支撑检查后由B确认；不直接开爪 |
| 右摇杆点击 | 新按下 | 启用记录时异步请求开始／停止，完全关闭记录器时无动作 |
| 两侧摇杆方向 | — | 首期预留，不直接控制底盘、腰或头；腰臂由QP分配 |
| Home／Menu／Capture等系统键（如有） | — | 保留系统用途，不依赖其送达应用来停止或准备机器人 |

常态保持已确认闭合，无需持续扣Trigger；虚握期间右爪保持经确认的受限夹持。首次接管／重连不因Trigger默认零值自动闭合，失联也不视为真实释放。模拟量初期采用按下`>0.7`、释放`<0.3`的迟滞区间，Squeeze接合1秒与准备A的1秒均为可配置初值；X使用有效按下／释放边沿。设备阈值和实体开合速度／力限制须标定，资格与中断处理由执行契约定义。

### 相对位姿映射

Snowbot雪铲配置下：双手柄中点增量控制工具平移，左手柄旋转增量控制工具姿态，右手柄旋转不额外驱动翻铲；刚握与虚握沿用同一输入映射。右手绕杆虚握只改变握持约束，不改变工具目标的坐标语义。

WebXR位姿先由[robot_pose](bindu_teleoperation/vr/mapping.py)统一到机器人轴向；配置的操作者朝向`operator_yaw_rad`再校正操作方向。对`robot_base`相对模式，设当前中点为`m`、左手旋转为`R_L`，接合基准分别为`m0`／`R_L0`，实际工具基准为`p0`／`R0`，方向标定为`C`、平移比例为`s`：

```text
p_target = p0 + s * C * (m - m0)
R_target = (C * R_L * R_L0^T * C^T) * R0
```

接合、换握或恢复后，从新鲜实测状态建立基准；离合退出后的手柄位移不累积，首个跟随目标对齐当前工具。输出以`car_base_link`下米制、行优先4×4 SE(3)传给`ToolPoseInput.pose`，握持请求传给`tight`；时效／身份字段照常检查。默认`tool_pose`路径消费上游已映射的目标，`vr`路径由`BimanualSession`内部完成映射，两者不重复叠加。旧单臂的`v34_anchor`组合语义保留，不在此改动。

### 协议字段与现有实现

| 物理输入／协议 | 当前代码 | 新PICO方案的落点 |
|---|---|---|
| Squeeze：`squeezeValue` | [receiver.py](bindu_teleoperation/vr/receiver.py)写入`VRFrame.grip`，ROS字段仍叫`grip` | 术语统一为Squeeze，保留兼容字段；不把侧握键误作夹爪开合 |
| Trigger：`triggerValue` | 写入`VRFrame.trigger`；旧`v34`配置保留右Trigger虚握 | PICO配置由协调器转成左右独立夹爪请求，`virtual_grip`单独传入会话，禁止一个输入触发两种动作 |
| 左X/Y、右A/B | 旧Vuer使用各侧`aButton/bButton`别名 | PICO的`buttons`位0／1／2表示各侧主键／副键／摇杆点击，保留可靠边沿；X虚握、Y停止、A联合准备、B单步确认，`init=false` |
| 摇杆点击与操作事件 | `button_edges`为主键／副键／摇杆／Trigger各自按下与释放的8个累计计数 | 已保留来源、时效、状态代次及边沿，合并位姿后仍可识别短按／释放；首次收到的历史计数不执行 |

PICO实体布局参考[官方OpenXR定义](https://github.com/Pico-Developer/PICO-Unity-SDK/blob/main/XR/Runtime/Scripts/OpenXRFeatures/Interactions/PICO4UltraControllerProfile.cs)，标准WebXR索引见[PICO 4 profile](https://github.com/immersive-web/webxr-input-profiles/blob/main/packages/registry/profiles/pico/pico-4.json)：Trigger／Squeeze／摇杆点击／两面键为0／1／3／4、5，摇杆轴为2、3。仅在设备声明匹配profile／mapping时使用，并以只读面板逐键核对；具体PICO型号尚未确认。

本机可核对旧例为`代码参考包/Bindu-历史代码精选-2026-09-16/03_遥操作与灵巧手/v3.4/`，独立v4尚未定位。`vr_main.py`实际分支是右B长按初始化左臂、右A长按初始化右臂、侧握跟随、左`bButton`退出；旧提示日志与分支不一致，以分支为准。当前双臂会话仍拒绝旧`init`；PICO映射需显式启用`pico_enabled`并选`vr`输入，默认装配不改变。协调器复用模式Action、工具Action和执行租约，夹爪／准备使用120 ms参考段、250 ms有效期；原始输入停止、协调器退出或松键都不能留下长动作队列。准备检查空手CAD路径，X切换在改变第8轴前检查含工具的CAD路径，完成依据来自模拟反馈。实体夹持、真实使能／保持及watchdog重新绑定没有由模拟结果开放。

### 程序接入

[PicoControl](bindu_teleoperation/pico.py)维护纯操作状态和代次；[PicoNode](../../integration/bindu_runtime/bindu_runtime/pico_node.py)负责ROS串接与执行权交接。[专用仿真配置](../../integration/bindu_runtime/config/deployments/teleop_pico_snowbot_sim.json)和[模拟第8轴参数](../../integration/bindu_runtime/config/robots/snowbot/pico_snowbot_sim.json)是显式入口；后者的开合／虚握角仅为运动学代理，不能用于实机。新节点拒绝真实及外部物理后端，详细启动、人工承载声明与验证命令见[运行流程](../../../运行与验证.md#pico-扫雪操作流程目标方案)。

`teleop/pico/state`区分操作阶段、阻断原因、模拟夹爪反馈及QP阶段，`physical_gripper_confirmed`保持`null`；`teleop/pico/context`需要当前协调器实例绑定的来源和generation，人工声明空手／持铲／外部承载，不伪造传感器证据。Squeeze接合及X切换通过真实工具Action进入QP；Y或断流取消动作并确认停稳，B只回待命。左右摇杆分别连装卸请求及`recorder/capture`。启动／重启持物状态固定未知，须重新声明；跟随／故障／换连接作废承载声明，装卸完成需要新的持物声明。协调器更换投影来源身份，避免继承旧工具会话；Vuer接收器在PICO配置下显示该状态面板。

## 源码导航

| 入口 | 内容 |
|---|---|
| [pico.py](bindu_teleoperation/pico.py) | PICO按键边沿、资格、装卸／虚握／停止恢复状态机 |
| [session.py](bindu_teleoperation/session.py) | 单臂相对控制会话 |
| [bimanual_session.py](bindu_teleoperation/bimanual_session.py) | BimanualSession（成对 VR 相对映射）、ToolPoseSession（映射后工具目标）、ToolPause 与 AcceptedReference |
| [modes.py](bindu_teleoperation/modes.py) | 手动模式选择与会话切换检查 |
| [config.py](bindu_teleoperation/config.py) | 输入映射与控制配置加载 |
| [vr/](bindu_teleoperation/vr/) | VR 接收、坐标映射、One Euro 滤波和显示 |
| [feedback.py](bindu_teleoperation/feedback.py) | 供显示使用的只读反馈 |

## 接入与验证

ROS 入口：[单臂节点](../../integration/bindu_runtime/bindu_runtime/teleop_node.py)、[工具节点](../../integration/bindu_runtime/bindu_runtime/bimanual_tool_node.py)、[VR 输入节点](../../integration/bindu_runtime/bindu_runtime/vr_input_node.py)。装配与模式配置从 [bindu_runtime](../../integration/bindu_runtime/README.md) 进入。

- 输入与依赖：[VR 遥操作运行入口](../../../运行与验证.md#vr-遥操作运行入口)。
- 模式与自由度：[手动选择本体与自由度](../../../运行与验证.md#手动选择本体与自由度)。
- 单元检查：[会话](../../../tests/test_teleoperation.py)、[模式](../../../tests/test_teleop_modes.py)、[One Euro](../../../tests/test_one_euro.py)。
- ROS 场景：[validate_teleop.py](../../../tests/validate_teleop.py)、[validate_bimanual.py](../../../tests/validate_bimanual.py)，依赖对应模拟装配。

## 关联模块

[运动学](../bindu_kinematics/README.md) · [公共输入契约](../../shared/bindu_contracts/README.md) · [共享执行](../../control/bindu_execution/README.md) · [运行装配](../../integration/bindu_runtime/README.md)
