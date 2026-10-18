# bindu_teleoperation：遥操作输入与会话

[项目模块导航](../../../README.md#模块快速导航) · [职责与边界](#职责与边界) · [扫雪输入与握持](#扫雪输入与握持) · [PICO 按键方案](#pico-按键方案) · [源码导航](#源码导航) · [接入与验证](#接入与验证) · [关联模块](#关联模块)

处理 VR 输入、坐标映射、位姿滤波、单臂／双臂工具会话与模式切换。ROS 节点由运行装配模块提供。

## 职责与边界

- 把手柄位姿转为机器人坐标下的相对目标，维护握持、松握重接、暂停和重新对齐语义。
- 区分原始 VR 输入与已在机器人基座坐标系中的工具位姿输入，检查时间、序号与会话身份。
- 单臂委托 IK、双臂委托 QP；轨迹平滑和设备写入交给共享执行与设备层。
- 现场头显、网络与真实握持仍需联调；合成输入或本机 WebSocket 验证不代表现场 VR 已验收。

首次连接实体头显使用[独立连通检查](../../../运行与验证.md#vr-首次连通检查)：只采集双手柄并显示位姿、按键和断流，不启动机器人控制；通过后再进入仿真遥操作。

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

**独立PICO运动学仿真已接入；真实夹爪另有独立位置闭环，完整实体装卸、载荷／滑移确认及真实底层恢复仍待接入。** 本文维护按键和相对映射；操作条件与状态图见[执行契约6.1](../../../动作执行契约.md#61-snowbot-工具操作与握持切换目标契约)，启动与使用见[运行流程](../../../运行与验证.md#pico-扫雪操作流程目标方案)。统一称呼 **Squeeze（侧握键）**、**Trigger（食指扳机）**。

### 普通／扫雪统一按键规划（2026-10-17，建议，未整体实现）

用户已确认双臂独立＋夹爪、两模式摇杆控制底盘但暂不接入、头部后续随VR但暂不接入。下表是本轮统一建议，A／B和菜单细节尚未获用户逐项确认，不代表更改了现有真机按键行为。位姿管运动、Squeeze管离合、Trigger管对应夹爪；模式、装卸和恢复集中进入可见操作面板，不靠摇杆代管。

| 输入 | 普通双臂模式 | 扫雪模式 | 当前真机实现 |
|---|---|---|---|
| 左／右手柄位姿 | 分别控制对应臂相对末端目标 | 双手中点平移、左手姿态控制共同工具 | 已接扫雪相对工具；普通双臂协调待接 |
| 左／右Squeeze | 各管对应臂离合，松开侧保持；重新接合捕获该侧新基准 | 两侧先释放、再同时握住接合；松任一侧暂停共同工具 | 扫雪已接；普通逐侧离合待接 |
| 左／右Trigger | 按住开对应爪、有效释放合；首期只在双臂停稳释放后的授权装卸中开放 | 同样控制对应爪，仅授权装卸有效；作业误扣先停，不排队开爪 | 独立位置接口已接，须标定、装卸声明及VR来源授权；工具会话内仍停止跟随 |
| 左X | 面板关闭时保留，无运动动作 | 按住请求右手轴向虚握，有效释放回锁；先停稳、实际确认、重建基准 | 真实虚握未接，当前按X停止工具会话；模拟换握已接 |
| 左Y | 任意状态软件停止并锁定，取消待处理操作，保持现有夹持 | 相同 | 已接停止；不是硬件急停，不自动回位／开合 |
| 右A | 面板关闭且明确空手时，按住1秒后持续按住，执行该模式已验证的双臂准备；松开停止 | 同样原则，准备构型按工具任务配置 | 仅模拟准备已接；真机按A停止工具会话，不回准备位 |
| 右B | 新按下请求打开操作面板；已有明确待确认项时，只确认当前一步 | 相同 | 仅模拟当前步骤确认已接；真机按B停止工具会话，不确认夹持／模式 |
| 两侧摇杆方向 | 预留底盘控制，本期无底盘动作 | 相同 | 未接；不映射到腰／头／模式选择 |
| 两侧摇杆点击 | 本轮建议保留，不分配装卸或录制；录制以后通过面板扩展 | 相同 | 真机点击仍停止活动工具会话；旧模拟保留左装卸、右录制，尚未迁移 |
| 头显视角 | 后续跟随头部，本期只用于VR视图 | 相同 | 未接头部转动 |
| Home／Menu／Capture等系统键（如有） | 保留系统用途 | 相同 | 不依赖系统键送达应用来停止或准备 |

**B面板交互建议。** 跟随中按B先暂停，确认实测停稳并释放两侧Squeeze后才开放面板；未确认停止则进入故障状态。面板中X上一项、A下一项、B确认，均取新按下，摇杆不参与。选项限于普通／扫雪模式、请求装卸／完成装卸、持物与外部支撑声明、恢复待命、关闭面板并保持。每次打开和完成一个步骤后清空选择；B不能连按跳过多项条件，恢复只回待命，不自动跟随。普通切扫雪需装夹和工具条件有效；持铲时禁止切独立臂。具体交接规则见[双模式方案](../../../软件架构设计.md#834-snowbot普通双臂与扫雪模式切换待实现建议)。

**状态决定路由。** 面板打开时A／X只导航，不执行准备或虚握，Trigger和Squeeze不启动运动；面板关闭后所有运动键须重新观察释放再接受新操作，不把菜单期间的按住沿用到运动状态。面板关闭时A只用于准备、X仅用于扫雪换握；Y始终优先，其他冲突输入拒绝并给出原因。原模拟左摇杆装卸／右摇杆录制是现有行为，不再作为统一方案的目标映射。

**接入顺序。** 本轮已完成独立夹爪位置闭环；下一阶段先统一普通双臂／扫雪的协调与B操作面板、逐侧离合及控制权交接，再接经验证的A准备。X实体虚握在夹持、承载和轴向滑移条件可验证后开放。不要把模拟A／B直接移到真机：A缺实机准备路径和保持验收，B缺真实操作状态及证据；当前`grasp_state=unknown`不能充当B完成持铲确认的依据。普通模式边移动边开合不在本次夹爪接口的已实现范围。

常态不持续扣Trigger，也不因切换模式自动开合。模拟量采用按下`>0.7`、释放`<0.3`的迟滞区间，现有模拟接合／准备的1秒为配置初值；实体阈值、开合速度与力限制须标定。首次接管、失联及操作仲裁统一见执行契约。

头部后续可跟随VR视角，当前不接入。Snowbot真实夹爪位置闭环通过独立的`gripper/lease`、`gripper/control`及`gripper/state`接入，命令由现有Snowbot执行节点下发。经现场标定且双臂已停止／释放控制权后，显式授权装卸并绑定VR来源，Trigger才控制对应夹爪；作业会话中的Trigger仍先停止工具跟随，不自动获得装卸资格。该入口不启用完整模拟PICO协调器，也不执行X虚握或A准备。位置到位、软件开合状态与真实夹持分别判断，`grasp_state`当前始终为`unknown`；命令、底层保护与标定步骤见[真实夹爪位置闭环](../../../运行与验证.md#snowbot真实夹爪位置闭环)。

### 相对位姿映射

Snowbot雪铲的平移取双手柄中点增量，姿态取左手柄旋转增量，右手转腕不额外驱动翻铲；刚握与虚握使用同一映射。

WebXR位姿先由[robot_pose](bindu_teleoperation/vr/mapping.py)统一到机器人轴向；配置的操作者朝向`operator_yaw_rad`再校正操作方向。对`robot_base`相对模式，设当前中点为`m`、左手旋转为`R_L`，接合基准分别为`m0`／`R_L0`，实际工具基准为`p0`／`R0`，方向标定为`C`、平移比例为`s`：

```text
p_target = p0 + s * C * (m - m0)
R_target = (C * R_L * R_L0^T * C^T) * R0
```

接合、换握或恢复后重建基准；松握期间的位移不累积，首个目标对齐当前工具。目标采用上表的基座系SE(3)表达；旧单臂`v34_anchor`组合语义保留。

### 协议字段与现有实现

| 物理输入／协议 | 代码字段与兼容边界 |
|---|---|
| Squeeze：`squeezeValue` | [receiver.py](bindu_teleoperation/vr/receiver.py)写入兼容字段`grip`，不用于夹爪开合 |
| Trigger：`triggerValue` | `trigger`控制左右独立夹爪；PICO的`virtual_grip`单独传入会话，避免旧右Trigger虚握双重触发 |
| 左X/Y、右A/B及摇杆点击 | Vuer面键别名为`aButton/bButton`；各侧`buttons`位0／1／2为主键／副键／摇杆，PICO固定`init=false` |
| 短按／释放 | `button_edges`为主键／副键／摇杆／Trigger按下和释放的8个累计计数；Squeeze另用`clutch_seq`。合并位姿不丢边沿，首次历史计数不执行 |

PICO实体布局参考[官方OpenXR定义](https://github.com/Pico-Developer/PICO-Unity-SDK/blob/main/XR/Runtime/Scripts/OpenXRFeatures/Interactions/PICO4UltraControllerProfile.cs)，标准WebXR索引见[PICO 4 profile](https://github.com/immersive-web/webxr-input-profiles/blob/main/packages/registry/profiles/pico/pico-4.json)：Trigger／Squeeze／摇杆点击／两面键为0／1／3／4、5，摇杆轴为2、3。仅在设备声明匹配profile／mapping时使用，并以只读面板逐键核对。2026-10-18已确认现场设备为PICO 4并收到真实双手柄位姿、面键、摇杆点击与扳机输入；PICO OS版本、完整逐键方向／侧握和机器人跟随仍需验证。

本机旧例位于`代码参考包/Bindu-历史代码精选-2026-09-16/03_遥操作与灵巧手/v3.4/`，独立v4尚未定位。`vr_main.py`实际为右B初始化左臂、右A初始化右臂、侧握跟随、左`bButton`退出，旧提示与分支不一致。PICO须显式启用，双臂会话继续拒绝旧`init`。

### 程序接入

[PicoControl](bindu_teleoperation/pico.py)维护操作状态与代次；[PicoNode](../../integration/bindu_runtime/bindu_runtime/pico_node.py)串接模式／工具Action和执行租约。[专用配置](../../integration/bindu_runtime/config/deployments/teleop_pico_snowbot_sim.json)显式启用`pico_enabled=true`、`tool_input_source=vr`；[第8轴参数](../../integration/bindu_runtime/config/robots/snowbot/pico_snowbot_sim.json)仅作模拟代理。状态面板、人工持物声明和验证命令统一见运行流程。

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
