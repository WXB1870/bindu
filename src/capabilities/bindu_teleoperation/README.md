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

**位置契约入口：** `snowbot_vr_sim/real.json`共用PICO协调器、模式Action、位置执行和夹爪服务。默认`independent_arms`普通双臂，B切换`bimanual_tool`扫雪；准备和跟随提交`joint_target`。A依赖现场准备位，X轴向虚握当前不可用。旧四节点配方仍采用原映射。启动与能力边界见[统一入口](../../../运行与验证.md#vr-统一部署与启动)。

位置契约主流程为：连接并获得有效反馈 → 原位待命 → 对应侧Squeeze先松再握 → 捕获该臂当前末端与手柄基准 → 独立相对跟随，不检查固定工具握点。扫雪模式才要求双侧同时握合及工具握持几何。两种模式共用一个执行租约，普通模式腰部保持；逐侧离合变更先确认整组保持并隔离旧结果，再保留未变侧锚点、只重捕获变化侧。模式切换、A准备和独立装卸完成都只回待命。确认与故障边界见[执行契约6.1](../../../动作执行契约.md#61-snowbot-工具操作与握持切换)。

**PICO准备／装卸已接入位置契约；实体载荷／滑移确认及真实底层恢复仍待验收。** 本文维护按键和相对映射；操作条件与状态图见[执行契约6.1](../../../动作执行契约.md#61-snowbot-工具操作与握持切换)，启动与使用见[运行流程](../../../运行与验证.md#pico-扫雪操作流程)。统一称呼 **Squeeze（侧握键）**、**Trigger（食指扳机）**。

### 普通／扫雪按键（位置契约入口）

左右输入固定对应左右臂／爪，A始终回准备位，Y始终停止；B在待命单击切模式，在明确的装卸或锁定状态确认当前一步。没有长短按B的隐藏分支。两种模式的摇杆方向仍预留底盘，头部跟随未接入。

| 输入 | 普通双臂模式 | 扫雪模式 | 边界 |
|---|---|---|---|
| 左／右手柄位姿 | 分别控制对应臂相对末端目标 | 双手中点平移、左手姿态控制共同工具 | 基准来自当前实测，普通模式腰部保持 |
| 左／右Squeeze | 各侧先松再握，松开该侧保持；另一侧可继续 | 两侧先释放、再同时握住；松任一侧暂停工具 | 初始已握住不算新接合，暂停移手不积累目标 |
| 左／右Trigger | 按住开对应爪、有效释放合；首期只在双臂停稳释放后的授权装卸中开放 | 同样控制对应爪，仅授权装卸有效；作业误扣先停，不排队开爪 | 独立位置接口已接，须标定、装卸声明及VR来源授权；工具会话内仍停止跟随 |
| 左X | 保留，无运动动作 | 按住请求右爪到虚握开度，释放回闭合位；先停稳、等开度到位／保持、重建基准 | 统一位置入口已接，开度可配置；位置反馈不证明实体防滑 |
| 左Y | 任意状态软件停止并锁定，取消待处理操作，保持现有夹持 | 相同 | 已接停止；不是硬件急停，不自动回位／开合 |
| 右A | 停稳松握后，长按1秒并持续按住回准备位；松开停止 | 相同，持物／未知需通过带工具路径检查 | 不要求空手声明；缺现场预设时明确拒绝 |
| 右B | 待命松握时单击请求切扫雪 | 同样条件下请求切普通；已确认持共同工具时拒绝 | 先检查目标模式，成功后仍待命；装卸中确认当前一步，锁定停稳后只恢复待命 |
| 两侧摇杆方向 | 预留底盘控制，本期无底盘动作 | 相同 | 未接；不映射到腰／头／模式选择 |
| 两侧摇杆点击 | 左侧单独请求装卸，右侧录制请求 | 相同 | 装卸需支撑确认；录制关闭时明确不可用 |
| 头显视角 | 后续跟随头部，本期只用于VR视图 | 相同 | 未接头部转动 |
| Home／Menu／Capture等系统键（如有） | 保留系统用途 | 相同 | 不依赖系统键送达应用来停止或准备 |

**A回装夹初始位。** 用户要求的init pose是任务预设姿态，用于把雪铲放到双手的装夹位置；首期建议A在两模式指向同一预设，避免切模式后改变回位含义。它不承担解锁或通用确认。按A从新鲜实测状态平滑返回，到位并停稳后保持；松A、Y或输入失效停止。到位后再按对应Trigger开爪、放入雪铲、有效释放闭爪，返回初始位本身不改变夹爪。预设来源、控制范围及持铲返回边界见[init pose定义](../../../软件架构设计.md#832-snowbot-实机启动与准备建议流程)。

**B切模式与当前步骤确认。** 待命时只接受停稳、双侧松握后的新单击；跟随中按B只停止，不补执行切换，须松握后重新按。目标模式检查失败保留原模式；成功后保持姿态和夹爪、回待命并重新接合，不触发A。装卸中的B只确认支撑或完成，锁定后的B只恢复待命，同一次按键不跨状态连过。具体交接见[双模式实现](../../../软件架构设计.md#834-snowbot普通双臂与扫雪模式切换待实现建议)。

**待现场验收。** 普通双臂、A配置化准备和B模式交接已共用位置契约；现场准备位、夹爪标定、实际保持／跟踪仍须实测。X按用户确认先采用右爪轻微张开的可调位置，参数与流程见[夹爪位置闭环](../../../运行与验证.md#snowbot真实夹爪位置闭环)；当前`grasp_state=unknown`不证明夹稳或防滑。普通模式边移动边开合不在本次实现范围。

常态不持续扣Trigger，也不因切换模式自动开合。模拟量采用按下`>0.7`、释放`<0.3`的迟滞区间，现有模拟接合／准备的1秒为配置初值；实体阈值、开合速度与力限制须标定。首次接管、失联及操作仲裁统一见执行契约。

头部后续可跟随VR视角，当前不接入。Snowbot真实夹爪位置闭环通过独立的`gripper/lease`、`gripper/control`及`gripper/state`接入，命令由现有Snowbot执行节点下发。经现场标定且双臂已停止／释放控制权后，显式授权装卸并绑定VR来源，Trigger才控制对应夹爪；作业会话中的Trigger仍先停止工具跟随，不自动获得装卸资格。旧四节点入口不启用PICO协调器；统一位置入口已通过协调器接入装卸、配置化A准备和X位置虚握。位置到位、软件开合状态与真实夹持分别判断，`grasp_state`当前始终为`unknown`；命令、底层保护与标定步骤见[真实夹爪位置闭环](../../../运行与验证.md#snowbot真实夹爪位置闭环)。

### 相对位姿映射

标准Snowbot雪铲配方的平移取双手柄中点增量，姿态取左手柄旋转增量；刚握与虚握使用同一映射。简化PICO运动学仿真采用下述双手短棍映射。

WebXR位姿先由[robot_pose](bindu_teleoperation/vr/mapping.py)统一到机器人轴向；配置的操作者朝向`operator_yaw_rad`再校正操作方向。对`robot_base`相对模式，设当前中点为`m`、左手旋转为`R_L`，接合基准分别为`m0`／`R_L0`，实际工具基准为`p0`／`R0`，方向标定为`C`、平移比例为`s`：

```text
p_target = p0 + s * C * (m - m0)
R_target = (C * R_L * R_L0^T * C^T) * R0
```

接合、换握或恢复后重建基准；松握期间的位移不累积，首个目标对齐当前工具。目标采用上表的基座系SE(3)表达；旧单臂`v34_anchor`组合语义保留。

**短棍简化仿真**：`pico_simulation_demo=true`通过[simulation_fixture.py](bindu_teleoperation/simulation_fixture.py)从同一握点配置生成长约0.56m、握点间距0.34m的短棍，工具原点移到双握点中点。QP、CAD碰撞、Vuer和Meshcat共用该变换；没有隐藏铲头或接雪箱碰撞。原长铲几何的坐标原点或长度改变，不改变短棍的握点与控制杠杆。

- 平移仍为上式，`s=0.5`，三个方向等比例；两手共同平移10cm产生5cm中点目标位移。间距变化不会拉长机器人短棍，绕轴角不乘平移比例。
- 棍轴方向取两手柄连线相对接合连线的转动。先剥离连线转向，再分别从两手柄相对各自接合姿态的旋转中提取轴向分量，连续展开角度后等权平均；同向等量转动产生等量自转，单侧转动贡献一半，反向等量抵消。不平均左右手柄的绝对姿态。
- 接合时把人手连线标定到机器人初始棍轴，连线的相对转向通过该固定标定传递；共同转角最终绕**机器人短棍的当前轴**施加。因此初始两棍轴不平行也不会丢失连线转向或把自转变成倾斜。初始腕姿不同及共同转动跨±π由回归覆盖。
- 接合期间不发运动目标；异步实测工具捕获完成后，同步重锁最新双手输入基准，首目标等于实测工具。松握先停止并等待停稳，隔离旧工作结果；暂停移手不积累目标，再握重新锁存平移、连线和两侧转角基准。Y锁定后，输入恢复或持续握住均不能自动续跑；松握后B恢复待命。
- 采样间隔与积分步长分别配置：Snowbot统一位置入口允许200ms采样间隔，积分步长仍最多100ms，输入／反馈有效期仍为250／200ms；模拟与实机接口共用。超过边界或时间倒退停止，实际间隔与积分值写入事件。旧演示最多250ms；配置不得超过输入有效期。客户端、接收与求解诊断见[运行日志](../../../运行与验证.md#vr-统一部署与启动)。

轴向转换与[OpenTeleVision的基变换](https://github.com/OpenTeleVision/TeleVision/blob/main/teleop/constants_vuer.py)、[Unitree TeleVuer](https://github.com/unitreerobotics/televuer/blob/main/src/televuer/tv_wrapper.py)一致：XR右／上／前对应机器人右／上／前。Unitree另支持头部yaw参考；本相对工具会话使用固定基座方向，头部转动不改变已接合映射。重接合的位置与姿态重锁、仅缩放平移也与[NVIDIA Isaac Capture的clutch设计](https://nvidia.github.io/IsaacCapture/main/references/retargeting/so101.html)一致；其5自由度机械臂的姿态保持取舍不直接套用于本双臂工具。

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
| [independent_session.py](bindu_teleoperation/independent_session.py) | 普通双臂逐侧离合、相对目标与暂停侧重捕获 |
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
