# bindu_hardware：设备适配与反馈汇总

[项目模块导航](../../../README.md#模块快速导航) · [编译与运行](#编译与运行) · [职责与边界](#职责与边界) · [Snowbot 位置接入](#snowbot-位置接入) · [源码导航](#源码导航) · [接入与验证](#接入与验证) · [关联模块](#关联模块)

把公共设备端口接到模拟驱动、外部物理仿真或底层接口，并向执行层提供统一反馈。

## 编译与运行

在仓库根目录执行，先准备 ROS 2、colcon 及 [基础依赖](../../../运行与验证.md#环境准备与构建)；Humble 环境将 `jazzy` 换为 `humble`。

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-up-to bindu_hardware --executor sequential
source install/local_setup.bash
python3 - <<'PY'
from bindu_hardware.drivers.simulated_joints import SimJointDriver
driver = SimJointDriver(['demo_joint'], 1.0)
driver.write_positions({'demo_joint': 0.1})
print(driver.read(1.0, 0.01))
PY
```

上例只驱动内存中的模拟关节。设备库没有本包内的 ROS 可执行程序；要运行 Snowbot 模拟反馈链，先准备 [运行装配依赖](../../integration/bindu_runtime/README.md#编译与运行)，再启动：

```bash
colcon build --base-paths src --packages-up-to bindu_runtime --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE="$(command -v python3)" -DPYTHON_EXECUTABLE="$(command -v python3)"
source install/local_setup.bash
ros2 launch bindu_runtime snowbot_adapter.launch.py \
  runtime_mode:=simulation recording_mode:=normal
```

其他已加载相同环境的终端可执行 `ros2 topic echo /bindu_snowbot/execution/state --once`；Ctrl+C 收尾。真实位置接入需要另行构建厂商消息、配置路由并核对新鲜反馈，按 [现场部署顺序](../../integration/bindu_runtime/README.md#snowbot-真机最小遥操作) 操作，默认只读。

## 职责与边界

- 分开维护设备驱动、命令路由和反馈汇总；`RobotIO` 负责组合这些组件。
- 保留反馈来源、源时间、仿真标记与未知速度，汇总停止失败，支持底盘里程计和关节位置映射。
- Snowbot 底层位置适配与二代 SOEM 只读反馈分别维护；配置映射需要与实机关节顺序和单位一致。
- 模拟驱动用于协议和行为验证，不代表物理仿真；真实设备运动与标定仍需现场验收。

## Snowbot 位置接入

[snowbot.py](bindu_hardware/snowbot.py) 接收现场双臂及夹爪的16轴实测反馈，也兼容扩展头腰反馈；缺失轴不补零、不引用目标值或保留旧测量冒充新帧。`execution/state.missing_feedback_groups`明确列出缺失组，`joints`与反馈源时间仅含实际收到的数据。20轴profile继续保留，完整本体状态与腰臂协调不因16轴接收成功而视为已验收。底盘未接入。

独立[snowbot_gripper.py](bindu_hardware/snowbot_gripper.py)提供夹爪位置闭环、装卸授权、到位与保持确认，默认标定门槛禁止夹爪写入。通过健康、路由和控制权检查后才允许写位置；现有底层缺口与当前适配边界集中见[底层驱动问题记录](../../../../../snowbot/底层驱动问题记录.md)。

该位置入口默认只读，已供[真机最小遥操作](../../integration/bindu_runtime/README.md#snowbot-真机最小遥操作)复用；独立夹爪开合见[真实夹爪位置闭环](#snowbot真实夹爪位置闭环)，实体夹持及虚握确认仍待接入。底层已有柔顺；六维力传感器的坐标、去皮和接口仍需接入与标定。Isaac 空铲实验的载荷通路由仓库级工具单独实现，不能把本包外部仿真驱动视为现成的 Snowbot 六维传感器接口。接入状态和运行步骤见[Snowbot 位置控制适配层](#snowbot-位置控制适配层)。

已盘点的底层源码包含启动回位／清零，看门狗不支持新实例接管旧会话；现场待命及夹爪接口按用户要求先假定可用，正式测试前核实。适配器只读不能阻止底层自身启动动作。默认待命、受控重新绑定与持载重启边界统一见[实机启动方案](../../../软件架构设计.md#832-snowbot-实机启动与准备)。夹爪位置接口已有独立写入路径，实际握持确认仍未接入，判据见[工具操作契约](../../../动作执行契约.md#61-snowbot-工具操作与握持切换)。

## 源码导航

| 入口 | 内容 |
|---|---|
| [robot_io/system.py](bindu_hardware/robot_io/system.py) | RobotIO 组合与 ExecutionIO 实现 |
| [robot_io/routing.py](bindu_hardware/robot_io/routing.py) / [feedback.py](bindu_hardware/robot_io/feedback.py) | 命令分发／反馈汇总 |
| [robot_io/odometry.py](bindu_hardware/robot_io/odometry.py) | 轮式底盘里程计 |
| [drivers/](bindu_hardware/drivers) | 模拟关节、手、底盘及外部物理仿真驱动 |
| [snowbot.py](bindu_hardware/snowbot.py) / [position_mapping.py](bindu_hardware/position_mapping.py) | 底层位置接口与配置映射 |
| [soem_feedback.py](bindu_hardware/soem_feedback.py) | 二代 SOEM 反馈转换 |

## 接入与验证

设备选择在 [device_assembly.py](../../integration/bindu_runtime/bindu_runtime/device_assembly.py) 完成。Snowbot ROS 适配入口为 [snowbot_adapter.py](../../integration/bindu_runtime/bindu_runtime/snowbot_adapter.py)，二代反馈入口为 [soem_joint_feedback.py](../../integration/bindu_runtime/bindu_runtime/soem_joint_feedback.py)。

- 配置示例：[Snowbot 底层位置](../../integration/bindu_runtime/config/robots/snowbot/snowbot_vendor_position.json)、[二代位置映射](../../integration/bindu_runtime/config/robots/wheel_gen2/position_mapping_wheel_gen2.json)。
- 行为检查：[物理反馈](../../../tests/unit/test_physics.py)、[里程计](../../../tests/unit/test_odometry.py)、[Snowbot 适配](../../../tests/unit/test_snowbot_adapter.py)。
- 接入步骤和限制：[Snowbot 位置控制适配层](#snowbot-位置控制适配层)。

## 关联模块

[公共契约](../../shared/bindu_contracts/README.md) · [共享执行](../../control/bindu_execution/README.md) · [本体模型](../bindu_description/README.md) · [运行装配](../../integration/bindu_runtime/README.md)

## 运行与专项验证

以下命令和项目路径均相对于仓库根目录。带日期内容仅代表当时的专项实验，现行控制规则以动作执行契约和统一部署为准。

### Isaac Sim 图形物理仿真

`g1_sim.launch.py`默认仍为运动学模拟。可选`device_backend:=external_simulation`通过独立ROS设备桥连接Isaac Sim：PhysX负责重力、轮地接触和驱动响应，执行器读取实际关节位置/速度及底盘位姿，不把下发目标当反馈。当前仅在本机Isaac Sim **6.0.0-rc.22**、Ubuntu24.04/Jazzy、RTX4090测试；脚本使用该RC版本的导入API，其他版本须另行验证。

先按上文重建工作区，生成新增`SimulationCommand/SimulationFeedback`接口，然后准备固定版本资产：

```bash
python3 -m tools.models.prepare_g1_physics
# 源资产已在bindu_description；可追加 --description 指向已安装的资产包
export BINDU_INSTALL="$PWD/install"
export ISAAC_SIM_PATH="$HOME/isaacsim/_build/linux-x86_64/release"
# 本机 Fast DDS 仿真专用；所有参与终端使用相同设置，仅限制ROS DDS发现/传输到本机，不限制单独的Vuer WebSocket网络
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/tools/config/fastdds_local_sim.xml"
tools/run_g1_isaac.sh
```

默认打开完整G1图形界面，追加`--headless`才使用无头模式，`--no-ros`仅展示静止本体。启动器隔离Conda/Python动态库并限定ROS localhost，默认域125、命名空间`/bindu_g1_physics`。首次初始化可能较慢，等待终端`BINDU_G1_PHYSICS_READY`后，在另一个已加载ROS/Bindu工作区的终端运行：

```bash
export ROS_DOMAIN_ID=125 ROS_LOCALHOST_ONLY=1
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/tools/config/fastdds_local_sim.xml"
python3 -m tests.physics.validate_physics --output artifacts/g1-physics-check
# 另一次独立实验：先重启Isaac，再验证逆解与动态指令流
python3 -m tests.physics.validate_physics --suite dynamics --output artifacts/g1-physics-dynamics
# 各批次前均须重启Isaac；下面是独立的验证批次
python3 -m tests.physics.validate_physics --suite boundaries --output artifacts/g1-physics-boundaries
python3 -m tests.physics.validate_physics --suite faults --output artifacts/g1-physics-faults
# 独立批次：自行启动／重启Isaac GUI，只终止本验证器创建的进程。
# 沿用上面的ISAAC_SIM_PATH、BINDU_INSTALL和ROS_DOMAIN_ID；该命名空间不能已有模拟器。
python3 -m tests.physics.validate_physics --suite recovery --namespace /bindu_g1_recovery \
  --keep-simulator --output artifacts/g1-physics-recovery
python3 -m tests.physics.validate_physics --suite soak --soak-seconds 600 \
  --soak-amplitude .45 --soak-period 12 --output artifacts/g1-physics-soak
# 真实Vuer WebSocket接收合成手柄输入；右臂改为 --side right，使用新批次目录
python3 -m tests.physics.validate_physics --suite vr --side left --output artifacts/g1-physics-vr-left
# 独立加强批次：重新启动Isaac后，验证TLS、毫米噪声、大范围/快速运动和1米突变
python3 -m tests.physics.validate_physics --suite vr --vr-stress --vr-tls --output artifacts/g1-vr-stress
# --vr-filter-off用于另一次独立的滤波关闭对照；--side right验证右臂
# 固定延时近似：Isaac启动参数追加 --command-delay .030，验证器追加 --vr-network-delay .020
```

验证器自行启动和收尾同命名空间的G1执行/记录节点，检测20轴实测反馈、五组关节跟踪、取消后实际停稳、直行/圆弧及执行进程失联保护；图形仿真继续保留。 `--suite dynamics`复用Pinocchio/CasADi独立IK工作进程，验证左右臂末端目标到达、不可达目标不下发、20Hz连续笛卡尔目标、100Hz关节目标换向、取消/TTL断流保持及旧租约/越限/错序拒绝。IK种子和非活动关节均来自PhysX反馈；末端误差为实测关节FK在`base_link`中的推算，不是独立视觉或PhysX连杆位姿测量。该批使用合成目标，不含现场头显、Vuer接收或Pi服务；原始目标、执行参考和物理反馈分别记录在输出目录。每个仿真实例只接受首个执行器身份，重新运行验证器或替换执行器前须重启图形仿真。若手动运行能力入口，不要同时运行验证器：

```bash
ros2 launch bindu_runtime g1_sim.launch.py recording_mode:=normal namespace:=/bindu_g1_physics \
  profile:="$PWD/artifacts/g1-physics-model/g1_physics_sim.json" \
  device_backend:=external_simulation navigation_enabled:=false
```

`boundaries`增加双臂近奇异小步、突变拒绝、工作空间外扩扫描，以及独立PhysX连杆位姿与关节FK的同源时间对照。`simulation/link_poses`为world帧的PoseArray，固定顺序为base_link、左法兰、右法兰；由物理张量直接测量。`faults`通过隔离ROS中继注入反馈丢失、旧反馈重复、命令丢失/延迟及实例身份变化，检查看门狗保持、旧租约拒绝和故障锁定；身份变化为消息注入，不等于真实进程重启。`soak`默认100Hz运行10分钟，肩部±0.45rad、肘部±0.27rad、周期12秒；先用有限轨迹把测试轴移入留有摆幅余量的区间，再保存逐帧输入、参考和反馈的压缩JSONL、接纳计数、时间对齐跟踪误差与执行器内存趋势，失败批次也保留。

`recovery`复用故障中继，并两轮实际SIGKILL执行节点／Isaac进程后重启，检查本地看门狗、重连不自动运动、旧租约／旧实例命令拒绝，以及显式重建控制后的新命令到达。当前仿真桥绑定首个执行器身份，执行器单独重启不能接管，须重启双方；这不是已实现真机重新使能协议。Isaac重启会重新加载场景，进程消失期间没有物理演化，不把这段实验当作真机断电制动证明。`--keep-simulator`只在全部检查通过后保留最后GUI，并写入`simulator-process.json`；省略则关闭。中继断流是应用层传输故障注入，不改变主机网卡或验证实际Wi-Fi。真实Vuer连接的断开／重连可另运行`python3 -m tests.integration.validate_teleop --g1 --cases websocket_disconnect --output artifacts/g1-vr-reconnect`，检查断线结束、重连保持和显式新会话重新握持；使用模拟关节与合成手柄输入。

`vr`走真实Vuer WebSocket → CONTROLLER_MOVE解码 → ROS VRInput → 独立IK工作进程 → 租约执行器 → PhysX反馈链路。合成双手柄以名义72Hz发送OpenXR列主序矩阵；活动手柄做20秒三维弧线（x/y/z范围约80/80/120mm）及±0.08rad转腕，加入人为设定的0.15mm位置噪声、±2ms间隔抖动和周期25ms发送延迟，并渐变握持/扳机值。测试松手后手柄迁移214mm/转腕0.5rad再握持、取消、缺失手柄位姿和输入静默，检查实际停稳、重接无跳变及恢复输入不自动续跑。准备姿态单独用有限轨迹设置；开始运动前等待Vuer/IK/观察节点就绪及连续5秒新鲜物理采样。扳机只记录，不控制手部。原始WebSocket包、解码/IK事件/接受命令/执行状态、PhysX反馈及观察曲线均保留；这些输入分布不是头显实测数据。本批左右臂各5项检查通过：原始手柄帧4796/4862，接纳目标3162/3175；首个完整弧线的观察跟踪RMS为3.58/3.84mm、最大7.58/8.64mm（10Hz最新目标对实测关节FK，含时间滞后）。214mm手柄迁移后重新握持的首目标关节差为0，松手后保持漂移约3.4e-5rad；追踪无效和断流均停稳、恢复输入不自动续跑。修复了等待物理停稳后继续检查旧VR帧而误报输入超时的问题；117项模块及旧G1离合/断流/无效追踪3项回归通过。深弯肘准备姿态未满足速度停稳、启动期约330ms反馈发布阻塞及Vuer断连收尾异常的失败记录均保留；改用已验证姿态并等待启动采样稳定，不代表这些问题已解决。

2026-09-24加强验证采用弯肘±1.2rad准备姿态，并复用执行层检查，从4秒起按当前反馈选择满足动态限制的准备时长；执行等待时间随轨迹有效期计算。`--vr-stress`加入8秒精细运动、12秒大范围运动（x/y/z约160/160/240mm）、4秒快速往返、1mm高斯位置噪声及1米目标突变。`--vr-tls`生成仅本批使用的自签名测试证书，合成客户端显式信任它并校验127.0.0.1；验证WSS服务路径，不代表现场头显证书已配置。结果目录包含原始输入、IK目标、执行参考、实际关节、代理间距及耗时；受控组的执行参考采用实测非活动关节作为上下文，采样检查不构成连续碰撞证明。

仿真参数已在本体[`physics.json`](../bindu_description/physics.json)中调整：启用每次TGS迭代施加外力，改善本机静止关节速度残差；依据[NVIDIA求解器说明](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/107.3/dev_guide/simulation_control/simulation_control.html)进行对照，单独减少速度迭代未解决问题。双臂执行上限采用0.75rad/s、3rad/s²、18rad/s³，生成工具逐轴核对速度不超过URDF；位置/驱动力限制及停稳容差不变。这些是仿真参数，未标定真机；修改后需重新运行模型准备工具，公共执行算法与普通运动学profile未修改。仅回环非阻塞DDS配置已保存为上述可选工具文件，本批显式使用，未改变全局中间件默认。

本轮最终结果：左右臂各11项联合物理检查、基础物理10项（含此前低位停稳）、91项相关模块检查通过，2包重建；新生成profile与验证候选指纹一致。左右大范围跟随峰值16.8／16.4mm，4秒快速段28.6／19.8mm；实际与执行参考的配置代理最小间距约108.3mm（要求20mm，包含静止另一臂），IK中位约6.0ms／P95约7.7ms，接收事件到接纳观察中位22.6／23.0ms、P95约32ms，不含头显采集或接收前网络。两侧各完成追踪无效、输入断流和1米目标突变停车，恢复输入不自动续跑；记录器写入33,578／33,300条、接收后丢弃0。旧近伸直姿态大范围误差53.5mm、原动态限制右臂快速32.3mm及关闭滤波左臂快速39.9mm的失败均保留；滤波开关对照使用相同参数／随机种子脚本但运行调度不同，不能当成严格同时间戳回放或统计显著性结论。完整配置、原始轨迹、图和失败分类见`artifacts/vr-joint-2026-09-24/verification.json`。现场头显／局域网、长时间负载与连续碰撞证明仍未验收；Vuer断连处理仍打印异常，未影响本批停车与记录器收尾，待单独修复。未启动真机。

2026-09-24响应延时补测采用可调固定延时：Isaac `--command-delay`延迟指令交付，原时间戳／有效期不改、交付时仍校验，设备看门狗独立工作；验证器 `--vr-network-delay`延迟合成输入发送，保存采样和发送时刻，两者默认0。本轮设置执行30ms＋单程网络20ms，网络注入受72Hz发送循环调度实际中位27.8ms；按周期轨迹基频相位拟合，采样到物理末端FK的等效跟随滞后由基线185–222ms变为245–286ms，普通运动246ms、大范围278ms、快速286ms。延时批11项物理、11项命令门控及6项参数检查通过，未放宽原判据。该值是给定动作下的仿真近似，不是起动时间、最坏延时或真实头显／无线测量；额外延时不再次重复计入原滤波和插值。数据与复算脚本在`artifacts/vr-response-2026-09-24/verification.json`。

2026-09-22扩展验证：双臂IK边界与独立连杆位姿8项通过，438个同时间样本最大位置/姿态差8.40e-7m/8.96e-7rad；当前IK位置容差15mm，外扩10mm仍可接纳，30/80mm拒绝。大幅度100Hz流连续600秒、60,000/60,000接纳，实测肩部总摆幅0.9003rad、肘部0.5400rad，跟踪RMS0.00195rad、最大0.01021rad，执行器预热后RSS增长0.004MiB。早期长流约66秒失败，诊断发现第2代GC占用235.67ms并阻塞物理推进；就绪前回收/冻结初始化对象后通过完整10分钟，运行期新对象仍正常GC、退出解冻，原超时阈值不变。此结果不能替代30分钟及更长稳定性验证。故障组6项、117项模块及旧ROS取消/反馈丢失/动作块3项通过；同时修复历史停止失败记录污染恢复后新取消/释放服务响应的问题，受理停止仍不等于实测停稳。

本机生成的URDF／profile和导入缓存在忽略入库的`artifacts/g1-physics-model`，本体源网格／USD由统一资产包提供并保留Apache-2.0许可。该Isaac RC的GLB导入曾只生成空节点，工具改用匹配的上游USD视觉网格；同时按URDF重建关节并校验两侧关节坐标。两只驱动轮及低摩擦支撑为合成几何，尺寸、惯性、增益、初始姿态和120Hz物理步长集中在[physics.json](../bindu_description/physics.json)，尚未标定。无质量固定坐标框架使用微小数值质量；自碰撞关闭，该基础物理批次未验证手部、抓取接触、相机图像；导航和建图另见下文专项。完整外观和物理设备闭环不等于真机模型验收。 本机RC完整扩展卸载曾崩溃，启动器采用Isaac默认快速退出，先完成Bindu日志和ROS清理；异常路径保留非零返回码。物理步长是积分配置，不代表已达到实时频率。

本批物理验证：117项模块与10项PhysX检查通过；五组关节跟踪最大目标误差0.00050rad，直行0.0837m、圆弧0.0696m/0.1215rad，执行器冻结后实测线速度0.000142m/s。相关运动学回归G1全身9、VR4、Pi14通过；旧ROS首轮15/16、导航17/18，其启动拒绝场景单独复测各通过，启动偶发性尚未解决。以上不代表VR/Pi/导航已经完成物理环境验收。

2026-09-22追加逆解/动态流物理批次：10项检查通过。左右臂末端位移约46mm，到达位置误差0.486/0.512mm；20Hz连续逆解100/100接纳，末端跟踪RMS3.38mm，求解耗时p95为3.08ms；100Hz关节流600/600接纳，实发均频99.99Hz、间隔p95为10.97ms，实测相对参考的关节误差RMS0.00157rad。取消/断流后0.5s保持漂移均小于0.00010rad；不可达、旧租约、越限与错序拒绝通过。这是短时合成目标实验，不代表长期实时性或现场VR体验。



### Snowbot真实夹爪位置闭环

夹爪复用现有Snowbot执行节点，默认配方引用[`snowbot_grippers_real.json`](../../integration/bindu_runtime/config/robots/snowbot/snowbot_grippers_real.json)，不新增电机控制进程。该接口确认的是位置到位和保持；`grasp_state=unknown`明确表示没有实体夹持／夹紧力／虚握证明，不能据`position_reached=true`进入持物作业。

| 接口（默认前缀`/bindu_snowbot`） | 用途 |
|---|---|
| `gripper/lease`，`bindu_interfaces/srv/Lease` | 独立夹爪控制权，资源为`left_tool_axis`／`right_tool_axis`；acquire／renew／release，2秒租期，lease_id与epoch隔离旧请求 |
| `gripper/control`，`bindu_interfaces/srv/GripperControl` | `authorize_direct`授权停稳后的开／闭端点；`authorize_loading`声明空手／持物及支撑，`open`／`close`／`position`／`hold`；`authorize_grasp`授权仅右爪位置换握，`virtual`／`rigid`选择配置的虚握／闭合位；请求含递增sequence、源stamp及≤0.4秒valid_for；接纳仅返回`REQUEST_ACCEPTED` |
| `gripper/state`，`bindu_interfaces/msg/GripperState` | 分侧发布实测位置／速度／电流、目标、配置指纹、授权与扳机就绪、到位／保持／故障状态；原始反馈保留源时间 |

只有臂会话已停止且释放租约、反馈与PDO健康、受保护接收方及发布者唯一、配置已标定时才允许取得夹爪控制权。夹爪租约、停止中或停止故障会拒绝新的手臂接管。装卸授权下，持物或状态不明开爪仍要求支撑；任意`position`操作在持物时也要求支撑。PICO停稳、双侧松握后的显式扳机操作使用独立`authorize_direct`，仅允许标定的`open`／`close`，不接受任意位置／虚握，不伪造空载、支撑或夹持证明。未操作侧不发送目标，停止撤销授权并保持实测开度。实体标定、握持与防滑效果仍待现场验收，行为边界见[工具操作契约](../../../动作执行契约.md#61-snowbot-工具操作与握持切换)。

**位置虚握。** 按用户确认，先以右爪轻微张开的可调开度接通X换握。修改现场夹爪配置的`grippers.right.virtual`，单位rad，示例暂设`0.02`（约1.15°），闭合位为`grippers.right.closed`；虚握值须位于闭合与完全张开之间，并与两端各相隔超过两倍位置容差。示例仍默认`calibrated=false`，现场调整开合／虚握值并核实后再按原步骤启用写入。普通模式不使用X换握；扫雪跟随中按住X先停稳并释放臂租约，确认左爪闭合停稳，检查右爪完整路径，再仅租用右爪执行虚握位，等底层接纳、实测到位和最终保持后按轴向释放模型捕获当前基准。释放X按同流程回闭合位，再以当前实际转角恢复刚握。切换期间不推进工具目标，双侧握和输入仍有效才恢复跟随；松握、Y或断流保持当前开度，不自动夹紧。这里只确认位置模式，`grasp_state=unknown`与`physical_gripper_confirmed=null`保持，实体承托、可转动及防滑效果由现场验证。

**118原生协议。** 当前发送到`<vendor>/gripper/left_cmd`和`right_cmd`，格式为`[position, velocity, torque, kp, kd]`；位置操作的速度参考和前馈力矩为0。配置中的`max_speed`不等于该协议的速度参考，现场底层能否限制运动速度仍待核实。实测位置、速度、电流复用`RetInformation`左右第8项；`*_cmd_sensor`是命令入口，不作为状态源。原生消息没有命令序号或保持回执，到位及停止均要求发送后推进的实测和稳定区间，不能用本地递增序号冒充底层ACK。

**现场启用前提。** 原生映射不解除标定、控制权、实际使能及PDO健康／看门狗门槛。118尚缺少这些保护契约的验证，写入仍保持关闭。历史`bindu_guard.patch`的自定义夹爪协议会屏蔽原生通道，不适用于当前映射；遇到该保护协议时预检／写入检查明确拒绝，不自动替换底层补丁。仿真后端原有应用序号确认继续保留。夹持与带载保持还需现场验收。

**分层预检。** 在完成本机改码、验证、版本同步与目标环境重建后，`python3 -m tools.deployment.x86_real check`只证明软件依赖、消息字段、配置、TLS和端口检查通过；16轴兼容时会列出缺失头腰，输出`ROBOT_INTERFACE_NOT_CHECKED`，不声称机器人在线。底层由现场按既有流程运行后，可执行只读`python3 -m tools.deployment.x86_real check-robot`，核对话题类型、发布者／接收节点、桥接端点、新鲜实测和保护健康；返回非零时保留具体阻断项。统一入口也支持`tools/run_vr.sh --check --check-robot --robot-check-timeout 3`。检查只创建观察节点和订阅，不启动底层、不调用运动服务或发布命令；允许写入的真实启动还会强制执行该在线门控。

历史自定义夹爪补丁提供EtherCAT线程的独立TTL与保持回执；当前原生通道尚未取得同等底层保护证据，不能沿用该验收结论。上层停止／释放／断流先请求保持实测位置，再要求推进的PDO、新鲜实测位置与速度及稳定时间；超时锁存`GRIPPER_STOP_UNCONFIRMED`并拒绝重新接管。目标受阻未到达则报告`POSITION_TIMEOUT`，不伪装成夹持成功。软件保持不替代现场机械保持或急停；旧底层启动回位／清零仍未被此功能取消。

复制夹爪配置到现场目录，依据实物标定每侧`open`／`closed`（rad）、`kp`／`kd`和`max_speed`（rad/s），核对位置／速度容差和超时后才把`calibrated`设为true。示例左开度−0.4、右+0.4及零位闭合仅取驱动方向的参考，默认`calibrated=false`，不是实机标定值，不能照搬模拟夹爪开度。电流是底层反馈数值，不能当作已标定指面夹紧力。将现场文件传给部署工具`configure --gripper-config /绝对路径/grippers-site.json`；其他TLS、话题和写权限参数沿用现场配方。

在已经标定、允许写入且本轮获准的运动实验中，工具提供有界的开合与收尾；以下是**操作命令**，应先结束工具／手臂会话。每条命令自动取得租约、授权、续租、等待实测到位并释放后确认保持：

```bash
bash tools/snowbot_teleop.sh gripper --config artifacts/real-teleop/site-write.json \
  --side left --operation open --load-state empty
bash tools/snowbot_teleop.sh gripper --config artifacts/real-teleop/site-write.json \
  --side left --operation close --load-state empty
# 持物装卸须明确已有外部支撑；同样适用于right。
bash tools/snowbot_teleop.sh gripper --config artifacts/real-teleop/site-write.json \
  --side right --operation open --load-state held --external-support
ros2 topic echo /bindu_snowbot/gripper/state
```

需要手柄装卸时，先从当前`VRInput.source_id`复制实际来源，绑定夹爪租约和装卸资格：

```bash
read -r -p '填入当前VRInput的source_id: ' SNOWBOT_VR_SOURCE
bash tools/snowbot_teleop.sh gripper --config artifacts/real-teleop/site-write.json \
  --side both --operation authorize_loading --load-state held --external-support \
  --vr-source-id "$SNOWBOT_VR_SOURCE"
```

该命令只授权，不自动开合；观察`loading_authorized`及两侧`trigger_armed`，先各自释放Trigger，再按住对应Trigger打开、有效释放闭合。启动时已按住再释放不构成闭合请求，合并帧中的含糊按下／释放会停止，不补执行。保持两侧Squeeze释放，Y、其他操作键、追踪失效、任一路输入超时或来源失效会撤销资格并请求保持；断连缺省零值不代表释放，不自动闭合。授权绑定当前租约／来源，停止后需新授权。工具会话中的Trigger仍走停止语义，不能直接从跟随跳进装卸。

验证入口（合成输入，不启动底层控制／总线）：

```bash
python3 -m tests unit test_snowbot_gripper
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=226 python3 -m tests.integration.validate_snowbot_gripper \
  --output artifacts/gripper-wire-check
```

本轮Jazzy的单元、ROS替身及底层假PDO验证不等于Humble／Orin、现场PICO、实体开合、夹紧力或虚握验收。摇杆方向在普通／扫雪模式都预留为底盘控制，头部后续可跟随VR视角，本轮均未接入。



### Snowbot 位置控制适配层

2026-10-13按用户提供的`src.zip`增加独立`snowbot_adapter`入口；原控制包的消息、J1～J7顺序、rad单位和URDF限位作为本次依据。`snowbot_vendor_position.json`单独维护20轴反馈及源限位，不覆盖既有Snowbot仿真profile、G1或二代轮式配置。原始控制源码不迁入公共模块；控制侧专属补丁、源码准备工具及离线验证在[Snowbot工作区](../../../../../snowbot/运行与验证.md#本机底层源码准备与离线验证)维护，本模块只保留可复用设备适配接口。

底层原包与118补丁的准备、版本指纹及适用边界见[Snowbot运行与验证](../../../../../snowbot/运行与验证.md#本机底层源码准备与离线验证)。

`session_v1`使用独立位置会话与`PositionObservation`，真实场景以腰部2轴＋双臂14轴为一个控制组接管、联合求解和停止；完整模型仍要求其余关节具备新鲜实测，不补零。腰部目标与双臂共用原目标身份／期限，在三个独立封包内透传，servo单处Follower输出由原PT／CSP接口发送，不再把新腰目标串入旧腰部插补。暂停回传16轴固定保持点、采样号及会话／PAUSE序号；腰部回执对应实际编码器目标的量化值，执行器仍按原位置／速度容差及预算确认新鲜实测停稳，回执本身不表示物理停止。接管只改变受控组使能，不改变旧全局使能或头部／夹爪资源。新字段仅扩展独立消息，全部使用方须统一构建。期限要求控制链同机部署；共同向量限幅的适用边界、停止策略和现场条件见[修改目标](../../../../../snowbot/位置遥操作修改目标.md#暂停确认资源与异常停止策略)。

真实腰臂遥操的新装配为`session_v3`：仅发送16轴目标（左J1～7、右J1～7、腰J1～2，rad）及源身份／原期限，servo显式`target_stream`路径后台生成、统一采样。普通目标与PAUSE不依赖上一轨迹接纳；新路径的停止参考只接受正常PAUSE有效输出，过期／故障保持不能冒充成功。`legacy_guard/session_v1`设备接口保留；旧上层曲线`session_v2`已从本轮装配退休。配对、动态限制和纯位置边界见[运行装配](../../integration/bindu_runtime/README.md#扫雪遥操作)。

`session_v1`只读预检同时比较本机生成消息与各层端点的类型哈希，并按实际发布／订阅方向核对QoS；缺失／未知元数据或不兼容均不报告可用。探测仅增加被动订阅，不创建运动发布者或调用控制服务。ROS元数据语义见[端点信息](https://github.com/ros2/rclpy/blob/jazzy/rclpy/rclpy/topic_endpoint_info.py)及[QoS检查](https://github.com/ros2/rclpy/blob/jazzy/rclpy/rclpy/qos.py)。

以下为`legacy_guard`接口：在目标ROS环境构建／加载原包的`snowbot_ctl_msgs`，然后构建并加载Bindu。启用写入时，现场底层控制版本必须应用补丁、上层启动为`control_source:=topic`、伺服节点启动为`bindu_guard:=true`；后者默认false，保留原有控制入口。未补丁的底层节点仍可用于只读观察，适配器拒绝取得控制权。底层全包的硬编码依赖路径和ARM64构建仍需现场验证，准备成功不代表已构建／部署。

```bash
# <控制源码>指原包解压路径；此命令只构建消息，不启动运控。
colcon build --base-paths <控制源码>/src/ctl_msgs --packages-select snowbot_ctl_msgs \
  --build-base /tmp/snowbot-msg-build --install-base /tmp/snowbot-msg-install
source /tmp/snowbot-msg-install/setup.bash
# 再按环境准备章节构建并加载Bindu；默认入口只读，无命令发布者。
ros2 launch bindu_runtime snowbot_adapter.launch.py
```

默认Bindu命名空间为`/bindu_snowbot`，底层话题前缀为`/snowbot`。如果现场底层节点仍用源码中的根话题，应**显式**设置`vendor_namespace:=/`；这只是话题匹配，不负责启动底层节点。原接口中若有绝对话题，给底层节点设置namespace并不会自动改写它们，须配置一致的显式remap；沿用根话题时选择`vendor_namespace:=/`。只有在获准的运动实验中才配置`command_enabled:=true`。此兼容launch默认真实只读；指定`runtime_mode:=simulation`则改装配仿真执行器。直接运行`snowbot_adapter`可执行文件只允许`real`；合成协议验证仍选择真实接口并在隔离ROS域提供替身，证据须标为合成，不能改模式掩盖入口性质。

控制上层必须在启动时配置`control_source=topic`；直接运行原上层节点的默认值是`demo`，不能把话题已订阅视为位置命令已经接管。适配器默认匹配`<vendor>/robot_upperMonitor`双臂接收节点和`<vendor>/servo_node`腰头接收节点；现场重命名时可用`arm_receiver_node`、`servo_receiver_node`指定实际全限定节点名。`mid_receiver_node`默认`<vendor>/dual_arm_control_node`，双臂还须检查上层→中层→伺服两个内部话题的唯一发布方与实际接收节点。新增`bindu/control_source`回显内部实际枚举，修改ROS参数但未切换内部模式不能伪造接管。新增`bindu/servo_health`包含启动回位结果、总线及当前实例心跳确认；这些检查通过才允许取得租约。

| 输入／输出 | 作用 |
|---|---|
| `/<vendor>/joint_position_cmd` | 原`JointPositionCommand`，左／右各7项；两臂共用消息，必须同时取得`left_arm`、`right_arm`控制权，单臂目标中的另一臂首次从实测位置捕获后保持 |
| `/<vendor>/body/motor_cmd`、`head/motor_cmd` | 两项rad位置，直接接原servo入口；不再重复经过body/head目标显示节点 |
| `/<vendor>/dualArmCtl_fdback` | 原`RetInformation`实测位置／速度；J8映射为左右`*_tool_axis`，臂位置契约不写J8，开合使用[独立夹爪接口](#snowbot真实夹爪位置闭环) |
| `/<vendor>/bindu/{control_source,servo_health}` | `std_msgs/String`，JSON schema 1；分别由实际上层／伺服节点发布，校验发布者身份、源时间、成功PDO进度与驱动故障 |
| `/<vendor>/bindu/heartbeat` | `UInt64MultiArray`，`[ROS纳秒,递增序号,实例ID]`；20 ms周期，底层确认实例绑定后接管；不包含运动目标 |
| `/<bindu>/execution/{lease,submit,control}`、`targets` | 沿用Bindu服务／消息，单组与schema 2完整具名组目标均可提交；只接受`joint_target`，不接受定时轨迹、导数动作块或底盘速度 |
| `/<bindu>/execution/state` | 实测反馈、源时间、控制权、参考目标与停止状态；没有底盘接口，零值底盘字段不表示已取得里程计 |

执行周期配置20 ms，直接转发目标，原控制器负责平滑；不承诺Bindu曲线的速度／加速度／jerk上限或指定到达时间。`stamp`不能在未来，目标TTL最多0.4秒；拒绝旧时间、错误profile／epoch、缺关节、非有限和超限值。反馈源须唯一，目标话题须只有本适配器一个发布者并具有匹配的底层接收节点；被动监视器可同时订阅，只有监视器时拒绝下发。取得控制权前也检查接收方就绪，服务已出现不代表DDS路由已可用。节点名用于ROS路由检查；总线健康与实际控制模式另由上述两路状态验证。重启实例前需确认旧命令发布者从DDS图中清退，竞争源／接收方不匹配会在控制台给出具体话题诊断。不同腰头／双臂话题依次发送，不承诺硬件原子写入。

停止／有效期耗尽时，适配器只对已涉及的轴发送实测位置保持请求；新反馈位置和速度满足容差后才结束`STOPPING`。双臂保持仍为合并消息，腰头保持分别尝试，某一路失联不能阻止其他路收到保持。无新鲜反馈、发送失败或等待超时会报告`STOP_UNCONFIRMED`、撤销租约，并拒绝在同一实例重新接管；后续周期故障不延长第一次停止的截止时间。停止中释放租约立即撤销控制权，原控制者不可继续提交；新接管仍须等待停稳。只读启动、取得租约、空闲退出均不产生运动消息。该保持依赖原控制器跟随，**不是硬件急停**，也不通过失能消息让原程序回位／抱闸。

适配器自身收到SIGINT／SIGTERM时先关闭新接管和写入，保持ROS通信完成上述有界停止确认后再退出；未确认时退出码为1。未补丁或`bindu_guard=false`的原控制程序`Ctrl+C`仍会触发回位。补丁在`bindu_guard=true`时跳过退出回位，进入原有失能／关闭总线收尾；实际抱闸状态和停机效果仍未验收。

补丁看门狗在EtherCAT线程独立检查心跳，接管后200 ms断流、有效PDO超过20 ms未更新、WKC失败或受控轴驱动故障会锁存保持；ROS主线程卡住或中层继续重发旧目标也不能刷新这一时限。只捕获一次最近成功PDO中的位置，速度归零，冻结PT的原增益／力矩前馈；不启用原先关闭的轴，也不推进使能序列或自动复位故障。J8不进入保持覆盖；2026-10-14补丁进一步约束保护模式下的臂回调，不再随臂指令强制使能左右J8，原默认模式行为保留。丢失总线时保持包可能无法送达，PT保持载荷与CSP停止效果须实测；这是软件保持请求，不能替代硬件急停。

心跳绑定当前适配器实例，拒绝乱序／重复／过期数据，迟到包或新实例不能解锁。停止确认期间保留心跳，正常退出后也会触发设备侧保持。故障或适配器重启后须由操作者按获准流程检查／恢复底层控制，软件不自动重启原程序；底层启动仍包含回位动作。

当前底层启动仍含回位／六维清零，只读设置和`bindu_guard=true`不能阻止。待命改造、受控重新绑定及支架卸载后的恢复条件统一见[实机启动方案](../../../软件架构设计.md#832-snowbot-实机启动与准备)，未验证前不得按上层PICO流程带铲重启旧底层。

原`RetInformation.header`仍是ROS发布时间。独立健康消息中的`sample`只在成功PDO交换及缓存更新后递增，`sample_age`来自底层单调时钟；失败WKC不刷新测量缓存或样本ID。适配器要求WKC相等、PDO年龄≤20 ms及成功样本在50 ms内推进，持续刷新ROS header不能掩盖冻结PDO。`present_mask`表示实际映射，`fault_mask`来自状态字；`enable_requested_mask`为请求使能，`operation_enabled_mask`为状态字实际operation-enabled，二者不可混为一谈。各掩码使用源驱动0–19槽位布局。实测反馈和健康消息均核对实际伺服发布者身份，单一错误发布者也不能用于停稳确认。上层源和健康消息均须新鲜（≤200 ms），`ready`表示启动回位检查完成，不表示负载、机械限位或运行模式已做真机验收。后续新增的[真机最小遥操作](../../integration/bindu_runtime/README.md#snowbot-真机最小遥操作)复用此位置适配；实体夹持、六维反馈和完整PICO流程仍需另行适配，独立`../snowbot`项目未改。

**验证。** 首版证据在`artifacts/snowbot-adapter-2026-10-13/`。同日审查修复监视器误拒绝、部分停止失败阻断其他轴、重复超时延长停止和信号退出无法送达保持及停止中释放未撤权5项问题，并补齐接收方就绪门控；Jazzy和Ubuntu22.04/Humble各重建3个相关包，各80项模块／分层检查、11项合成ROS接口检查（含活动期间SIGINT／SIGTERM保持并等待反馈）通过，Jazzy默认launch只读检查及其他本体取消／在线流2项既有ROS回归通过。分层测试明确已声明ROS依赖的Airy模块为输入适配角色，核心禁依赖规则保留；导航运行源码未改。审查证据在`artifacts/snowbot-adapter-review-2026-10-13/`。一轮快速重启未观察到RUNNING且旧日志诊断不足，失败证据保留；联调增加端点清退等待／状态尾部及话题诊断，适配器取得控制权前新增接收方就绪门控后最终通过，尚不能据此认定长期调度／重启稳定性。未启动原运控／EtherCAT，无真机、ARM64或带载验收；时间修正、独立看门狗与健康门控的后续验证见下段。复现接口检查须先加载两套消息／工作区，并使用新的本地结果目录：

```bash
python3 -m tests unit test_snowbot_adapter
ROS_DOMAIN_ID=191 ROS_LOCALHOST_ONLY=1 python3 -m tests.integration.validate_snowbot_adapter \
  --shutdown-check --output artifacts/snowbot-adapter-ros-check
```

本轮完善的证据保存于`artifacts/snowbot-guard-2026-10-13/`：Jazzy与Ubuntu22.04/Humble各84项相关模块／分层检查、14项合成ROS检查；假PDO测试执行补丁驱动的实际循环，覆盖断流、乱序、实例冲突、位置冻结、J8隔离和无效WKC不推进。三个修改的底层ROS/C++源文件通过两平台编译检查；并发假PDO／健康发布检查覆盖样本年龄非负，修正读取时间与RT更新之间的竞争。源码准备及检查可复现如下（先加载ROS和底层消息安装）：

底层假PDO验证已移至[Snowbot离线验证](../../../../../snowbot/运行与验证.md#本机底层源码准备与离线验证)，从该工作区运行。

上述检查未链接／构建完整底层运控程序，未启动SOEM主站、使能电机或运行回位；原有硬编码依赖、ARM64、真实总线断流、带载保持、夹持和机械停稳仍未验收。默认只读保护和其他本体执行策略保持不变。



### 木质雪铲的 PhysX 受力实验（2026-10-15）

日常物理后端复用VR、QP、位置执行和浏览器观察。启动及完整流程检查：

```bash
python3 -m tools.teleop.run_vr --physics --config src/integration/bindu_runtime/config/deployments/snowbot_vr_sim.json
ROS_DOMAIN_ID=191 ROS_LOCALHOST_ONLY=1 python3 -m tests integration position --mode simulation --teleop-mode both --physics --websocket-input --output artifacts/vr-physics-check
```

`tools/run_shovel_physics.sh`另用于载荷／接触专项实验，参数由`snowbot_shovel_physics.json`及脚本帮助维护。参考与实测反馈分别记录，参考插补不作为实际握持证明；实体接触、防滑、雪载荷及真机性能仍需验收。2026-10-15批次的完整命令、参数和结果保留在本地`过程记录/2026-10-15.md`。



### 8.5.5 设备接入拆分与历史代码落点

2026-10-13增加Snowbot专用位置适配：硬件包`snowbot.py`负责原协议布局、具名组和实测反馈，运行包`snowbot_adapter.py`只在显式入口导入底层消息并创建话题；`delegated_position.py`复用执行器合同检查，将平滑委托给原控制器。限位单独来自原控制源码，现有本体配置与普通执行器策略不变。新入口默认只读，未接通夹爪或底盘；当前遥操作会话的模拟保护继续保留，原控制器启停／总线健康仍需现场验收。同日复查后，停止按独立端点逐一尝试并固定首个截止时间，Snowbot入口自身负责保持通信完成有界退出；接收节点名用于匹配底层入口，被动监视器不再误触发拒绝。控制侧以校验原ZIP指纹的准备工具和可复现补丁维护，独立于普通Bindu构建；补丁负责单调计时、实际控制源／成功PDO健康回显及EtherCAT线程看门狗。适配器负责唯一源、实例绑定和接管门控，缺少底层健康能力时写入关闭。补丁仅针对Snowbot，不改变其他本体装配。详细接口与命令集中在[运行与验证](#snowbot-位置控制适配层)。

用户要求进一步收窄“机器人后端”的含义。当前用以下职责替代宽泛后端；下表驱动与 I/O 路径相对 `src/hardware/bindu_hardware/bindu_hardware/`，执行与运行装配另列所属包：

| 职责 | 当前代码 | 边界 |
|---|---|---|
| 执行管理 | `bindu_execution/executor.py`（位于 control 分类） | 控制权、有效期、revision、到达与停止；同包 `interpolation.py` 生成/认证参考曲线，算法不导入具体能力或驱动 |
| 身体/手臂关节驱动 | `drivers/simulated_joints.py` | 单个具名关节组的位置写入、停止请求和自身反馈；设备编号、单位和设备协议由真实适配器处理 |
| 灵巧手驱动 | `drivers/simulated_hand.py` | 独立实例与配置，可单独替换；当前仅模拟位置响应，真实耦合/触觉/力模式尚未实现 |
| 底盘驱动 | `drivers/simulated_base.py` | 底盘速度、停止及自身反馈；不负责导航、建图或双足步态 |
| 命令路由 | `robot_io/routing.py` | 按具名组路由；schema 2 对租约内互斥组分发同一采样参考，部分写入失败请求全部设备停止；某设备停止失败仍尝试停止其他设备 |
| 反馈汇总 | `robot_io/feedback.py` | 校验字段、保留每个设备采样时间；汇总时间取最旧样本，健康设备不能掩盖失联设备 |
| 装配 | `robot_io/system.py` + `bindu_runtime/device_assembly.py`（位于 integration 分类） | 组合端口，不承载设备物理模型或 SDK；配置选择每类驱动，模拟故障注入留在装配/测试侧 |

各驱动使用[独立接口](../../shared/bindu_contracts/bindu_contracts/devices.py)。读取接口必须有界且不阻塞；`dt` 供模拟推进，真实驱动应提供独立采样时间的最新设备反馈。关节与手可共享位置接口，但不能因此假设手部支持 CSP、触觉或相同周期。`ExecutionState` 带各反馈源时间与停止失败设备；停止后位置参考置空，停止请求不伪装成新的关节目标。当前仍串行执行，不是多设备原子事务或生产实时控制。

历史资产的源码依据集中在盘点报告（本地参考：`历史代码盘点与复用评估.md`，不随 Git 分发），下表定义接入位置；**Pi 客户端及 v3.4 单臂 VR/IK 已提取，其他历史资产尚未迁移**：

| 历史资产 | 对应位置 | 当前需要补齐 |
|---|---|---|
| `adaptive_interpolator.cpp`（腰腿控制工程） | 执行层的插值/采样组件，位于 `control/`，邻接 `executor.py`；C++ 集成时可独立执行包 | 接入时间轴、断流/取消与跟踪反馈，不放进设备驱动 |
| Ti5/CAN、EtherCAT、`slrHardware/slrKernel`、旧手部与 Tracer/AgileX 接口 | 各类 `drivers/` 适配实现 | 与当前硬件匹配、SDK版本、单位/具名映射和停止能力 |
| v3.4 VR 相对目标、模式管理 | `src/capabilities/bindu_teleoperation/` 输入适配，输出进入统一执行入口 | 现场头显/TLS、init 姿态、双臂及手部语义 |
| v3.4 Pinocchio/CasADi FK/IK | `src/capabilities/bindu_kinematics/` 能力，首版供单臂遥操作调用 | 当前本体标定、碰撞对和 Orin 求解性能；保持与 VR/驱动解耦 |
| cuRobo 规划节点、OCS2 控制器 | `src/capabilities/bindu_planning/` 的规划/连续控制适配；重计算由独立进程承载 | 生成同一执行合同的运动提议，不能保留旧直接硬件写入通路 |
| `foundationpose_zmq_to_pose.py`、软管 YOLO/UNet/点云 | `src/capabilities/bindu_perception/` 的算法实现及对应 ROS 服务适配，保留 `LocateObject` 边界 | FP推理、饮料模型与标定；FoundationStereo 不等于 FoundationPose |
| ROS 1 `nav_server.py`、`TaskTransfer.py`、LIO/地图/航点 | `src/capabilities/bindu_navigation/` 的实际能力适配 | ROS 2/Nav2迁移、TF、取消/停稳、有限重试；底盘驱动独立 |
| ZMQ Pi 客户端；ACT 暂缓 | `src/capabilities/bindu_vla/` 的真实模型客户端与动作解码；通信依赖留在适配器 | Pi 已接模拟执行，真实服务/相机/映射待联调；ACT 不在本批范围 |
| `collect_data.py`、LeRobot插件、CSV诊断 | `data/bindu_recording/` 接收侧与后续数据采集/转换管线 | 修复保存错误、多指维度、图像同步和动作标签，当前JSONL不等于训练数据集 |
| Qt GUI / 第二版GUI设计 | 后续独立调试客户端，调用公共ROS服务/Action并订阅反馈 | ROS 2迁移；不把插值循环和硬件写入移到GUI |
| URDF、碰撞球、手模型与参数 | 后续模型/描述包 + 当前 `profile.py` 的配置输入 | 对应当前本体及标定；简单JSON profile不能代替完整机器人描述 |

运动学与遥操作首版见8.5.4；交互、完整本体描述包与训练数据管线仍待建设。v3.4 输入/IK 已拆分接入，原历史工程保持只读。

**手动本体选择（2026-10-11）。** 运行装配通过版本化本体配置引用URDF、具名profile、单臂/工具配置及可选位置协议映射；语义角色与硬件名称分离。Snowbot和二代轮式是独立本体。`RobotSetup`在启动节点前交叉检查模型、关节覆盖、限位、末端和测量上下文，控制/显示模型必须保持同一关节运动链，工具碰撞检查使用所选显示模型；节点再次核对同一配置指纹。活动自由度由组内具名关节推导，不以协议通道数决定QP维数；未参与求解的关节仍保持实测上下文。`PositionMapping`隔离rad/deg、方向、偏置及整包保持通道，失败反馈废弃旧捕获；传输、接管、停止及实体握持仍属于后端职责。默认遥操作入口和旧Snowbot入口共用一份装配。当前真实后端未启用，二代完整模型待核对，不因有映射文件宣称真机适配完成。配置和验证入口见[运行说明](../../integration/bindu_runtime/README.md#手动选择本体与自由度)。
