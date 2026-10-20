# bindu_navigation：导航适配与输入检查

[项目模块导航](../../../README.md#模块快速导航) · [Orin导航部署](#orin-导航专用部署与新手操作) · [编译与运行](#编译与运行) · [职责与边界](#职责与边界) · [源码导航](#源码导航) · [接入与验证](#接入与验证) · [关联模块](#关联模块)

维护导航配置、站点与每次目标的输入检查，并提供 Nav2 速度身份检查和 Airy 扫描转换。Nav2 会话的 ROS 装配位于 `bindu_runtime`。

## Orin 导航专用部署与新手操作

目标是 Ubuntu 22.04／ARM64／ROS 2 Humble。底盘已确认为松灵 Tracer 2.0；用户要求本轮底盘接口先占位。**当前可交付环境准备、专用构建、只读预检、传感器条件满足后的建图／保存／重载定位；人工速度控制、导航到点与停车闭环仍待底盘适配，不能宣称真机导航已完成。** Orin 型号／内存／JetPack、雷达／IMU、驱动和真实里程计尚未确认。本节不采用历史 Airy 或 G1 参数。

### 简短启动入口

所有终端先执行 `cd /home/wxb/bindu`。统一脚本会加载本机专用环境，不需要复制长串 launch 参数。先按后文安装依赖并填写配置，再依次运行：

```bash
tools/run_navigation.sh environment                     # 只读检查目标机
tools/run_navigation.sh dependencies --route 2d          # 只打印所选路线的安装命令；LIO换成lio
tools/run_navigation.sh prepare --route 2d               # 本机生成现场配置；LIO换成lio
tools/run_navigation.sh build                           # 专用构建，默认7包；LIO再加1包
tools/run_navigation.sh check --stage config             # 配置检查，缺项逐项报出
tools/run_navigation.sh check --stage sensors            # 只读采样，启动算法前检查
tools/run_navigation.sh start --phase mapping            # 终端A：只启动建图，不启动底盘
tools/run_navigation.sh rviz                             # 终端B：观察地图/扫描/TF
tools/run_navigation.sh check --stage mapping             # 终端C：建图与保存服务检查
tools/run_navigation.sh save-map                         # 终端C：保存本批地图
# Ctrl+C退出终端A的建图；填写site.json中map.yaml（LIO还需map.pcd）后：
tools/run_navigation.sh start --phase localization       # 终端A：重载定位；RViz给近似初始位姿
tools/run_navigation.sh check --stage localization       # 终端C：确认定位及地图有效
```

默认工作目录 `artifacts/orin-navigation/`。如需另一批部署，每条命令加同一 `--workspace /绝对路径/新目录`。`prepare` 不覆盖非空目录，地图也拒绝覆盖。`environment` 和 `check` 为只读操作；`prepare/build` 只写本机构建目录；脚本不安装驱动、配置 CAN／网络、使能电机或重刷 JetPack。

### 1. 环境与依赖准备（目标 Orin 的终端）

运行 `tools/run_navigation.sh environment`，预期显示 Ubuntu 22.04、`aarch64`、Python 3.10，以及可读到的设备型号、内存、L4T／JetPack。`nvidia-jetpack` 元包未安装时版本可能为空，应现场补充，不能据此认定 JetPack 未安装。x86 或 Python ABI 不符会退出2；不能通过改机器类型继续部署。新克隆需要先安装 Git 并取得本仓库源码；不要搬运开发机的 `install/`、venv、`.so` 或实验 `env.sh`。

尚未安装 ROS 时，按 [Humble 官方安装步骤](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)完成 UTF-8 locale、Universe、ROS 软件源及系统包兼容性检查，再执行：

```bash
sudo apt update
sudo apt install ros-humble-ros-base
source /opt/ros/humble/setup.bash
ros2 --help
```

成功判据：`ros2 --help` 正常显示帮助；`/opt/ros/humble/setup.bash` 存在。找不到软件包时先核对 Ubuntu 版本、软件源和 ARM64 架构，不安装 x86 包。官方安装页的[源码版本](https://github.com/ros2/ros2_documentation/blob/humble/source/Installation/Ubuntu-Install-Debs.rst)可在文档站不可访问时查阅。

选路线后，运行 `dependencies --route 2d` 或 `--route lio`，**阅读并执行它打印的完整 `sudo apt install ...` 命令**。该输出包含编译工具、ROS 导航与 RViz；二维路线加 SLAM Toolbox／AMCL，LIO 路线加 PCL／Eigen／SciPy／Open3D。再安装实际型号的 ARM64 传感器驱动。两条路线均不要求 VR、Pinocchio、CasADi、ProxSuite、VLA 或 Isaac。

| 路线 | 必须具备的输入 | 建图／定位与保存 |
|---|---|---|
| `2d` | 有效 LaserScan、真实 odom、连续 odom→base、已标定 base→scan；点云转扫描须先按实际传感器适配 | SLAM Toolbox → YAML＋图像＋位姿图；重启后 AMCL 加近似初值 |
| `lio` | 已标定且同步的点云＋IMU；点云 x/y/z/intensity/**time 均为 float32**，time 是相对帧头的秒偏移，非接收时间 | FAST-LIO → PCD＋观测扫描栅格；重启后 FAST-LIO＋需近似初值的 ICP＋栅格 map_server |

当前 Orin LIO 入口要求适配后的 `timed_cloud`，`scan` frame 与 `lidar` 一致，雷达 z 轴平行底盘 z 轴；倾斜安装须先适配水平扫描。既有 Livox 转换单独保留，不能把不含逐点时间或无 IMU 的雷达直接选为 LIO。LIO初始化时须静止，IMU使用m/s²和rad/s；核心会拒绝不满足静止初始化／输入时序的消息。栅格仅由有效观测射线生成，NaN 未观测区保持未知；点云体素端点保守扩张，不能从 PCD 的空白区域推断可通行。以上路线均需现场确认扫描高度对低矮障碍的覆盖。

版本依据：ROS 官方列出 [Humble Jammy 的 ARM64 包](https://github.com/ros2/ros2_documentation/blob/humble/source/Installation/Ubuntu-Install-Debs.rst)；[NVIDIA JetPack 6.0 说明](https://developer.nvidia.com/embedded/jetpack-sdk-60)记载 Ubuntu 22.04，但不据此猜现场 JetPack 版本；[Ubuntu Jammy Open3D 0.14.1 包](https://packages.ubuntu.com/jammy/python3-open3d)提供 ARM64/Python 3.10 构建。本轮尚未在 Orin 验证其运行与性能。

### 2. 配置与构建（目标 Orin，同一仓库）

执行简短入口中的 `prepare --route ...` 后，编辑唯一现场配置：

```bash
nano artifacts/orin-navigation/site.json
```

配置模板是 [`navigation_orin.json`](../../integration/bindu_runtime/config/navigation/navigation_orin.json)。不用改历史 Airy 配方。JSON 不允许注释；保留 `null` 就会明确报缺项，不应填示例零值通过检查。

- `hardware`：填写实际设备和驱动包；`device/sensor_ip/host_ip` 仅记录连接事实，不自动修改设备或网络。源码没有通用传感器启动命令，按厂家驱动文档在独立终端启动，只开传感器。
- `driver_setup`：驱动为本机独立工作区时，填写其绝对 `install/local_setup.bash`；系统安装包则保留 `null`。脚本在清除旧覆盖层后加载该路径，不使用其他机器的库。
- `scan_topic/odom_topic/points_topic/imu_topic`：填写真实的绝对话题名；二维只要求前两项。`namespace` 控制本套算法输出，默认地图话题 `/bindu_navigation/navigation/map`。`tf_topic/tf_static_topic` 必须和设备使用同一 TF 图及 ROS 域。
- `frames`、`base_from_scan`：frame 名不带开头 `/`；外参按 **xyz 米＋xyzw 单位四元数**填写实测值。外部模型／驱动负责发布传感器 TF；集中配置校验其与实测 TF 一致，不自动发布猜测外参。LIO 另填 `imu_from_lidar/imu_from_base`，保证同一运动链；高度切片当前按**雷达坐标系**，必须实测填写。
- `tf_authorities.odom_to_base/base_to_scan`：二维填写实际发布节点全名，例如从 `ros2 node list` 读取的名称；LIO 的 odom 项固定写 `bindu_lio`，并关闭外部 odom→base 发布者。不是让用户填写示例节点名。
- `map.prefix`：本机新批次绝对路径且不带扩展名；`map.yaml/map.pcd` 在保存成功后再填写。`checks/grid` 是软件诊断／栅格参数候选值，不代表实测设备频率、精度或障碍间距。
- `footprint/limits/base`：机器人轮廓、线／角速度与加速度、接口与看门狗保持待标定。即使填写这些字段，当前 `unimplemented` 底盘阶段也不会解锁；不猜轮径、轮距、协议或额定速度。

执行 `tools/run_navigation.sh build`。成功判据：二维 `Summary: 7 packages finished`，LIO 为8包；文件在专用 `build/install/log`，不污染根目录现有构建。部署工具设置 `BINDU_NAVIGATION_ONLY=1`，按 [REP-149 条件依赖](https://docs.ros.org/en/independent/api/rep/html/rep-0149.html)排除可选能力；不设置时原完整依赖保留。不要在本流程运行全仓 `rosdep install --from-paths src` 或完整构建，也不要用导航专用环境启动遥操作／取物示例。需要 rosdep 时仅对已选七／八包路径操作。

成功后运行 `check --stage config`。失败时按输出修正对应字段；它不会因没接底盘而阻塞传感器阶段，但导航阶段始终报告底盘缺项。源码变化需重新 `build`；换路线需新的工作目录。LIO 构建吃内存，默认单编译任务；型号／内存未知时不自动创建 swap 或调电源模式。

### 3. 只读查看传感器（终端B）

先在终端A按实际驱动文档启动传感器及既有只读里程计发布，不启动会自动使能／回零的控制器。在终端B：

```bash
cd /home/wxb/bindu
source artifacts/orin-navigation/env.sh
ros2 topic list -t
ros2 node list
tools/run_navigation.sh check --stage sensors
```

成功判据：输出 `PASS: sensors`，扫描及 odom 的源时间持续增加；LIO 启动前检查原始点云和 IMU，启动后再检查它生成的 scan／odom。预检检查依赖、内容／frame、源时间与时效、采集时刻 TF、重复 TF 与来源；不申请租约、发布命令或调用运动服务。它不启动缺失驱动、不做时间／网络调整，也不能证明雷达已覆盖所有障碍。

TF 唯一责任：二维建图由 SLAM Toolbox 发布 map→odom，定位时只由 AMCL 发布；二维 odom→base 由真实底盘／融合里程计发布。LIO 建图由 FAST-LIO 发布 map→odom 和 odom→base；LIO 定位改由 ICP 发布 map→odom，FAST-LIO 保留 odom→base。base→传感器始终由已标定的模型／设备发布。本入口不额外发布同一静态 TF。不得同时运行两套建图／定位或外部恒等 map→odom。启动前若已有 map→odom，先确认并退出旧算法。

常见错误：`NO_ADVANCING_DATA/STREAM_STALE` 查驱动、话题、ROS_DOMAIN_ID、QoS及源时钟；`SOURCE_TIME_INVALID/REGRESSION` 查驱动时基和跨机同步；`TF_*` 查 frame、发布者全名、重复父节点及时间链；`DEPENDENCY_MISSING` 安装所选路线或配置正确的驱动覆盖层。不要用更大的超时、接收时间戳或伪里程计掩盖问题。

### 4. 人工低速建图与保存（终端A/B/C）

`start --phase mapping` 只启动算法。终端B运行 `rviz`，配置由现场话题生成到 `artifacts/orin-navigation/navigation.rviz`；默认 fixed frame 为 map。算法未启动前看传感器可在 RViz 把 Fixed Frame 临时切到 base；建图时恢复 map。成功判据：扫描与墙面对应、TF 连贯、地图随有效观测增长，无坐标跳跃。

首次移动按人工低速建图准备，**不做自主探索**。底盘适配前只允许静止传感器／地图观察；后续经单独现场授权，使用已验收的人工低速控制与实体急停，缓慢走通路线并回环。Bindu 人工速度入口 `tools/run_navigation.sh manual` 目前明确返回 `BASE_ADAPTER_REQUIRED`，不发送 `/cmd_vel`。不要拿脚本成功启动代替人工控制可用或停车验证。

另开终端C执行 `check --stage mapping`，通过后在机器人已由人工确认静止时执行 `save-map`。二维保存 `.yaml/.pgm`（或实际图像格式）和 `.posegraph/.data`；LIO保存 `.pcd` 和同坐标 `.yaml/.pgm`。保存服务超时或文件部分缺失时保留现场日志与部分产物，不自动重试覆盖。成功判据：命令返回0，YAML引用的图像存在且非空；LIO还须有PCD。位姿图用于继续建图，AMCL重载使用栅格地图。

### 5. 重启、重载定位与到点／取消／停止

确认人工已停稳，再 Ctrl+C 退出建图。把 `site.json` 的 `map.yaml` 指向刚保存的绝对 YAML 路径；LIO 再填写同批 `map.pcd`。机器或节点重启后重新启动实际传感器与里程计、执行 `check --stage sensors`，然后 `start --phase localization`。**不自动恢复旧目标、租约、初值或运动。** 终端B运行 `rviz`，用 `2D Pose Estimate` 在已知位置给近似初始方向；其实际话题为配置 namespace 下的 `initialpose`。不要加载另一个软件使用的全局 `/initialpose`。LIO的ICP不能从任意未知位置全局重定位。

终端C执行 `check --stage localization`。成功判据：保存地图正确加载，scan与墙面重合，map→odom来自本阶段算法且持续有效，真实／LIO odom继续更新。缺图像报 `MAP_IMAGE_REQUIRED`；无近似初值／配准失败会缺有效 map→odom；先解决原因再进入运动阶段。

本轮运动接口占位，以下是**明确阻塞入口**，不是可用的真机命令：

```bash
tools/run_navigation.sh check --stage navigation
tools/run_navigation.sh start --phase navigation
tools/run_navigation.sh navigate
tools/run_navigation.sh cancel
tools/run_navigation.sh stop
```

均报告缺底盘适配；`cancel/stop` 不会假装车辆已停。现有真实机器人由其已验收的人工停车／实体急停处理，Ctrl+C退出算法不等于硬件停车。RViz配置不提供绕过执行层的Nav2 Goal工具。

现有通用执行装配仍拒绝real，Profile加载还要求非空关节组；后续须补底盘专用profile与执行装配，不能填虚拟关节伪造反馈。待适配后必须接通的链路为：传感器 → scan／odom → TF → 建图／定位 → 每目标独立 Nav2 会话与发布者身份校验 → `NavigationVelocity` → `MotionCommand`（租约／epoch／revision／源时间／有效期）→ 公共 `Executor/RobotIO` → Tracer适配器 → 实测位姿／速度／故障与停止反馈。不得改 `navigation_physics.launch.py` 开关借用 `external_simulation`。原 Nav2 会话已补 Humble 单个progress_checker、结果字段与 Twist 来源时间兼容；原生入口对固定GID、单次UUID绑定、源时间和旧会话隔离的保护保留，RMW不提供源时间时拒绝。该兼容检查不等于完整Humble Nav2或Tracer闭环验收。

现场后续须实现并逐项验收：底盘协议与单位／frame、命令超时看门狗、控制权、实测停车判据、取消先关闭速度入口再请求停止与后端取消；雷达／定位／反馈断流、命令过期均停止且未确认停稳时失败锁定。底盘反馈不能用命令回显或LIO定位代替。重启后须重新预检、定位并显式提交新目标。验收顺序为**建图 → 保存 → 重启 → 重载定位 → 到点 → 取消 → 各类断流停车 → 显式恢复**；还需记录真机时间、位移和反馈，不能引用x86／Isaac历史结果作为通过证据。

### 验证入口

```bash
# 无设备的单元与源时间／执行保护回归（先加载专用env.sh）
python3 -m tests unit test_orin_navigation
python3 -m tests unit test_navigation
# 有Docker的开发机，独立Ubuntu22.04/Humble容器；不会启动真机或改宿主软件
bash tools/deployment/validate_humble_navigation.sh
```

ROS替身预检另外覆盖正常输入、扫描／odom断流、旧／冻结时间戳、错误frame和重复TF。实际测试范围和本轮结果见[当前验证说明](../../../运行与验证.md#orin-导航专用验证)；Orin构建、传感器标定和Tracer现场闭环仍待验收。

## 编译与运行

在仓库根目录执行；先准备 ROS 2、colcon、C++17 和 [包声明的 ROS 依赖](../../../运行与验证.md#环境准备与构建)。Humble 环境将 `jazzy` 换为 `humble`。

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-up-to bindu_navigation --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE="$(command -v python3)" -DPYTHON_EXECUTABLE="$(command -v python3)"
source install/local_setup.bash
ros2 pkg executables bindu_navigation
python3 -m tests unit test_airy_scan
```

可执行列表应包含 `nav2_velocity_identity` 与 `airy_scan_bridge`。前者需由 Nav2 会话绑定目标身份后使用，后者需有效传感器参数与 TF；不能仅启动程序就认为导航可用。

要运行含任务／执行链的协议模拟，先准备 [运行装配依赖](../../integration/bindu_runtime/README.md#编译与运行)，再执行：

```bash
colcon build --base-paths src --packages-up-to bindu_runtime --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE="$(command -v python3)" -DPYTHON_EXECUTABLE="$(command -v python3)"
source install/local_setup.bash
python3 -m tests integration navigation \
  --output "artifacts/navigation-check-$(date +%Y%m%d-%H%M%S)"
```

验证器自行启动并收尾模拟节点和导航协议替身，不使用真实底盘或实际 Nav2 规划。实际 Nav2／Isaac 运行需额外安装导航与仿真依赖，按 [房间导航步骤](#真实-nav2-与-isaac-房间导航) 启动；真实雷达和外参按 [Airy 接入](#二代轮式-airy-导航传感器预适配) 配置。

## 职责与边界

- 根据目标身份、发布进程、序号、时间戳和坐标系检查速度输入，避免旧会话或新进程的速度混入当前控制。
- 校验地图位姿、到站范围与停稳反馈；导航速度经控制权和共享执行入口送到底盘。
- 分别维护协议模拟基线与实际 Nav2／传感器适配。Nav2、SLAM、AMCL 是装配使用的外部能力。
- 已有仿真验证；真实雷达、外参、底盘参数和长期运行仍需现场验证。

## 源码导航

| 入口 | 内容 |
|---|---|
| [navigation.py](bindu_navigation/navigation.py) | NavigationConfig、站点、NavigationGate 与 SessionVelocityFence |
| [simulated.py](bindu_navigation/simulated.py) | 导航协议模拟实现 |
| [native/nav2_velocity_identity.cpp](native/nav2_velocity_identity.cpp) | 原生 Nav2 速度来源身份检查 |
| [airy_scan.py](bindu_navigation/airy_scan.py) / [scripts/airy_scan_bridge](scripts/airy_scan_bridge) | Airy 点云到导航扫描的转换与节点入口 |
| [CMakeLists.txt](CMakeLists.txt) | Python 与原生组件的安装和依赖 |

## 接入与验证

ROS 会话入口：[runtime/nav2_session.py](../../integration/bindu_runtime/bindu_runtime/nav2_session.py)、[runtime/navigation.py](../../integration/bindu_runtime/bindu_runtime/navigation.py)。传感器参数示例：[navigation_sensors.json](../../integration/bindu_runtime/config/navigation/navigation_sensors.json)。

- 单元检查：[导航](../../../tests/unit/test_navigation.py)、[Airy 扫描](../../../tests/unit/test_airy_scan.py)。
- 集成检查：[导航协议](../../../tests/integration/validate_navigation.py)、[Nav2 身份](../../../tests/integration/validate_nav2_identity.py)、[Nav2 物理场景](../../../tests/physics/validate_nav2_physics.py)。各入口需要相应 ROS／仿真环境。
- 配置和运行：[导航适配验证入口](#导航适配验证入口)、[真实 Nav2 与 Isaac 房间导航](#真实-nav2-与-isaac-房间导航)、[Airy 预适配](#二代轮式-airy-导航传感器预适配)。

## 关联模块

[三维定位](../bindu_lio/README.md) · [共享执行](../../control/bindu_execution/README.md) · [设备与里程计](../../hardware/bindu_hardware/README.md) · [运行装配](../../integration/bindu_runtime/README.md)

## 运行与专项验证

以下命令和项目路径均相对于仓库根目录。带日期内容仅代表当时的专项实验，现行控制规则以动作执行契约和统一部署为准。

### 导航适配验证入口

首批导航适配保留任务的 `navigate(port, context, site)` 接口；选择 `bindu_runtime.navigation:Nav2Navigation` 时，通过 `nav2_msgs/action/NavigateToPose` 调用后端，目标速度经已有租约、revision和执行器到模拟底盘。默认启动仍使用原 `SimNavigation`。

在按[环境准备与构建](../../../运行与验证.md#环境准备与构建)完成依赖安装/构建的 Ubuntu ROS 环境中运行：

```bash
python3 -m tests integration navigation --output artifacts/navigation-check
```

该验证器自行启动、收尾隔离命名空间的任务/执行/记录节点和 `tests/support/navigation_peer.py` 协议替身，覆盖往返、取消、断流、旧目标消息、来源变化、定位失效、错误成功结果和停稳检查。它使用真实 ROS Action 通信，但没有路径规划、障碍物、SLAM或物理底盘模型，不能作为真实 Nav2 导航验收。

2026-09-18 在 Ubuntu/Jazzy、bindu Conda Python 3.12 环境完成13包构建，97项模块及16项导航场景通过；原有16 ROS、14 Pi、11 VR场景回归通过。nav2_msgs使用1.3.13。首轮2项因测试源停止更新而报定位过期；测试源改串行回调后完整通过，超时阈值未放宽。模拟结果不代表现场稳定性。

随后按差速底盘要求校验：修复模拟器将机体前进量直接累加为全局x的问题，改为平面x/y/yaw积分并补齐二维反馈。新增圆弧/倒车、原地转向后前进、二维往返和错误朝向成功检查。此处是无打滑的理想运动学，仍未模拟轮地接触或真实定位；`ExecutionState`新增`base_y`后须重建所有依赖它的ROS包。临时G1模型基准及实机差异见[架构实现边界](../../../软件架构设计.md#854-当前代码组织与替换边界)。

差速校验在独立Ubuntu/Jazzy工作区完成13包重建、100项模块、18项导航和16项通用ROS场景，全部通过；Pi/VR端到端场景沿用上一批证据，本批未重跑。

- 站点与门控参数见 [`navigation_sim.json`](../../integration/bindu_runtime/config/navigation/navigation_sim.json)，只包含测试地图版本和测试站点；重复站点ID、非法位姿/参数拒绝加载，不能直接换成未经坐标核对的历史航点。
- `navigation_backend` 指定后端命名空间，接口为 `navigate_to_pose`、`velocity`、`pose`。速度使用 [`NavigationVelocity`](../../shared/bindu_interfaces/msg/NavigationVelocity.msg)，必须在命令产生时附实际Action目标UUID、来源进程实例、单调序号、源时间及机体frame。定位使用 [`NavigationPose`](../../shared/bindu_interfaces/msg/NavigationPose.msg)，保留测量时间、地图版本与全局frame；后续定位/TF适配不得用接收时间刷新旧数据。
- **未修改的 Nav2 `/cmd_vel` 不能直接接入本批入口。** 它缺少目标身份，不能由普通转发节点在接收时补当前目标ID。下文的独立物理导航入口通过每目标独立进程、固定发布者GID和一次性UUID绑定完成该桥接；本节协议替身测试仍不包含真实规划。
- 到点需要后端Action成功、全局位姿误差达标、底盘反馈持续停稳；取消先关速度入口，再停止执行并取消后端。停稳或取消无法确认时明确失败。输入使用best-effort有界队列，Action/执行服务保留可靠通信；丢失输入触发短期有效期和门控超时。
- 底盘独立速度/加速度参数及停止语义见[执行契约](../../../动作执行契约.md#2-输入模式必须显式选择)。真机底盘/雷达尚未接入；Isaac中的真实导航、建图和定位见下文独立入口。动作块适配和手部扩展继续暂缓。



### 真实 Nav2 与 Isaac 房间导航

`navigation_physics.launch.py`独立装配现有任务、租约执行与记录节点；真实Nav2 1.3.13使用NavFn、Regulated Pure Pursuit与路径失效后重规划的行为树。它不导入G1模型或关节名。本体参数集中在[`navigation_g1_fixture.json`](../../../tests/config/navigation/navigation_g1_fixture.json)，房间和站点分别来自`navigation_room.json`、`navigation_room_sites.json`。替换机器人需提供匹配的URDF/物理设备适配、执行profile、关节姿态、碰撞轮廓、坐标系、雷达外参与速度限制；不能只换网格后沿用旧轮廓和标定。

导航前临时G1以站立、双臂收拢姿态初始化，并检查姿态位置/速度反馈；移动期间持续检查姿态。这里尚未验证从任意姿态起身收臂的动作，也不包含手指驱动或自碰撞检查。房间为可复现的USD几何墙体、桌子与障碍物，机器人复用固定版本开源资产，无需下载额外场景。360线水平雷达直接查询PhysX碰撞。本节默认基线使用理想物理里程计、预制地图和恒等`map→odom`；下节的轮式里程计模式关闭这两个预制输出，接入实际建图/定位算法。

在Ubuntu/Jazzy已安装`ros-jazzy-navigation2`和`ros-jazzy-nav2-regulated-pure-pursuit-controller`、完成工作区重建及物理资产准备后：

```bash
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=126 ROS_LOCALHOST_ONLY=1
export RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS
# 终端1：GUI物理场景；开始独立实验前重启Isaac，避免复用旧执行器身份。
tools/run_g1_isaac.sh --namespace /bindu_navigation \
  --navigation-scene src/integration/bindu_runtime/config/navigation/navigation_room.json \
  --navigation-robot tests/config/navigation/navigation_g1_fixture.json \
  --output artifacts/navigation-room-scene
# 终端2：加载相同ROS环境与域，等待物理场景READY后启动。
ros2 launch bindu_runtime navigation_physics.launch.py recording_mode:=normal namespace:=/bindu_navigation \
  profile:="$PWD/artifacts/g1-physics-model/g1_physics_sim.json" \
  navigation_robot:="$PWD/tests/config/navigation/navigation_g1_fixture.json" \
  navigation_config:="$PWD/src/integration/bindu_runtime/config/navigation/navigation_room_sites.json" \
  output:="$PWD/artifacts/navigation-room-run" run_id:=navigation_room
# 终端3：同一ROS环境与域。测试结果目录必须是新的。
python3 -m tests.physics.validate_nav2_physics --output artifacts/navigation-room-tests \
  --sessions artifacts/navigation-room-run
```

每个目标消耗独立预热的Nav2进程和命名空间，C++入口逐包校验RMW发布者GID、源时间戳，一次性绑定该次实际Action UUID，退休后不复用进程。未就绪时拒绝目标，准备期间不运动；本版以启动开销换取旧目标隔离。速度仍经公共执行契约，不直接驱动模拟轮子。原生来源/重放/不可重绑检查可用`tests/integration/validate_nav2_identity.py --binary install/bindu_navigation/lib/bindu_navigation/nav2_velocity_identity --output artifacts/nav2-identity-check`单独运行。

2026-09-22分批通过正常往返、取消、动态障碍强制改道、道路堵塞、定位断流、控制器进程退出六类场景；最终改道路径由上侧y≈1.19m切到下侧y≈−1.23m，实际到桌边/返回误差58.0/31.2mm。119项模块、3项原生身份检查与6项旧导航回归通过。结果索引见该批`verification.json`，最终专项为`route-tests/results.json`，轨迹为`trajectory.png`。这是分批证据，不是所有场景一次连续全绿；早期失败均保留。

修复了任务停稳判据未读取profile容差、PhysX休眠保留旧底盘速度、生命周期查询丢响应后无限等待的问题；休眠反馈依据PhysX原生状态置零，原始速度保留，不放宽反馈期限。另行复测旧基础物理入口时，GUI/无头均在低位腿关节目标处因速度未停稳超时，**本轮基础物理10项未全部通过，原因未定位**。站立导航成功不能覆盖该姿态下的失败；自碰撞、任意姿态起身、真实传感器/定位仍未验收。

本机依赖隔离解包在`artifacts/nav2-physics-2026-09-22/deps`，未修改系统Nav2安装；机器专用环境见该批`env.sh`，不能直接复制到新机器。原始扫描、物理反馈、动作与命令、路径、会话配置及失败日志均保留在该批目录；数据忽略入库，尚未远端备份。



### 二代轮式 Airy 导航传感器预适配

2026-10-12依据本机`../二代轮式/robot_tools_package/`的2026-09-20快照准备二代导航接入。该资料记录Orin上的速腾Airy以ROS 2 Humble在域42发布`/rslidar_points`，点云frame为`rslidar`；控制链路使用域0。雷达配置的`imu_port: 0`，没有可直接接入当前FAST-LIO的IMU证据，也没有确认逐点时间字段。资料中的工控机`ros2_shm_control`仅覆盖48通道双臂／躯干／手控制；Bindu既有`PositionMapping`只映射其中16个具名关节，不是导航底盘驱动。这些是打包时的源码事实，现场正在运行的版本仍需核对。

新增`bindu_navigation`的`airy_scan_bridge`：在源域读取PointCloud2，以显式`base_from_lidar`外参投影到水平面并按底盘坐标高度切片，仅把有界`LaserScan`和`base→scan`静态TF发布到控制域。它不订阅或发布运动命令，也不把原始点云搬入控制域。输出保留采集时间；frame、字段布局、过期／倒退时间、非法外参与过稀扫描均拒绝。未观测角度为NaN未知，不标为可通行。原始点云在扫描期间未作逐点去畸变，此入口只供二维SLAM／AMCL候选链路验证，不等于三维FAST-LIO接通或真实避障验收。

[`navigation_wheel_gen2_airy.json`](../../integration/bindu_runtime/config/robots/wheel_gen2/navigation_wheel_gen2_airy.json)记录快照中的话题、frame和域；`base_frame`、`base_from_lidar`、切片高度保留`null`，配置检查会报`AIRY_CALIBRATION_REQUIRED`。必须根据真机模型、安装标定和现场扫描填入后才可启动。安装包内检查入口为`ros2 run bindu_navigation airy_scan_bridge --config /绝对路径/已标定配置.json --check`；检查成功后去掉`--check`才运行仅传感器桥接。扫描输出默认为`/bindu_navigation/navigation/scan`，合成水平扫描frame为`wheel_gen2_nav_scan`，与目标Nav2命名空间的`tf_static`配套；实际名称和跨机时钟还需现场核对。Nav2接入此扫描时必须启用`require_scan:=true`，使断流不能靠旧定位继续运行。

本地验证：Jazzy目标包隔离构建、5项点云／配置检查、17项既有导航回归及两个隔离ROS域的合成点云→扫描＋静态TF联调通过；Ubuntu22.04/Humble无网络只读容器中，`bindu_interfaces`和`bindu_navigation`两包隔离构建、同5项检查及跨域扫描／TF联调也通过。另用Python3.10完成语法检查。这些是合成数据验证，不代表真机导航可运行。

2026-10-12真机传感器检查：经S600访问Orin，先只读盘点，再短时运行Airy单个驱动节点（域42）采样，随后确认进程已退出；未启动运控、底盘或导航。Airy测量／配置UDP端口6699／7788均收到来自`192.168.1.200`的1248字节包。10秒内订阅到99帧`/rslidar_points`和22331个`/rslidar_packets`；点云frame为`rslidar`，组织尺寸96×900、每点16字节，字段为float32 `x/y/z/intensity`，一次样本的消息时间戳距Orin当前时间约0.146秒。这确认了话题和基础字段与二维桥接输入匹配，尚未用实测安装外参运行桥接或验证其导航扫描。Airy配置`imu_port: 0`，实际点云也没有逐点`time`字段，不能直接接入当前要求IMU和逐点时间的FAST-LIO。S600与Orin的短时SSH时钟差估计约0.1秒，仍需正式同步与时间链核对。

用户确认Airy位于机器人胸口。资料中T1/T2为可动躯干轴；现场模型仅有一个`torso_joint`，没有第二躯干关节或雷达安装link。因此，在证明雷达刚性固定于底盘或躯干受控锁定并持续监测前，当前桥接的固定`base→scan`外参不能用于躯干活动时的建图/导航；后续需要实测胸口外参、完整运动链和与点云时间对齐的关节反馈，或采用确实固定的装配。高位扫描还需单独检查低矮障碍覆盖。`navigation_wheel_gen2_airy.json`的外参、高度切片继续保持`null`并拒绝启动。

其余传感器盘点：Orin枚举到D455、D435I及两台普通相机，已有`realsense2_camera` ROS包，但检查时没有相机／IMU话题；不能把D435I的IMU硬件等同于已标定且时间同步的LIO输入。超声波`/dev/ultrasonic`存在，一轮8路Modbus读取均有应答，其中3路测得距离、5路返回“无目标或未接探头”，单轮不能判定后5路故障；当前无超声波ROS发布节点。底盘速度／里程计／停车反馈、雷达外参、地图和低位避障仍未验收；`robot_wheel_gen2.json`保持不完整并拒绝启动。

2026-10-12补查关节现场链路：S600的`~/Documents/SOEM-demo/src/ros2_shm_control/install_ros`已安装`soem_shm_control`，含`ros2_shm_bridge`和`shm_simulator`；仅检查系统默认ROS环境会漏掉此覆盖层。现场检查时RT控制器、桥接均未运行，默认共享内存和域0关节反馈话题不存在。桥接源码在100Hz计时回调中写共享内存命令槽，即使没有有效上游目标也写`enable=0`；资料中的实机RT启动参数`--auto-start`会复位并移动手指，因此不能将实机RT＋桥接启动当作只读状态查询。本机文档足以确定48通道名称、ROS话题、状态门槛和组包协议，但不等于已取得实际关节测量或授权运动。

新增`soem_joint_feedback`只读ROS入口：订阅已有`/robot/joint_states`和`/robot/shm_status`，仅在48通道完整、有限、时间单调且≤200ms、状态`alive=1/ready=1/error=0/hand_held_mask=0`时，按现有`PositionMapping`输出16轴到`/bindu_wheel_gen2/joint_states`，保留来源消息时间；状态异常、反馈过期或重名通道均不发布。入口无`RobotCommand`发布者，不装配运动执行器。编译后运行命令为`ros2 run bindu_runtime soem_joint_feedback --mapping /绝对路径/position_mapping_wheel_gen2.json`；此入口本身不会启动RT或底层桥接，现场尚未持续部署。模型缺第二躯干轴和雷达安装链，得到角度也不能直接宣称胸口雷达动态TF可用。

验证：本地Jazzy重建`bindu_hardware`和`bindu_runtime`、8项相关映射／反馈测试通过；首轮`bindu_runtime --symlink-install`因旧生成安装目录冲突失败，清理该包忽略入库的`build/install`生成目录后重建通过。本地隔离ROS域合成48轴→16轴通过，检查命令话题发布者数为0。S600的Humble上另以独立ROS域和独立共享内存启动**无硬件**`shm_simulator`、已安装底层桥接及临时Bindu只读入口，收到16轴`JointState`，命令话题发布者数为0；进程、临时文件及共享内存均已清理。未启动EtherCAT RT、未接触真机关节，命令/停止/接管后端仍未实现。
