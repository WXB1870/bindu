# Bindu

基于 **ROS 2** 的模块化机器人控制系统，面向长程导航与移动操作。首期目标是让轮式人形根据语言指令寻找饮料、导航抓取并持物返回，后续扩展至双足人形。

通过公共接口连接任务、感知、规划、VLA、遥操作和设备控制，使算法与机器人硬件可以独立替换。VLA 在机器人端仅部署客户端，模型推理在远端运行。

## 功能状态

项目处于模拟骨架与能力接入阶段，完整移动抓取和真机控制尚未完成。

| 模块 | 当前状态 |
|---|---|
| 任务与执行 | 已实现模拟任务流程、控制权、在线关节目标、取消与异常处理 |
| Pi 客户端 | 已实现两种历史 ZMQ 模式、动作映射、时效检查和诊断；已通过模拟服务测试 |
| 设备适配 | 已实现模拟底盘、关节和灵巧手驱动；真实设备待接入 |
| 导航 | 已实现站点、Nav2 Action 适配与受控速度入口；当前用协议替身验证，真实 Nav2/建图待接入 |
| 感知、规划 | 已有模拟实现，真实算法待接入 |
| VR 遥操作与 IK | 已接 v3.4 手柄输入、相对控制和连续 IK；单臂模拟末端/状态可视反馈已通过 ROS/Vuer 验证 |
| 语音交互 | 已预留模块目录 |
| 数据记录 | 已实现异步事件与状态记录；图像同步及完整训练数据管线待实现 |
| 共享执行层插值 | C++ 统一数值内核已接入；在线流、定时多轴轨迹与动作块已通过 Ubuntu ROS/Pi/VR 模拟回归 |

## 工程入口

仓库根目录同时是 ROS 2 工作区，共包含 13 个可构建包。

```text
bindu/
├── src/
│   ├── shared/          # 公共契约、ROS 消息与服务
│   ├── tasks/           # 任务流程与场景状态
│   ├── control/         # 控制权、时序与轨迹执行
│   ├── hardware/        # 设备驱动、命令路由与反馈汇总
│   ├── data/            # 异步数据记录
│   ├── integration/     # ROS 节点、配置与 launch
│   └── capabilities/    # IK、VLA、感知、导航、规划、交互、遥操作
├── tests/               # 模块测试与跨进程模拟验证
├── tools/               # 开发与部署工具预留入口
├── README.md
├── AGENTS.md
├── 软件架构设计.md
├── 动作执行契约.md
└── LICENSE
```

`build/`、`install/`、`log/` 由构建生成；`artifacts/` 保存实验结果与临时依赖，均不提交 Git。`tools/requirements-teleop.txt` 保存可选 VR/IK 验证依赖。历史代码、图片、研究计划与过程记录在主开发工作区单独维护。

## 环境要求

- 已验证开发环境：Ubuntu 24.04、ROS 2 Jazzy、Python 3.12、x86_64。
- 构建工具：colcon、rosdep；运行依赖包括 NumPy 和 PyZMQ。
- Jetson Orin / Ubuntu 22.04 的部署兼容性尚未验证。

下列命令假定 ROS 2 Jazzy、colcon 和已初始化的 rosdep 可用。新终端只加载当前使用的工作区，避免混用旧安装环境。

### Conda 兼容性

2026年9月18日，在 Ubuntu 24.04 / x86_64 上验证了 **Conda Python 3.12.14 + 系统 ROS 2 Jazzy**：13包构建、71项模块、16个 ROS、14个 Pi 和9个 VR 场景通过。首轮一个场景在故障注入前因反馈过期结束，原用例连续3次及全组复测通过；不等同长期稳定性或 Orin 验收。系统 ROS、消息库及构建辅助包仍由 apt 提供，这不是独立的全 Conda ROS 发行版。

- 使用单独 Conda 环境，测试采用 setuptools 68.1.2；IK/VR 固定依赖见 `tools/requirements-teleop.txt`。在环境内追加 `/usr/lib/python3/dist-packages` 供 ROS 构建辅助包使用，禁用用户级 site-packages，避免混入个人环境。
- 先加载系统 ROS，再按[VR 入口](#vr-遥操作运行入口)设置 Pinocchio 的 Python 与动态库搜索顺序，依赖根改为 Conda 的 `lib/python3.12/site-packages`。仅执行 `conda activate` 不够：误加载系统旧 Pinocchio 与环境 NumPy 2 的组合曾导致段错误。
- 用 Conda 的 `python /usr/bin/colcon build` 构建，并显式设置 CMake 的 `Python3_EXECUTABLE` / `PYTHON_EXECUTABLE` 为该环境解释器；使用独立源码、build/install，重新编译原生扩展。核对生成的节点入口和实际进程解释器，不能仅检查 shell 是否激活。系统 `ros2` 启动器仍可使用系统 Python。

## 快速开始

### 1. 获取与构建

```bash
git clone https://github.com/WXB1870/bindu.git
cd bindu
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
colcon build --base-paths src --symlink-install
source install/setup.bash
```

### 2. 启动模拟系统

在工作区终端执行：

```bash
export ROS_DOMAIN_ID=116 ROS_LOCALHOST_ONLY=1
ros2 launch bindu_runtime skeleton.launch.py
```

另开一个终端，进入同一工作区并提交模拟取物任务：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=116 ROS_LOCALHOST_ONLY=1
ros2 action send_goal /bindu_sim/tasks/fetch_drink bindu_interfaces/action/FetchDrink \
  "{task_id: demo_1, object_id: drink, strategy: planner}" --feedback
```

`strategy` 支持 `planner` 和 `chunk` 两种模拟策略。一次成功任务后，模拟场景保留持物状态；新的独立实验需重启模拟系统。以上流程不连接真实机器人。

## 测试

在已加载 ROS 与工作区环境的终端执行；测试输出目录应为新的实验批次。

```bash
python3 -m unittest discover -s tests -p 'test_*.py' -v
python3 tests/validate_ros.py --output artifacts/ros-check
python3 tests/validate_pi.py --output artifacts/pi-check
```

集成验证会启动并收尾各自的模拟进程。Pi 验证自带回环服务、合成图像和关节观测，无需模型权重或真实设备。

此前 Python 后端已验证：13 包构建、64 项模块测试（无跳过）、16 个 ROS 集成场景、14 个 Pi 场景及 9 个遥操作场景全部通过；包含动作块时序/版本、受控停止，以及真实 Vuer 服务接收合成输入。尚未做头显现场联调。模拟测试不等于真机或实时性能验收。Pi 首版使用 NumPy 1.26.4；本轮 VR/IK 与回归使用 NumPy 2.3.5，PyZMQ 均为 26.4.0；若需要隔离安装该 PyZMQ 版本，可执行：

```bash
python3 -m pip install --target artifacts/pi-deps pyzmq==26.4.0
export PYTHONPATH="$PWD/artifacts/pi-deps:$PYTHONPATH"
```

## 导航适配验证入口

首批导航适配保留任务的 `navigate(port, context, site)` 接口；选择 `bindu_runtime.navigation:Nav2Navigation` 时，通过 `nav2_msgs/action/NavigateToPose` 调用后端，目标速度经已有租约、revision和执行器到模拟底盘。默认启动仍使用原 `SimNavigation`。

在按快速开始完成依赖安装/构建的 Ubuntu ROS 环境中运行：

```bash
python3 tests/validate_navigation.py --output artifacts/navigation-check
```

该验证器自行启动、收尾隔离命名空间的任务/执行/记录节点和 `tests/navigation_peer.py` 协议替身，覆盖往返、取消、断流、旧目标消息、来源变化、定位失效、错误成功结果和停稳检查。它使用真实 ROS Action 通信，但没有路径规划、障碍物、SLAM或物理底盘模型，不能作为真实 Nav2 导航验收。

2026-09-18 在 Ubuntu/Jazzy、bindu Conda Python 3.12 环境完成13包构建，97项模块及16项导航场景通过；原有16 ROS、14 Pi、11 VR场景回归通过。nav2_msgs使用1.3.13。首轮2项因测试源停止更新而报定位过期；测试源改串行回调后完整通过，超时阈值未放宽。模拟结果不代表现场稳定性。

- 站点与门控参数见 [`navigation_sim.json`](src/integration/bindu_runtime/config/navigation_sim.json)，只包含测试地图版本和测试站点；重复站点ID、非法位姿/参数拒绝加载，不能直接换成未经坐标核对的历史航点。
- `navigation_backend` 指定后端命名空间，接口为 `navigate_to_pose`、`velocity`、`pose`。速度使用 [`NavigationVelocity`](src/shared/bindu_interfaces/msg/NavigationVelocity.msg)，必须在命令产生时附实际Action目标UUID、来源进程实例、单调序号、源时间及机体frame。定位使用 [`NavigationPose`](src/shared/bindu_interfaces/msg/NavigationPose.msg)，保留测量时间、地图版本与全局frame；后续定位/TF适配不得用接收时间刷新旧数据。
- **未修改的 Nav2 `/cmd_vel` 不能直接接入本批入口。** 它缺少目标身份，不能由普通转发节点在接收时补当前目标ID。真实Nav2命令来源隔离/目标关联及TF定位适配属于下一批；本批未声称已完成该桥接。
- 到点需要后端Action成功、全局位姿误差达标、底盘反馈持续停稳；取消先关速度入口，再停止执行并取消后端。停稳或取消无法确认时明确失败。输入使用best-effort有界队列，Action/执行服务保留可靠通信；丢失输入触发短期有效期和门控超时。
- 底盘独立速度/加速度参数及停止语义见[执行契约](动作执行契约.md#2-输入模式必须显式选择)。真实底盘、雷达、地图、全局定位与建图尚未接入；动作块适配和手部扩展继续暂缓。

## Pi 客户端运行入口

- 核心代码：[`bindu_vla/pi`](src/capabilities/bindu_vla/bindu_vla/pi/)。
- 配置示例：[`pi_loopback.json`](src/integration/bindu_runtime/config/pi_loopback.json)。
- `pubsub`：机器人发布观测、订阅动作。
- `pull`：机器人回复模型端的观测请求，同时订阅动作。

手工联调时，先配置服务端点、关节映射及观测输入，再启动客户端：

```bash
ros2 launch bindu_runtime skeleton.launch.py pi_enabled:=true
```

在同一 ROS 域、已加载工作区的另一终端中查询就绪状态并发起会话：

```bash
ros2 service call /bindu_sim/vla/pi/ready std_srvs/srv/Trigger '{}'
ros2 action send_goal /bindu_sim/vla/pi/session bindu_interfaces/action/PiSession \
  "{task_id: pi_demo, prompt: 'pick water', resource_group: arm, duration: 5.0}" --feedback
```

缺少新鲜观测时拒绝启动，原因通过 `PI_GOAL_REJECTED` 事件和节点日志给出。会话持续时间结束表示控制会话停止，不代表抓取成功。目前每个会话控制一个关节组，默认使用模拟设备，ACT 未接入。

## VR 遥操作运行入口

核心代码位于 [`bindu_teleoperation`](src/capabilities/bindu_teleoperation/bindu_teleoperation/) 和 [`bindu_kinematics`](src/capabilities/bindu_kinematics/bindu_kinematics/)。本批接入 v3.4 的 `CONTROLLER_MOVE` 输入、OpenXR 坐标转换和相对控制，使用独立进程进行 Pinocchio FK / CasADi 连续 IK，经公共执行服务驱动 **7 轴左臂模拟器**。输入端与机器人驱动分离，后续仍以 VR 为遥操作入口。

示例配置 `teleop_v34.json` 使用 `mapping_mode: "robot_base"`，重新握持时维持空间方向；`operator_yaw_rad` 用于启动前水平朝向校准。旧配置未提供模式时保留 `v34_anchor` 行为；需要旧操作习惯可显式选择该模式并将偏角设为0。坐标和旋转定义见[架构中的单臂映射规则](软件架构设计.md#854-当前代码组织与替换边界)。

可选依赖在 Ubuntu 24.04 / Python 3.12 / x86_64 验证；Orin 尚未验证。先按快速开始构建并加载 ROS 与工作区，再在当前工作区隔离安装依赖。下面的库路径用于避免 ROS 自带 EigenPy 与 Pinocchio wheel 混用，仅影响当前终端：

```bash
python3 -m pip install --target artifacts/teleop-deps -r tools/requirements-teleop.txt
export BINDU_TELEOP_DEPS="$PWD/artifacts/teleop-deps"
export PYTHONPATH="$BINDU_TELEOP_DEPS:$BINDU_TELEOP_DEPS/cmeel.prefix/lib/python3.12/site-packages:$PYTHONPATH"
export LD_LIBRARY_PATH="$BINDU_TELEOP_DEPS/cmeel.prefix/lib:$LD_LIBRARY_PATH"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
export ROS_DOMAIN_ID=119 ROS_LOCALHOST_ONLY=1
python3 tests/validate_teleop.py --output artifacts/teleop-check
```

专项验证启动自己的模拟进程：覆盖启动入口、合成手柄事件经真实 Vuer WebSocket 服务接入、暂停/重接、取消、断流、追踪无效、连接身份改变、不可达目标和记录器退出，包含显示状态与显示进程退出隔离检查。2026-09-18 在 Ubuntu / bindu Conda 环境重建3包，85项模块及最终11个遥操作场景通过；首轮2项在故障注入前等待显示超时，补齐观察节点就绪前置检查后全组通过，保留首轮失败记录。真实头显的浏览器、TLS、跟踪质量与操控感受仍需现场联调。

连接头显时，使用本机有效的 TLS 证书和私钥路径，并让头显能访问输入服务地址。证书不随代码分发；默认仅监听 `127.0.0.1`，下面显式开启局域网输入：

```bash
ros2 launch bindu_runtime teleop.launch.py vr_enabled:=true host:=0.0.0.0 \
  cert_file:=/绝对路径/cert.pem key_file:=/绝对路径/key.pem
```

`vr_enabled:=true` 同时启动只读 `teleop_display` 节点。页面新增：

- 青色线框为请求的末端目标，橙色球为模拟关节反馈经 FK 算出的末端；RGB 轴分别表示末端局部 XYZ。
- 状态面板显示请求的跟随/暂停模式、结束或拒绝原因，以及当前目标与反馈的位姿误差；误差不是 IK 求解残差，也不是实际到达确认。
- 目标、输入或反馈过期时不继续显示有效目标/误差；显示节点超过 0.6 秒未更新时清除两组姿态并显示 `DISPLAY_STALE`。整个 Vuer 服务或网络断开时页面可能停留在最后一帧，须检查连接状态。
- 图形为 `robot_base` 坐标示意，转到 Y 向上的查看坐标并向前平移 1 米；不是机器人与操作者房间的空间配准。模型未加入碰撞几何。

观察节点以约 10 Hz 独立计算反馈 FK，使用 best-effort 订阅和容量为 1 的非阻塞显示队列，不持有执行控制权。字体由服务本地提供，需安装依赖清单中的 Pillow 11.3.0；请使用输入主机的页面，不使用远端托管的 Vuer 页面。单独启动输入节点时，观察节点须使用相同 namespace、profile、run_id 和 teleop_config。

头显访问输入主机的 HTTPS 8012 端口并进入 VR。先确认页面显示 `WAITING` 且橙色反馈末端已出现，再在另一个已加载相同 ROS 域和工作区的终端查询就绪并开启会话。显示是可选观察者，`teleop/ready` 不等待它；观察者晚启动或重启时需新会话恢复完整模式信息：

```bash
ros2 service call /bindu_sim/teleop/ready std_srvs/srv/Trigger '{}'
ros2 action send_goal /bindu_sim/teleop/session bindu_interfaces/action/TeleopSession \
  "{task_id: vr_demo, resource_group: left_arm, duration: 60.0}" --feedback
```

- 每次会话先松开左握持键，再握持超过 1 秒进入跟随；松开暂停，重新握持后以当前实测关节 FK 和当前 VR 姿态重新建立基准。左 B 停止会话，接收端锁存此停止输入，重新连接浏览器后方可再次启动。
- `init` 按键目前明确返回 `VR_INIT_NOT_CONFIGURED`；尚未有核实过的初始姿态，不自动回零。扳机值会记录，尚未控制灵巧手。右臂、双臂/手部并发、图像显示及训练数据同步未接入本批。
- [遥操作配置](src/integration/bindu_runtime/config/teleop_v34.json)与[本体配置](src/integration/bindu_runtime/config/huawei_v34_left_sim.json)定义资源、具名关节、时间限制、比例和模型。旧模型只保留运动学/惯性文本，未包含碰撞几何；非活动关节固定在配置值，不能当作当前本体标定。
- VR 时间戳为输入服务收到事件的时刻，未提供头显采样时间或网络时延估计；轮询不会刷新旧帧时间。输入、模式、IK 残差/耗时、接受目标与实测反馈异步记录，示教图像和完整训练数据集尚未实现。
- 会话到时表示控制结束，不代表抓取成功。运行层继续拒绝真实设备模式；本批没有连接电机、部署真机服务或承诺 IK 达到 100 Hz。

## 共享执行层验证

关节参考统一由 [C++ 内核](src/control/bindu_execution/native/interpolation.hpp)生成；[Python 接口](src/control/bindu_execution/bindu_execution/interpolation.py)只负责对象/时间轴适配，不再保留第二套曲线数学实现。在线目标用自适应策略，定时 P2P、导数轨迹和动作块用时序策略，共用曲线求值、限幅及制动。ROS 执行器继续管理 revision、控制权和实测到达确认，语义见[动作执行契约](动作执行契约.md#2-输入模式必须显式选择)。原实验目录已合入 `native/`，历史源码仍只读。

构建需要 C++17 编译器和与运行环境匹配的 Python 开发头文件。`colcon build` 自动构建原生扩展；模块缺失时直接报导入错误，不静默回退到 Python。更新后须重建并加载工作区，不能复制另一架构或 Python 版本的 `.so`。无 ROS 的本地测试可先构建：

```bash
(cd src/control/bindu_execution && python3 setup.py build_ext --inplace)
```

加载工作区后运行模块和 ROS 回归，`timed_chunks` 包含动作块原始时间轴、旧 revision 拒绝和减速取消：

```bash
python3 -m unittest discover -s tests -p 'test_*.py'
python3 tests/validate_ros.py --case online_targets --case timed_chunks --output artifacts/execution-check
```

本轮本地验证：14组纯 C++ 测试及 ASan/UBSan 通过，覆盖七轴定时 P2P、分轴限幅、带导数轨迹、过期动作块/制动尾段、在线预览一致性、模式接续和运动中停止；另包含6种输入频率、30组非定时单轴 P2P、34,408个随机流采样。68项 Python 调用与执行回归通过，真实 IK 类因缺 Pinocchio/CasADi 跳过；源码分发包在干净目录重新编译和加载也通过验证。9月18日 Ubuntu 原生后端实测：13包构建、71项模块测试（含真实 IK）、14组 C++、16个 ROS、14个 Pi 和9个 VR 场景通过。修复了 colcon symlink-install 下 C++ 源码路径解析；Pi 首轮一次就绪服务回复超时，原用例连续3次及全组复测通过，未改判定条件。该偶发通信问题保留记录，未进行真机或 Orin 验收。

纯 C++ 测试不依赖 ROS、Python 或历史源码：

```bash
mkdir -p artifacts/native-test
c++ -std=c++17 -O2 -Wall -Wextra -pedantic \
  -I src/control/bindu_execution/native \
  src/control/bindu_execution/native/adaptive_stream.cpp \
  src/control/bindu_execution/native/interpolation.cpp \
  tests/test_adaptive_stream.cpp -o artifacts/native-test/test_native
artifacts/native-test/test_native
```

保留历史源码的主工作区可运行以下对照。工具分别测旧 C++、标量自适应、新原生在线/五次曲线策略，输出 CSV/JSON；原生策略耗时包含 Python 绑定开销，七轴项目使用实际在线策略。当前本机离线正弦 RMS 为0.099219 rad，旧内核为0.097335 rad；固定时序策略不会为了降低误差自行改变轨迹时间。测试值不等于机器人定位精度或端到端延迟，内核仍有动态分配，Python/ROS 串行执行壳仍在，不宣称硬实时。

```bash
python3 tools/compare_interpolators.py --legacy-root 历史代码/legs_necks_control \
  --output artifacts/interpolation-comparison
```

## 开发与文档

- [软件架构](软件架构设计.md)：模块职责、替换接口与实现边界。
- [动作执行契约](动作执行契约.md)：目标、控制权、时间轴与停止语义。
- [协作规范](AGENTS.md)：开发验证要求和文件管理规则。

本地开发后提交并推送；目标机在保留自身未提交修改的前提下使用 `git pull --ff-only`，重新构建，并用 `git rev-parse HEAD` 核对版本。已有独立 Git 历史的工作区需先完成迁移。

## License

见 [MIT License](LICENSE)。历史第三方参考资产遵循各自来源许可。
