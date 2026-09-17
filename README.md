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
| 导航、感知、规划 | 已有模拟实现，真实算法待接入 |
| VR 遥操作与 IK | 已接 v3.4 手柄输入、相对控制和连续 IK；当前仅单臂模拟执行 |
| 语音交互 | 已预留模块目录 |
| 数据记录 | 已实现异步事件与状态记录；图像同步及完整训练数据管线待实现 |
| 共享执行层插值 | 在线目标、时序轨迹、动作块及减速停止已通过仿真；低延迟调优与真机实时适配待完成 |

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

共享执行层本轮已验证：13 包构建、64 项模块测试（无跳过）、16 个 ROS 集成场景、14 个 Pi 场景及 9 个遥操作场景全部通过；包含动作块时序/版本、受控停止，以及真实 Vuer 服务接收合成输入。尚未做头显现场联调。模拟测试不等于真机或实时性能验收。Pi 首版使用 NumPy 1.26.4；本轮 VR/IK 与回归使用 NumPy 2.3.5，PyZMQ 均为 26.4.0；若需要隔离安装该 PyZMQ 版本，可执行：

```bash
python3 -m pip install --target artifacts/pi-deps pyzmq==26.4.0
export PYTHONPATH="$PWD/artifacts/pi-deps:$PYTHONPATH"
```

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

专项验证启动自己的模拟进程：覆盖启动入口、合成手柄事件经真实 Vuer WebSocket 服务接入、暂停/重接、取消、断流、追踪无效、连接身份改变、不可达目标和记录器退出。真实头显的浏览器、TLS、跟踪质量与操控感受仍需现场联调。

连接头显时，使用本机有效的 TLS 证书和私钥路径，并让头显能访问输入服务地址。证书不随代码分发；默认仅监听 `127.0.0.1`，下面显式开启局域网输入：

```bash
ros2 launch bindu_runtime teleop.launch.py vr_enabled:=true host:=0.0.0.0 \
  cert_file:=/绝对路径/cert.pem key_file:=/绝对路径/key.pem
```

头显访问输入主机的 HTTPS 8012 端口并进入 VR。另一个已加载相同 ROS 域和工作区的终端查询就绪并开启会话：

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

VR 与 Pi 的单步目标、有限规划轨迹和动作块共用执行层曲线程序。输入时间、导数、版本和停止语义见[动作执行契约](动作执行契约.md#2-输入模式必须显式选择)。本次增加 ROS 消息字段，更新后需重新构建并加载工作区。执行参考的加速度/jerk 限制与设备跟踪限制分开配置；示例数值仅用于仿真。

加载工作区后运行模块和 ROS 回归，`timed_chunks` 包含动作块原始时间轴、旧 revision 拒绝和减速取消：

```bash
python3 -m unittest discover -s tests -p 'test_*.py'
python3 tests/validate_ros.py --case online_targets --case timed_chunks --output artifacts/execution-check
```

保留历史源码的主工作区可独立复现 C++/Python 对照；工具仅在指定输出目录编译测试程序、生成 CSV 和 JSON。新克隆不需要历史代码即可运行当前模块测试。

```bash
python3 tools/compare_interpolators.py --legacy-root 历史代码/legs_necks_control \
  --output artifacts/interpolation-comparison
```

当前 Python 方案优先满足整段约束和停止语义，离线正弦测试的跟随误差高于历史 C++ 内核；不能把“共用执行层”视为已完成 VR 跟手调优或真机实时验收。

低延迟优化候选位于 [experimental/adaptive_stream.cpp](src/control/bindu_execution/experimental/adaptive_stream.cpp)。它复用历史自适应内核的 jerk 建议计算，加入精确积分、整段限位与可制动检查、静止目标末段收敛、有效期内分段停止及异常时钟锁存。历史原件保持只读。候选只接受单轴位置目标，不接收目标速度/加速度，不负责动作块时间轴或控制权；尚未编入 ROS，现有运行后端不变。若后续采用，应接入共享执行层，避免在 VR/VLA 各加一套平滑器。

维护目标是一套正式执行模块，内部按在线目标、时序轨迹和动作块选择策略，共用约束与停止能力。当前 C++ 候选尚不能覆盖 Python 的多轴时序接口，因此保留用于对照；完成相同接口和模式衔接回归后再移除被替代的实现。历史只读资产不作为第二套运行实现。

冗余精简已移除 C++ 无用参数、无效赋值和额外四阶求导；静止捕获结束后复用已认证的保持状态，仍处理 TTL 和新目标。Python 曲线在采样与认证之间缓存共用 q/v/a/j 系数。8组场景的 Python/C++ 共9,600个采样与精简前逐点一致；独立 C++ 6组测试（含停稳后过期/恢复）及 ASan/UBSan 通过，本地 Python 61项通过，真实 IK 测试类因缺 Pinocchio/CasADi 跳过。本次未重跑 ROS 集成。

2026-09-17 本地 macOS ARM64 / Apple Clang 21 的同限幅对照（q ±1 rad、v 1.5 rad/s、a 20 rad/s²、jerk 400 rad/s³）中，候选消除了静止边界目标最后一秒的位置波动（旧版约 0.000764 rad），峰值由 1.000440 rad 收敛到 1 rad，断流后参考速度可归零。正弦 RMS 由旧版 0.097335 rad 变为 0.099219 rad（增加 1.9%；当前 Python 为 0.196798 rad），小幅往返 RMS 增加 6.1%。合成 6/14/9/11 ms 时间戳下按实际间隔积分，位置差商导出的速度/加速度/jerk 保持限幅。8 组对照中候选单轴计算 p99 为 0.67–5.00 µs，比旧 C++ 有额外开销；这不是 Orin、多轴或硬实时保证，代码仍有动态分配。

独立测试无需 ROS 或历史源码，包含非法输入、静止/边界、取消/过期、积分一致性、125 个制动初态和 8 组限幅下 34,408 个随机流采样。6 组测试及 AddressSanitizer/UndefinedBehaviorSanitizer 检查通过；本轮未重跑 ROS/VR/Pi 集成，也未连接头显或真机。上面的对照工具同时输出旧 C++、候选 C++ 与 Python 三组结果。

```bash
mkdir -p artifacts/adaptive-test
c++ -std=c++17 -O2 -Wall -Wextra -pedantic \
  -I src/control/bindu_execution/experimental \
  src/control/bindu_execution/experimental/adaptive_stream.cpp \
  tests/test_adaptive_stream.cpp -o artifacts/adaptive-test/test_adaptive_stream
artifacts/adaptive-test/test_adaptive_stream
```

## 开发与文档

- [软件架构](软件架构设计.md)：模块职责、替换接口与实现边界。
- [动作执行契约](动作执行契约.md)：目标、控制权、时间轴与停止语义。
- [协作规范](AGENTS.md)：开发验证要求和文件管理规则。

本地开发后提交并推送；目标机在保留自身未提交修改的前提下使用 `git pull --ff-only`，重新构建，并用 `git rev-parse HEAD` 核对版本。已有独立 Git 历史的工作区需先完成迁移。

## License

见 [MIT License](LICENSE)。历史第三方参考资产遵循各自来源许可。
