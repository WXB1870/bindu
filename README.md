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
| IK、语音交互、遥操作 | 已预留模块目录 |
| 数据记录 | 已实现异步事件与状态记录；图像同步及完整训练数据管线待实现 |
| 自适应插值 | 高频执行与真机适配待实现 |

## 工程入口

仓库根目录同时是 ROS 2 工作区，共包含 11 个可构建包。

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

`build/`、`install/`、`log/` 由构建生成；`artifacts/` 保存实验结果与临时依赖，均不提交 Git。`tools/` 目前只有占位文件。历史代码、图片、研究计划与过程记录在主开发工作区单独维护。

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

已验证：11 包构建、28 项模块测试、14 个 Pi 集成场景；原有 15 个 ROS 场景在先前版本通过，最近的 Pi 启动诊断修正后未重跑。模拟测试不等于真机或实时性能验收。验证环境使用 NumPy 1.26.4、PyZMQ 26.4.0；若需要隔离安装该 PyZMQ 版本，可执行：

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

## 开发与文档

- [软件架构](软件架构设计.md)：模块职责、替换接口与实现边界。
- [动作执行契约](动作执行契约.md)：目标、控制权、时间轴与停止语义。
- [协作规范](AGENTS.md)：开发验证要求和文件管理规则。

本地开发后提交并推送；目标机在保留自身未提交修改的前提下使用 `git pull --ff-only`，重新构建，并用 `git rev-parse HEAD` 核对版本。已有独立 Git 历史的工作区需先完成迁移。

## License

见 [MIT License](LICENSE)。历史第三方参考资产遵循各自来源许可。
