# Bindu

基于 **ROS 2** 的模块化机器人控制系统，面向长程导航与移动操作。首期目标是让轮式人形根据语言指令寻找饮料、导航抓取并持物返回，后续扩展至双足人形。

通过公共接口连接任务、感知、规划、VLA、遥操作和设备控制，使算法与机器人硬件可以独立替换。VLA 在机器人端仅部署客户端，模型推理在远端运行。

## 功能状态

当前以仿真验证和能力接入为主，完整移动抓取与真机控制尚未完成。

| 模块 | 当前能力与边界 |
|---|---|
| 任务与执行 | 模拟任务、控制权、C++ 插值、取消与异常处理；已做 Isaac 物理验证 |
| 导航 | 已接 Nav2、二维 SLAM/AMCL 和三维 FAST-LIO/ICP；真实雷达待接入，长期稳定性待验证 |
| VR 与 IK | 单臂相对控制、位姿滤波和局部自碰撞检查；合成输入闭环已验证，现场头显待联调 |
| VLA | Pi 远端推理客户端支持两种历史 ZMQ 模式；已通过模拟服务测试 |
| 设备与记录 | 模拟底盘、关节、灵巧手及异步日志；真实设备与完整训练数据管线待接入 |
| 其他能力 | 感知、规划已有模拟实现，语音预留；实际算法待接入 |

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
ros2 launch bindu_runtime skeleton.launch.py recording_mode:=normal
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

日志默认 `compact`；上述开发示例显式使用 `normal`。三档含义见[日志记录级别](运行与验证.md#日志记录级别)。

## 工程入口

仓库根目录也是 ROS 2 工作区；模块按职责组织：

```text
src/
├── shared/          # 公共契约、ROS 消息与服务
├── tasks/           # 任务流程与场景状态
├── control/         # 控制权、时序与轨迹执行
├── hardware/        # 设备驱动、反馈与本体描述
├── data/            # 异步数据记录
├── integration/     # ROS 节点、配置与 launch
└── capabilities/    # IK、VLA、感知、导航、规划、交互、遥操作
```

`tests/` 存放验证入口，`tools/` 存放开发与仿真工具。`build/`、`install/`、`log/`、`artifacts/` 为本地构建或实验产物，不提交 Git。

## 文档

- [运行与验证](运行与验证.md)：Conda 环境、日志、测试、G1/Isaac、导航、Pi、VR 的命令与验证边界。
- [软件架构](软件架构设计.md)：模块职责、替换接口与实现边界。
- [动作执行契约](动作执行契约.md)：目标、控制权、时间轴与停止语义。
- [协作规范](AGENTS.md)：开发验证要求和文件管理规则。

README 只保留项目入口与最小启动流程；专项命令维护在运行与验证文档，日常实验进展在本地进度与过程记录中维护。

## License

见 [MIT License](LICENSE)。第三方模块及参考资产遵循各自随附许可。
