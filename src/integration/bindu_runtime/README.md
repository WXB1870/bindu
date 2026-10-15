# bindu_runtime：ROS 运行装配

[项目模块导航](../../../README.md#模块快速导航) · [职责与边界](#职责与边界) · [源码导航](#源码导航) · [接入与验证](#接入与验证) · [关联模块](#关联模块)

提供 ROS 2 节点、launch 与配置，把公共契约、执行库、设备和能力组合为可启动的运行实例。算法核心仍在各自模块中维护。

## 职责与边界

- 负责消息转换、依赖选择、设备／遥操作装配、本体校验与进程生命周期。
- 启动时明确仿真／真实环境，检查指令和反馈模式一致性；环境选择与设备后端、写权限分别配置。
- 当前真实模式主要覆盖 Snowbot 专用适配的只读接入，运动写入另有使能条件；不代表完整真机遥操作、Pi 或导航已验收。
- 配置选择不替代真实模型标定、设备能力核实和现场验证。

## 源码导航

| 入口 | 内容 |
|---|---|
| [setup.py](setup.py) | 全部 ROS 可执行入口与安装资源 |
| [plugins.py](bindu_runtime/plugins.py)、[device_assembly.py](bindu_runtime/device_assembly.py)、[simulation_devices.py](bindu_runtime/simulation_devices.py) | 能力选择、设备与模拟后端装配 |
| [teleop_assembly.py](bindu_runtime/teleop_assembly.py) / [teleop_deployment.py](bindu_runtime/teleop_deployment.py) | 遥操作依赖和部署配置 |
| [robot_setup.py](bindu_runtime/robot_setup.py) / [common.py](bindu_runtime/common.py) | 本体／自由度校验、ROS 转换与公共运行辅助 |
| [execution_node.py](bindu_runtime/execution_node.py) / [snowbot_adapter.py](bindu_runtime/snowbot_adapter.py) | 共享执行节点与厂家位置适配 |
| [bindu_runtime/](bindu_runtime/) | 遥操作、VR、Pi、导航、感知、任务、记录等节点源码 |
| [launch/](launch/) / [config/](config/) | 启动装配与配置全集 |

### 常用启动与配置入口

| 用途 | 启动文件 | 配置示例 |
|---|---|---|
| 环境选择与运行装配 | [runtime.launch.py](launch/runtime.launch.py) | [runtime_deployment.json](config/runtime_deployment.json) |
| 本体与遥操作模式选择 | [robot_teleop.launch.py](launch/robot_teleop.launch.py) | [robot_snowbot.json](config/robot_snowbot.json) |
| Snowbot 双臂工具仿真 | [snowbot_sim.launch.py](launch/snowbot_sim.launch.py) | [bimanual_snowbot_shovel.json](config/bimanual_snowbot_shovel.json) |
| Snowbot 厂家适配 | [snowbot_adapter.launch.py](launch/snowbot_adapter.launch.py) | [snowbot_vendor_position.json](config/snowbot_vendor_position.json) |
| 导航物理场景 | [navigation_physics.launch.py](launch/navigation_physics.launch.py) | [navigation_room.json](config/navigation_room.json) |
| 三维建图／地图定位 | [lio.launch.py](launch/lio.launch.py) / [localization.launch.py](launch/localization.launch.py) | [navigation_lio.yaml](config/navigation_lio.yaml) / [navigation_localization.yaml](config/navigation_localization.yaml) |

`runtime.launch.py` 读取部署配置并选择具体 launch；[teleop_robot_deployment.json](config/teleop_robot_deployment.json) 是选择 `robot_teleop.launch.py` 的部署示例。`robot_teleop.launch.py` 自身通过 `robot_config` 读取本体配置。上表配置为场景示例，所需参数及默认值见对应运行说明。

木质雪铲 PhysX 实验使用仓库级 [run_shovel_physics.sh](../../../tools/run_shovel_physics.sh) 和 [snowbot_shovel_physics.json](config/snowbot_shovel_physics.json)，具体反馈路径和模型限制见运行说明。

## 接入与验证

构建依赖、环境加载、启动参数和验证命令统一维护在 [运行与验证](../../../运行与验证.md)，请按对应场景使用；上表用于定位文件，不是可直接执行的命令清单。

- 环境切换：[统一仿真与真实模式选择](../../../运行与验证.md#统一仿真与真实模式选择)。
- 本体与装配检查：[本体配置](../../../tests/test_robot_configuration.py)、[模式](../../../tests/test_runtime_mode.py)、[遥操作装配](../../../tests/test_teleop_deployment.py)。
- ROS 检查：[示例任务](../../../tests/validate_ros.py)、[环境一致性](../../../tests/validate_runtime_mode.py)、[遥操作](../../../tests/validate_teleop.py)、[厂家适配](../../../tests/validate_snowbot_adapter.py)。验证脚本依赖相应已启动的测试装配。

## 关联模块

[公共契约](../../shared/bindu_contracts/README.md) · [ROS 接口](../../shared/bindu_interfaces/README.md) · [共享执行](../../control/bindu_execution/README.md) · [设备适配](../../hardware/bindu_hardware/README.md) · [遥操作](../../capabilities/bindu_teleoperation/README.md) · [全部模块](../../../README.md#模块快速导航)
