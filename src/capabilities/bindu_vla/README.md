# bindu_vla：远端 Pi 推理客户端

[项目模块导航](../../../README.md#模块快速导航) · [职责与边界](#职责与边界) · [源码导航](#源码导航) · [接入与验证](#接入与验证) · [关联模块](#关联模块)

通过 ZMQ 与远端 Pi 服务交换观测和动作。本模块实现客户端适配，不包含本地 VLA 模型训练或推理。

## 职责与边界

- 兼容发布观测与模型端拉取观测两种历史通信方式，校验协议、观测身份、时效和动作数据。
- 将有效动作交给共享执行层，处理会话、动作接续和通信诊断；网络收发与执行周期隔离。
- Pi 客户端已有模拟服务测试；真实模型服务地址、协议联调及目标机性能仍待确认。ACT 尚未接入。
- 动作已发布、执行参考到达末点、设备真实到达是不同状态。

## 源码导航

| 入口 | 内容 |
|---|---|
| [pi/protocol.py](bindu_vla/pi/protocol.py) | ZMQ 帧编解码、协议与动作数据校验 |
| [pi/adapter.py](bindu_vla/pi/adapter.py) | 观测／动作适配、会话与时间处理 |
| [pi/transport.py](bindu_vla/pi/transport.py) | 非阻塞 ZMQ 收发与有界消息读取 |
| [simulated.py](bindu_vla/simulated.py) | 任务与客户端测试使用的模拟实现 |

## 接入与验证

ROS 节点为 [pi_node.py](../../integration/bindu_runtime/bindu_runtime/pi_node.py)，使用 [PiSession.action](../../shared/bindu_interfaces/action/PiSession.action) 管理会话。

- 配置示例：[pi_g1.json](../../integration/bindu_runtime/config/pi_g1.json)、[pi_loopback.json](../../integration/bindu_runtime/config/pi_loopback.json)。
- 单元检查：[test_pi.py](../../../tests/test_pi.py)。
- 模拟服务与 ROS 检查：[pi_peer.py](../../../tests/pi_peer.py)、[validate_pi.py](../../../tests/validate_pi.py)。模拟服务仅用于验证，不是实际模型服务。
- 依赖、协议与启动步骤：[Pi 客户端运行入口](../../../运行与验证.md#pi-客户端运行入口)。

## 关联模块

[公共契约](../../shared/bindu_contracts/README.md) · [共享执行](../../control/bindu_execution/README.md) · [数据记录](../../data/bindu_recording/README.md) · [运行装配](../../integration/bindu_runtime/README.md)
