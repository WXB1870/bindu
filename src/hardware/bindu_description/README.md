# bindu_description：本体模型与资产

[项目模块导航](../../../README.md#模块快速导航) · [职责与边界](#职责与边界) · [源码导航](#源码导航) · [接入与验证](#接入与验证) · [关联模块](#关联模块)

通过 ament_cmake 安装 URDF、网格和配套资产，供显示、运动学与仿真使用。本包不包含设备驱动。

## 职责与边界

- 分别提供控制／运动学模型与显示模型；选择模型时同时核对关节名、坐标系、单位及配置中的可控自由度。
- G1 为临时模型，Huawei v3.4 为历史模型适配；这些资产不说明实验室已有相应真机能力。
- Snowbot 模型支持当前仿真与求解，实体夹持、关节映射和未标定自由度仍以现场确认结果为准。
- 外部网格与模型按各自许可使用，来源和资产处理信息随包保留。

## 源码导航

| 入口 | 内容 |
|---|---|
| [urdf/](urdf/) | 全部机器人描述 |
| [snowbot_control.urdf](urdf/snowbot_control.urdf) / [snowbot_display.urdf](urdf/snowbot_display.urdf) | Snowbot 控制与显示入口 |
| [g1_provisional.urdf](urdf/g1_provisional.urdf) / [g1_display.urdf](urdf/g1_display.urdf) | 临时 G1 模型 |
| [huawei_v34_kinematic.urdf](urdf/huawei_v34_kinematic.urdf) / [huawei_v34_display.urdf](urdf/huawei_v34_display.urdf) | 历史 v3.4 模型 |
| [g1.assets.json](g1.assets.json) / [huawei_v34.assets.json](huawei_v34.assets.json) | 资产清单与来源信息 |
| [LICENSE](LICENSE)、[SNOWBOT_ASSETS_NOTICE](SNOWBOT_ASSETS_NOTICE)、[HUAWEI_ASSETS_NOTICE](HUAWEI_ASSETS_NOTICE) | 模型与外部资产使用说明 |

## 接入与验证

模型准备入口：[G1](../../../tools/prepare_g1_description.py)、[Snowbot](../../../tools/prepare_snowbot_description.py)、[Huawei](../../../tools/prepare_huawei_description.py)。修改前确认生成来源，避免只修改会被重新生成的文件。

- 本体选择与启动校验：[robot_setup.py](../../integration/bindu_runtime/bindu_runtime/robot_setup.py)。
- 模型检查：[G1 资产](../../../tests/test_g1_assets.py)、[G1 运动学](../../../tests/test_g1_model.py)、[Snowbot](../../../tests/test_snowbot_description.py)、[Huawei](../../../tests/test_huawei_description.py)。
- 模型用途与运行步骤：[机器人模型与资产](../../../运行与验证.md#机器人模型与资产)。

## 关联模块

[设备适配](../bindu_hardware/README.md) · [运动学](../../capabilities/bindu_kinematics/README.md) · [运行装配](../../integration/bindu_runtime/README.md)
