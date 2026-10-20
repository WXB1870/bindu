# bindu_description：本体模型与资产

[项目模块导航](../../../README.md#模块快速导航) · [编译与运行](#编译与运行) · [职责与边界](#职责与边界) · [源码导航](#源码导航) · [接入与验证](#接入与验证) · [关联模块](#关联模块)

通过 ament_cmake 安装 URDF、网格和配套资产，供显示、运动学与仿真使用。本包不包含设备驱动。

## 编译与运行

在仓库根目录执行；需要 ROS 2、colcon 和 ament_cmake，见 [环境准备](../../../运行与验证.md#环境准备与构建)。Humble 环境将 `jazzy` 换为 `humble`。

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-up-to bindu_description --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE="$(command -v python3)" -DPYTHON_EXECUTABLE="$(command -v python3)"
source install/local_setup.bash
ls "$(ros2 pkg prefix --share bindu_description)/urdf"
python3 -c "from xml.etree import ElementTree as ET; print(ET.parse('src/hardware/bindu_description/urdf/snowbot_control.urdf').getroot().tag)"
```

构建会安装现有模型及网格，不需要重新下载原始资产。本包没有独立节点；模型由运动学、`robot_state_publisher` 或浏览器显示加载。完整模型显示和反馈链使用 [运行装配](../../integration/bindu_runtime/README.md#编译与运行)；上述检查只验证模型文件，不启动设备。

## 职责与边界

- 分别提供控制／运动学模型与显示模型；选择模型时同时核对关节名、坐标系、单位及配置中的可控自由度。
- G1 为临时模型，Huawei v3.4 为历史模型适配；这些资产不说明实验室已有相应真机能力。
- Snowbot 模型支持当前仿真与求解，实体夹持、关节映射和未标定自由度仍以现场确认结果为准。
- 外部网格与模型按各自许可使用，来源和资产处理信息随包保留。

## 源码导航

| 入口 | 内容 |
|---|---|
| [urdf/](urdf) | 全部机器人描述 |
| [snowbot_control.urdf](urdf/snowbot_control.urdf) / [snowbot_display.urdf](urdf/snowbot_display.urdf) | Snowbot 控制与显示入口 |
| [g1_provisional.urdf](urdf/g1_provisional.urdf) / [g1_display.urdf](urdf/g1_display.urdf) | 临时 G1 模型 |
| [huawei_v34_kinematic.urdf](urdf/huawei_v34_kinematic.urdf) / [huawei_v34_display.urdf](urdf/huawei_v34_display.urdf) | 历史 v3.4 模型 |
| [g1.assets.json](g1.assets.json) / [huawei_v34.assets.json](huawei_v34.assets.json) | 资产清单与来源信息 |
| [LICENSE](LICENSE)、[SNOWBOT_ASSETS_NOTICE](SNOWBOT_ASSETS_NOTICE)、[HUAWEI_ASSETS_NOTICE](HUAWEI_ASSETS_NOTICE) | 模型与外部资产使用说明 |

## 接入与验证

模型准备入口：[G1](../../../tools/prepare_g1_description.py)、[Snowbot](../../../tools/prepare_snowbot_description.py)、[Huawei](../../../tools/prepare_huawei_description.py)。修改前确认生成来源，避免只修改会被重新生成的文件。

- 本体选择与启动校验：[robot_setup.py](../../integration/bindu_runtime/bindu_runtime/robot_setup.py)。
- 模型检查：[G1 资产](../../../tests/unit/test_g1_assets.py)、[G1 运动学](../../../tests/unit/test_g1_model.py)、[Snowbot](../../../tests/unit/test_snowbot_description.py)、[Huawei](../../../tests/unit/test_huawei_description.py)。
- 模型用途与运行步骤：[机器人模型与资产](#机器人模型与资产)。

## 关联模块

[设备适配](../bindu_hardware/README.md) · [运动学](../../capabilities/bindu_kinematics/README.md) · [运行装配](../../integration/bindu_runtime/README.md)

## 运行与专项验证

以下命令和项目路径均相对于仓库根目录。带日期内容仅代表当时的专项实验，现行控制规则以动作执行契约和统一部署为准。

### 机器人模型与资产

`src/hardware/bindu_description/` 统一保存随源码分发的本体 URDF、共享网格及来源信息；运动学能力包只读取配置指定的模型路径，不再携带模型。`artifacts/` 保存本机生成的物理仿真资产与缓存，不提交 Git。以下表格统一说明其用途。生成模型由脚本维护，修改派生规则后统一重新生成，避免单独手改 URDF。

| 模型 | 用途与派生关系 | 网格位置 |
|---|---|---|
| [`snowbot.urdf`](urdf/snowbot.urdf) | 完整导入：修复两个力传感器固定关节重名，重写网格 URI；保留四轮转动及视觉、碰撞几何，共 24 个活动关节 | `bindu_description/meshes/snowbot/` |
| [`snowbot_display.urdf`](urdf/snowbot_display.urdf) | 从完整模型固定四轮，保留几何与其余 20 轴；供静止底盘显示及 `robot_state_publisher` 使用 | 与完整模型共用同一目录 |
| [`snowbot_control.urdf`](urdf/snowbot_control.urdf) | 从 display 模型移除视觉、碰撞几何，保留相同的 20 轴运动学结构；供 IK／双臂 QP 使用 | 无网格引用 |
| [`g1_provisional.urdf`](urdf/g1_provisional.urdf) | 上游 G1 的临时 20 轴运动学派生模型；省略轮子和手爪子树、视觉及碰撞几何，供运动学模拟／IK 使用 | 无网格引用 |
| [`g1_display.urdf`](urdf/g1_display.urdf) | 保留相同20轴控制结构，恢复固定上游版本的显示／碰撞几何；不启用新的IK碰撞策略 | `bindu_description/meshes/g1/` |
| [`huawei_v34_kinematic.urdf`](urdf/huawei_v34_kinematic.urdf) | 历史v3.4的90连杆／19轴运动学基线，供旧单臂FK／IK及回归测试；保留原参数，省略几何 | 无网格引用 |
| [`huawei_v34_display.urdf`](urdf/huawei_v34_display.urdf) | 与运动学模型相同关节结构，恢复源模型88个显示／81个碰撞几何；不代表启用了旧单臂IK碰撞策略 | `bindu_description/meshes/huawei_v34/` |
| `artifacts/g1-physics-model/g1_physics.urdf` | 从G1显示模型增加合成差速轮与支撑；供Isaac物理仿真使用，需按当前来源重新生成 | 引用资产包的`meshes/g1/`和`usd/g1/`，不再复制原始资产 |

Snowbot 的三份 URDF 是同一机器人针对不同用途的生成结果。31 个 STL 仅保存一份，完整模型与显示模型通过 `package://bindu_description/meshes/snowbot/…` 引用；`bindu_description` 的安装规则一起安装 `urdf/` 与 `meshes/`。文件存在不等于几何可用于碰撞：源资产中的 `head_link_1.STL` 为空网格，相关检查边界见[雪铲几何验证](#单杆雪铲的握姿与工作空间2026-10-08)。

Snowbot 来源及压缩包 SHA256 在 [`SNOWBOT_ASSETS_NOTICE`](SNOWBOT_ASSETS_NOTICE) 中维护；[`prepare_snowbot_description.py`](../../../tools/prepare_snowbot_description.py) 校验该指纹并一次生成三份 URDF 和共享网格。原 ZIP 由用户提供，不随仓库分发。重新生成命令：

```bash
python3 tools/prepare_snowbot_description.py /path/to/snowbot_20260728_首次全身.zip
# 仅检查生成结果时，可追加 --output /tmp/bindu-snowbot-description
```

Huawei原URDF来自用户历史v3.4工程，STL来自旧电脑备份的`Music/huawei_new/meshes`。101个源文件引用按SHA256合并为94份网格；显示模型统一使用`package://bindu_description/meshes/huawei_v34/…`，文件名别名、源／生成指纹保存在[`huawei_v34.assets.json`](huawei_v34.assets.json)。原件保留在历史资料区，不修改或清理。生成脚本校验固定URDF指纹和已记录网格指纹，拒绝混用其他版本；资产未附许可声明，见[`HUAWEI_ASSETS_NOTICE`](HUAWEI_ASSETS_NOTICE)。

```bash
python3 tools/prepare_huawei_description.py /path/to/huawei_urdf_wheel.urdf \
  --mesh-root /path/to/huawei_new
# --mesh-root 指向含 meshes/ 的目录；可追加 --output /tmp/bindu-huawei-description 检查派生结果
```

旧单臂启动入口及观察节点的默认模型目录已改为安装后的`bindu_description/urdf/`；运行配置仍显式选择Huawei、G1或Snowbot模型。旧`bindu_kinematics/models/`路径不再提供副本。此次只统一资产与引用，未改变控制策略、关节参数或默认本体选择；有碰撞网格不等于现有IK或物理仿真已完成碰撞／接触验收。

G1固定上游版本、源URDF指纹及控制模型派生规则由[`derive_g1_model.py`](../../../tools/derive_g1_model.py)维护。此前物理准备脚本将源网格、完整上游USD与生成缓存混放在`artifacts/`；现在86个源网格引用合并为78份共享文件，保存在`meshes/g1/`，显示所需26份USD／纹理依赖在`usd/g1/`，与控制／显示模型及[`g1.assets.json`](g1.assets.json)一起安装。未使用的上游ROS、传感器、物理变体和演示环境不随当前包复制；历史源目录保持原样。

```bash
# 需要更新运动学派生模型及profile时执行；随后更新显示资产：
python3 tools/derive_g1_model.py /path/to/galbot_one_golf_description/urdf/galbot_one_golf.urdf
python3 tools/prepare_g1_description.py --source /path/to/galbot_one_golf_description
# 没有本地上游目录时，省略--source，下载并校验固定提交的压缩包。
# 当前源码已含源网格/USD；准备物理缓存无需再次下载：
python3 tools/prepare_g1_physics.py
# 使用安装后的资产包：追加 --description /path/to/install/bindu_description/share/bindu_description
```

物理缓存只含生成URDF、profile、参数和来源记录，网格／USD引用统一资产包。启动器在初始化Isaac之前检查控制／显示模型、源资产、profile、物理参数、生成脚本与输出指纹；旧缓存、来源变化或资产位置移动必须重新生成，不能静默继续使用旧关节结构。资产目录遗留未登记网格／USD也会拒绝；升级时清理对应包的旧安装目录后重建。原物理缓存曾固定`leg_joint4`，当前控制模型为可动回转轴，旧目录和历史失败仍保留。G1派生资产保留上游Apache-2.0许可；Snowbot／Huawei来源的许可边界分别以随包NOTICE为准，不能套用根目录MIT或G1许可。



### 单杆雪铲的握姿与工作空间（2026-10-08）

名义模型、握点与几何参数由`src/integration/bindu_runtime/config/robots/snowbot/`及`src/hardware/bindu_description/`维护；当前设备配置与名义实验姿态不能互相代替。`tools/plan_shovel_motion.py`负责工具位姿输入实验与几何评分，`tests.replay.validate_vr_axes`核对真实接合姿态的方向映射。原2026-10-08工作空间扫描、姿态比较和离线结果保留在本地`过程记录/2026-10-08.md`，不作为真机可达域验收。
