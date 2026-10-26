# bindu_lio：三维里程计与地图定位

[项目模块导航](../../../README.md#模块快速导航) · [编译与运行](#编译与运行) · [职责与边界](#职责与边界) · [源码导航](#源码导航) · [接入与验证](#接入与验证) · [关联模块](#关联模块)

集成 FAST-LIO 雷达／IMU 估计、已有地图的 ICP 定位及导航扫描转换。算法进程与控制执行分开运行。

## 编译与运行

Orin／Humble导航专用部署见[导航新手流程](../bindu_navigation/README.md#orin-导航专用部署与新手操作)，可单独选择LIO、使用Ubuntu的ARM64 Open3D包并保存PCD与同坐标观测栅格；不要套用下文x86历史依赖目录或仿真外参。

在仓库根目录执行；需要 ROS 2、colcon、C++17、PCL、Eigen3、OpenMP 及 `package.xml` 中的 ROS 依赖，见 [环境准备](../../../运行与验证.md#环境准备与构建)。Humble 环境将 `jazzy` 换为 `humble`。

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-up-to bindu_lio --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE="$(command -v python3)" -DPYTHON_EXECUTABLE="$(command -v python3)"
source install/local_setup.bash
python3 -m tests integration lio \
  --binary "$PWD/install/bindu_lio/lib/bindu_lio/fast_lio" \
  --output "artifacts/lio-check-$(date +%Y%m%d-%H%M%S)"
```

上述验证自带合成点云／IMU并收尾进程，不连接真实雷达。常规建图由运行包提供 launch；先准备 [运行装配依赖](../../integration/bindu_runtime/README.md#依赖准备)，再执行：

```bash
colcon build --base-paths src --packages-up-to bindu_lio bindu_runtime --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE="$(command -v python3)" -DPYTHON_EXECUTABLE="$(command -v python3)"
source install/local_setup.bash
ros2 launch bindu_runtime lio.launch.py namespace:=/bindu_lio_test \
  sensor_config:="$PWD/tests/config/navigation/navigation_lio_fixture.json" \
  mode:=mapping save_map:="$PWD/artifacts/lio-room.pcd"
```

该配置仅为合成输入夹具，launch 不产生传感器数据，也不启动雷达驱动；无输入时不会完成建图。现场必须替换话题、坐标系、外参和点时间配置。有输入并完成建图后，另一已加载相同环境／ROS 域的终端可调用 `ros2 service call /bindu_lio_test/lio/save_map std_srvs/srv/Trigger '{}'` 保存 PCD，已有文件拒绝覆盖；Ctrl+C 收尾。

重载地图定位还需在独立运行环境安装 `python3 -m pip install --target artifacts/lio-python -r tools/requirements/requirements-navigation.txt`，仅在定位终端加入 `export PYTHONPATH="$PWD/artifacts/lio-python${PYTHONPATH:+:$PYTHONPATH}"`。随后使用 `mode:=localization pcd_map:=/绝对路径/map.pcd` 并提供初始位姿；完整步骤见 [三维导航入口](#历史三维导航迁移入口)，不要将 Open3D 依赖目录混入 colcon 构建环境。

## 职责与边界

- 维护雷达／IMU 输入、去畸变点云、里程计、地图保存与定位适配。
- 地图定位采用带初始位姿的两级 ICP，并检查配准质量和位姿变化；不提供任意位置的全局重定位。
- 需要正确的点时间、IMU 轴向、传感器外参与时间同步。Livox 转换入口不代替实体雷达驱动。
- 不直接发底盘命令。已有 Isaac 场景验证，真实传感器和长期运行尚未完成验收。

## 源码导航

| 入口 | 内容 |
|---|---|
| [src/lio_node.cpp](src/lio_node.cpp) | FAST-LIO 的 ROS 2 节点适配，安装为 `fast_lio` |
| [vendor/fast_lio/](vendor/fast_lio) | 保留的 FAST-LIO 估计与 ikd-Tree 实现 |
| [localization.py](bindu_lio/localization.py) / [lio_localization_node.py](bindu_lio/lio_localization_node.py) | ICP 配准与 `lio_localization` 节点 |
| [lio_scan_node.py](bindu_lio/lio_scan_node.py) | 去畸变点云转二维导航扫描，入口 `lio_scan` |
| [livox_input_node.py](bindu_lio/livox_input_node.py) | Livox 消息转换，入口 `livox_input` |
| [source_manifest.json](source_manifest.json) / [上游许可](vendor/fast_lio/LICENSE) | 迁移来源、指纹与 GPL 许可；vendor 内容按上游许可维护 |

## 接入与验证

启动入口：[lio.launch.py](../../integration/bindu_runtime/launch/lio.launch.py)、[localization.launch.py](../../integration/bindu_runtime/launch/localization.launch.py)。构建涉及 PCL／Eigen 等依赖，ICP 定位使用 Open3D；Livox 消息依赖按实际输入选择。

- 单元检查：[配准](../../../tests/unit/test_lio_localization.py)、[ROS 适配](../../../tests/unit/test_lio_ros_adapters.py)。
- 场景检查：[LIO](../../../tests/integration/validate_lio.py)、[地图定位](../../../tests/integration/validate_lio_localization.py)、[定位物理场景](../../../tests/physics/validate_localization_physics.py)。
- 来源、依赖、建图／地图重载与限制：[历史三维导航迁移入口](#历史三维导航迁移入口)。

## 关联模块

[导航](../bindu_navigation/README.md) · [设备与里程计](../../hardware/bindu_hardware/README.md) · [运行装配](../../integration/bindu_runtime/README.md)

## 运行与专项验证

以下命令和项目路径均相对于仓库根目录。带日期内容仅代表当时的专项实验，现行控制规则以动作执行契约和统一部署为准。

### Isaac 内的实际建图与地图定位

[`localization.launch.py`](../../integration/bindu_runtime/launch/localization.launch.py)分别装配 **SLAM Toolbox 2.8.5**（在线异步二维建图）和 **Nav2 AMCL 1.3.13**（重载栅格地图后的粒子滤波定位）。建图阶段由SLAM发布地图和`map→odom`；定位阶段由map_server发布已保存地图、AMCL发布`map→odom`。两种模式应互斥运行。算法参数在[`navigation_localization.yaml`](../../integration/bindu_runtime/config/navigation/navigation_localization.yaml)，不包含G1关节或模型路径。

Isaac的[`navigation_sensors.json`](../../integration/bindu_runtime/config/navigation/navigation_sensors.json)启用实测轮子转角积分，带人为设置的2%轮径、1%轮距误差和5mm标准差的射线距离噪声；这些是测试扰动，尚未按实机标定。此模式不发布预制地图或恒等`map→odom`。物理世界位姿只用于生成传感器射线、独立评估和执行层运动反馈，不提供给SLAM/AMCL作定位输入。轮转角每次采样必须变化小于π；当前实现处理±π回绕。沿用执行层墙上时钟，算法`use_sim_time=false`，不混用`/clock`。

在上一节环境基础上添加`ros-jazzy-slam-toolbox`、`ros-jazzy-nav2-amcl`和`ros-jazzy-nav2-map-server`，重建`bindu_hardware`与`bindu_runtime`。三个终端加载相同ROS、工作区和域环境，依次执行：

```bash
# 终端1：全新GUI场景，启用轮式里程计和噪声。
tools/run_g1_isaac.sh --namespace /bindu_navigation \
  --navigation-scene src/integration/bindu_runtime/config/navigation/navigation_room.json \
  --navigation-robot tests/config/navigation/navigation_g1_fixture.json \
  --navigation-sensors src/integration/bindu_runtime/config/navigation/navigation_sensors.json \
  --output artifacts/slam-scene
# 终端2：物理场景READY后，启用扫描新鲜度门控。
ros2 launch bindu_runtime navigation_physics.launch.py recording_mode:=normal namespace:=/bindu_navigation \
  profile:="$PWD/artifacts/g1-physics-model/g1_physics_sim.json" \
  navigation_robot:="$PWD/tests/config/navigation/navigation_g1_fixture.json" \
  navigation_config:="$PWD/src/integration/bindu_runtime/config/navigation/navigation_mapping_sites.json" \
  output:="$PWD/artifacts/slam-run" run_id:=slam_room require_scan:=true
# 终端3：自行启动SLAM，采集扫描/保存地图，再切换AMCL和执行导航检查。
python3 -m tests.physics.validate_localization_physics \
  --robot tests/config/navigation/navigation_g1_fixture.json \
  --sites src/integration/bindu_runtime/config/navigation/navigation_mapping_sites.json \
  --output artifacts/slam-tests
```

验证器通过公共执行契约发送受控合成遥操作路线，不是自主探索；用SLAM估计位姿引导绕房间一周，保存`room.yaml/room.pgm`与`room_graph.posegraph/.data`。随后从人为偏置的初始位姿启动AMCL，检查往返、前进时雷达断流停车及恢复。`require_scan:=true`防止扫描消失后仅凭轮式TF继续认为定位有效；它停止更新公共定位消息，再由既有定位超时和停止流程收尾；总停车延迟需覆盖上述完整链路测量。断流可能先触发Nav2的TF_ERROR（102），或先触发公共层NAV_POSE_STALE；验证器只接纳这两种定位失败，另检查断流后的位移与物理停稳。保存地图显式等待最多10秒，算法退出先执行生命周期关闭。

如需单独运行算法，使用`ros2 launch bindu_runtime localization.launch.py namespace:=/bindu_navigation navigation_robot:=绝对配置路径 mode:=mapping`；地图定位改为`mode:=localization map:=已保存地图的绝对YAML路径`并提供初始位姿。建图可用`pose_graph:=已保存图的绝对前缀`恢复；当前配置假定机器人位于原始建图起点，尚未验证任意位置恢复或绑架后全局重定位。不要与完整验证器同时启动第二套算法。

2026-09-22分阶段验证：五个建图航点、位姿图恢复/栅格地图保存、AMCL重载、正常往返及前进断流停车通过。建图定位RMS约31.8mm，AMCL正常往返定位RMS约36.7mm（与独立物理真值同原点、50ms内最近采样比较，未作刚体拟合）；桌边/返回实际到站误差100.6/51.4mm。断流注入时线速度0.062m/s，额外移动75.8mm，约0.622秒后满足150ms窗口的停稳判据。**断流恢复后的近距离回站失败，返回NAV_ACTION_FAILED:105并停稳；完整验证不是全绿。** 123项模块和6项旧导航回归通过。结果、失败批次及数据指纹索引为该批`verification.json`，地图和曲线为`restored-verified-tests/room.yaml`、`mapping_localization.png`。

本机依赖在`artifacts/slam-physics-2026-09-22/deps`隔离解包，环境入口为该批`env.sh`；未修改系统安装，不适用于新机器直接复制。地图、压缩原始扫描/位姿、轨迹、成功与失败批次保存在同目录。该传感器使用固定高度水平二维射线；3D雷达/IMU接入、真实传感器标定及完整碰撞检查需分别验证；精细抓取还需单独验证对位精度。旧低位腿关节停稳回归失败仍待排查。本机曾出现DDS生命周期服务响应超时，导致自动启动失败且需重启算法；失败日志保留，未宣称启动稳定性已解决。验证器等待AMCL处理首帧扫描后才采集收敛结果。



### 历史三维导航迁移入口

2026-09-22按用户要求，以本地`导航-源码.zip`中的FAST-LIO和FAST_LIO_LOCALIZATION为后续主路线。原二维SLAM Toolbox/AMCL保留为已验证基线。算法进程位于[`bindu_lio`](.)，与站点业务、Nav2和共享执行层隔离；原始ZIP不改。保留迭代滤波、IMU去畸变、局部地图及ikd-Tree和两级点到点ICP，来源指纹／适配范围见[`source_manifest.json`](source_manifest.json)，该包按随附GPL-2.0许可维护，原文件声明保留。公共执行模块不导入此算法包。

- `fast_lio`：带时间点云＋IMU → `navigation/odom`及`odom→base_link`、`lio/cloud_registered`；地图服务`lio/save_map`保存PCD，地图坐标为建图实例的odom系，重载时作为配置地图坐标，站点和二维栅格必须一起核对。地图路径预配置，已有文件拒绝覆盖；地图体素数有上限，达到上限锁定失败。
- `lio_localization`：重载PCD，接收`initialpose`后用粗／细两级ICP估计`map→odom`。初值按同一观测的`T_map_base × inverse(T_odom_base)`换算；匹配要求点数、fitness、RMSE和修正幅度同时达标。单工作线程／单待处理扫描，旧初值对应的异步结果丢弃；失败或过期不继续刷新TF。不是地点识别、任意位置重定位或回环优化。
- `lio_scan`：按同时间的LIO位姿和显式外参，把去畸变点云投影为`navigation/scan`。切片内没有回波的方向标为NaN，表示通行状态未知；高度和外参必须按实际传感器设置。可选发布`base_link→lidar`静态TF，已有机器人描述发布该边时关闭。
- `livox_input`：可选订阅已运行的`livox_ros_driver2/CustomMsg`，保留header/timebase与逐点offset_time，按原参考的line/tag规则过滤。需单独安装真正的Livox驱动消息包；本入口不启动设备驱动。该分支尚未与真实Livox驱动联调。

点云输入要求小端FLOAT32的`x/y/z/intensity/time`，`time`为相对header采集起点的秒数，**逐点时间使用传感器采样数据，接收时间和缺省零偏移不符合输入要求**。IMU为m/s²（含重力）和rad/s，必须和点云同一ROS时钟；启动时保持静止初始化。外参采用xyz＋xyzw，`imu_from_lidar`将雷达坐标转换到IMU坐标，`imu_from_base`将底盘坐标转换到IMU坐标。配置显式指定所有frame，不静默套用历史MID360/Tracer外参。输入过期、IMU间断、扫描重叠、队列超限或跟踪失效锁定后需重启估计器；节点不发布运动指令，导航侧仍由原有定位时效／停止契约接管。

ROS 2 Jazzy工作区增加PCL/Eigen依赖后构建；ICP的Open3D等可选依赖需独立安装，不混入VR/IK构建环境。最小Ubuntu22.04容器还需`libgl1`／`libglib2.0-0`，缺少`libGL.so.1`时即使仅做数值配准也无法导入Open3D：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select bindu_lio bindu_runtime --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
# 仅给定位进程使用的可选Python依赖
python3 -m pip install --target artifacts/lio-python -r tools/requirements/requirements-navigation.txt
```

启动示例中的传感器配置是**合成测试fixture**，零外参不是实机标定，也不是现有Isaac二维射线输入的适配：

```bash
ros2 launch bindu_runtime lio.launch.py recording_mode:=normal namespace:=/bindu_lio_test \
  sensor_config:="$PWD/tests/config/navigation/navigation_lio_fixture.json" \
  mode:=mapping save_map:="$PWD/artifacts/lio-room.pcd"
# 有输入并完成建图后，另一个终端保存；不会覆盖已有PCD
ros2 service call /bindu_lio_test/lio/save_map std_srvs/srv/Trigger '{}'
```

重载定位时先收尾建图实例，给**定位进程所在终端**增加`PYTHONPATH="$PWD/artifacts/lio-python:$PYTHONPATH"`，再用相同入口的`mode:=localization pcd_map:=/绝对路径/map.pcd`并发送`initialpose`。已有同坐标的二维栅格可用`grid_map:=/绝对路径/map.yaml`启动Nav2 map_server；不可把旧地图随意换名使用。随后原`navigation_physics.launch.py`可消费相同命名空间的`navigation/odom`、`navigation/scan`和TF；算法速度仍必须经过目标身份、租约和执行层。此模式独占map→odom和odom→base，不能并行启动AMCL/SLAM或模拟器旧odom发布者。Isaac三维物理复测入口见下文；各批通过与失败边界单独记录。

验证入口：`tests/integration/validate_lio.py --binary install/bindu_lio/lib/bindu_lio/fast_lio --output artifacts/lio-check`使用时间展开的合成三维房间扫描与IMU，检查动态估计、采集时间、错误字段拒绝、断流及PCD保存；`tests/integration/validate_lio_localization.py --map artifacts/lio-check/map.pcd --output artifacts/lio-localization-check`验证真实ROS点云重载定位、错误匹配和重设初值；`test_lio_localization.py`、`test_lio_ros_adapters.py`覆盖配准和扫描外参。以上不连接真实机器人。



### 三维算法的 Isaac 同场景复测

复用上面的房间、站立收臂模型、`navigation_mapping_sites.json`和Nav2装配，Isaac启动参数改为`--navigation-sensors src/integration/bindu_runtime/config/navigation/navigation_lio_isaac.json`。该配置独立描述8个仰角、360个方位、每12个物理步完成一帧的三维射线扫描和IMU；逐束采样使用当时的PhysX姿态，`time`保存实际采集偏移。5mm距离噪声沿用二维基线，IMU另加0.02m/s²和0.002rad/s的高斯噪声，均为未标定的测试假设。

`tools/simulation/isaac_lio_sensors.py`只发布原始点云和IMU，不发布机器人位置、地图或TF。IMU通过物理姿态的差分获得，使用与执行器一致的墙上采集时间，并转换重力/角速度到传感器坐标；这不是特定雷达型号的完整误差模型。建图时FAST-LIO按测量时间发布map→odom原点定义；定位时该边由ICP唯一发布，模式切换时需清理建图阶段的静态恒等变换。

新工作区先重建`bindu_lio`和`bindu_runtime`。Open3D运行依赖只在运行时加入`PYTHONPATH`，不要让其附带的setuptools进入colcon构建环境。保持与Isaac相同的ROS域，Nav2按前面的`require_scan:=true`入口启动；第三个终端运行：

```bash
python3 -m tests.physics.validate_localization_physics --backend lio \
  --sensor-config src/integration/bindu_runtime/config/navigation/navigation_lio_isaac.json \
  --robot tests/config/navigation/navigation_g1_fixture.json \
  --sites src/integration/bindu_runtime/config/navigation/navigation_mapping_sites.json \
  --output artifacts/lio-physics-tests
```

验证器复用二维基线的五个建图航点、初值偏置、往返目标及位置/航向/停稳判据。保存原生PCD，并从带同时间LIO位姿的实际注册点云生成5cm栅格：观测射线标自由空间，高度0.1–0.95m的回波标占据，0.2m点云体素按0.1m半宽保守覆盖；没有读取场景障碍物坐标生成地图。这个切片与体素覆盖是本次fixture的配置，替换传感器外参、体素或场景高度时须同步调整。物理真值只用于传感器合成、设备反馈及独立误差评估。

FAST-LIO断流锁定后要求显式重启，故新测试在确认停车后重启LIO/ICP并重新提供近似初值，再提交回站目标；不会通过扫描恢复自动续跑旧目标。单独复测定位可追加`--phase localization --pcd-map /绝对路径/room.pcd --map /同坐标地图/room.yaml`。原始点云以字段信息和base64数据保存于压缩观测日志；失败批次不覆盖。精度对比需同时记录二维/三维输入、轮式误差与IMU噪声差异，以区分输入条件和算法的影响。

2026-09-22同场景实测分批完成：FAST-LIO五航点建图到点误差16.7–33.1mm，建图定位RMS15.9mm；PCD与5cm栅格保存、ICP重载初值收敛通过。桌边单程定位RMS25.5mm，到站35.1mm；行进中断雷达触发`NAV_POSE_STALE`，额外移动44.1mm、约0.634秒后满足150ms停稳窗口，显式重启并用最后算法位姿初始化后回站57.5mm。10项LIO模块、2项IMU采样、6项旧导航和3项Fast DDS原生身份检查通过。原始数据、地图、曲线与失败批次保留在`artifacts/lio-physics-2026-09-22/verification.json`索引中。定位单程与建图属于不同阶段，不应混用误差统计。

**9月22日批次完整正常往返未通过**：返程准备及后续重测出现约107–320ms的仿真反馈间断，FAST-LIO按50ms IMU间断保护锁定；新增耗时诊断把主要停顿定位到反馈读取／发布阶段，尚未确认更底层原因。启动前检查连续5秒反馈稳定，运行期间仍持续监测故障。隔离CycloneDDS对照可定位，但原生发布者身份校验拒绝合法输入，未绕过校验或切换项目默认中间件。`--navigation-cases fault`可独立复现断流/恢复，`roundtrip`可独立复测正常往返；默认`all`保持完整顺序。上述检查分批完成；完整端到端运行、真实雷达精度和相对二维基线的收益仍待验证。

2026-09-23复测补齐一轮完整链路：`sync-full-tests`的13项检查通过，包含重新建图、地图重载、正常往返、断雷达停车及显式恢复回站。进展检查从只看平移的`SimpleProgressChecker`改为`PoseProgressChecker`，0.1m平移或0.2rad航向变化均计为运动进展，停滞期限仍15秒；到点、停车、输入时效和IMU间断判据保持不变。实际Nav2插件的时钟控制测试确认原地转向通过、完全静止／不足阈值的运动超过15秒仍失败。17项导航模块检查通过。

同阶段对照旧二维版本的保留数据（未作轨迹刚体对齐）：

| 指标 | SLAM Toolbox / AMCL基线 | FAST-LIO / ICP本次完整批次 |
|---|---:|---:|
| 建图定位RMS | 31.8mm | 19.0mm |
| 正常往返定位RMS / P95 | 36.7 / 54.2mm | 26.4 / 52.0mm |
| 正常往返定位最大误差 | 56.8mm | 99.4mm |
| 桌边 / 回站位置误差 | 100.6 / 51.4mm | 39.2 / 38.9mm |
| 去程实际路长 / 时间 | 4.18m / 27.6s | 4.59m / 38.7s |

新版本用带逐点时间的三维点云＋IMU替代有偏轮式里程计＋二维扫描，原生地图为PCD，定位为需要近似初值的两级ICP；旧版保留二维位姿图和AMCL粒子定位。全局NavFn、局部RPP、站点及公共执行契约沿用。新栅格包含高度切片内的桌面等三维回波，并按体素宽度保守覆盖；障碍更厚、规划路线更远，地图评价还需结合障碍边界、可通行区域和路线长度。本批平均误差较小，但峰值更高、耗时更长；两批实验使用不同传感器、地图投影、进展检查和发布模式，精度及稳定性结论限于各自记录。

本轮原始点云/IMU/定位/物理反馈、PCD/栅格、各目标规划路径和速度、失败日志、统计与图保存在`artifacts/lio-stability-2026-09-23/`，索引为`verification.json`，对照数据为`comparison.json`，图片为`map_comparison.png`及`navigation_comparison.png`。物理真值仍只作传感器合成、设备反馈和独立评分。完整批次断雷达后额外45.2mm，约0.542秒满足150ms停稳窗口；显式重启后回站71.2mm。

发布稳定性仍未解决：细分计时与临时原生探针确认约313ms停顿发生在`rmw_publish`内部，PhysX读取约0.2ms；阻塞栈进一步落在Fast DDS旧历史消息移除路径（`DataWriterHistory` → `StatelessWriter::change_removed_by_history` → `pthread_mutex_lock`），具体持锁线程仍待确认。禁用共享内存的UDP对照、切换同步发布均仍复现。完整成功批次使用Isaac同步发布，但启动／恢复阶段仍记录约325–330ms停顿，因此未把同步发布固化为默认修复。诊断还保留生命周期`change_state`响应超时、一次返程任务成功但独立物理速度未达停稳阈值等失败。探针只用于诊断，不用于性能验收。

进一步使用仅回环接口、非阻塞发送的UDP配置（本批`loopback-nonblocking.xml`，所有ROS参与者统一设置`FASTRTPS_DEFAULT_PROFILES_FILE`），无探针对照的6项定位／往返／断流恢复检查及3项原生身份检查通过；到站42.6／34.7mm，断雷达额外16.8mm，恢复回站53.9mm。约170.6秒观测中反馈与IMU各17,876条，源时间戳最大间隔22.0ms，无超过50ms间断。40秒端点反复创建／退出检查也未出现超过50ms反馈或IMU源间断。该配置同时改变发送模式和接口选择，仅是本机仿真候选缓解措施；不固化默认、不适用于跨机链路，仍需重复／长时间负载验证，也未证明具体持锁线程的原因。
