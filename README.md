# Bindu 项目文档入口

更新：2026-09-16。`bindu/` 是主工作区；本页集中维护文档入口。早期 Galbot 文档位于[参考资料](../参考资料/Galbot/)，按需对照。先读上下文和进度，再按工作范围读取主文档。

## 每类信息只在一个地方维护

| 内容 | 主文档 |
|---|---|
| 用户已确认的需求、设备、未知项 | [项目上下文](项目上下文.md) |
| 已完成证据、当前阻塞与下一步 | [当前进度](当前进度.md) |
| 历史源码、归档资产、已具备功能与复用边界 | [历史代码盘点与复用评估](历史代码盘点与复用评估.md) |
| 分类去重后的源码精选与使用说明 | [代码参考包](代码参考包/README.md) · [下载 ZIP](代码参考包/Bindu-历史代码精选-2026-09-16.zip) |
| 模块职责、状态所有权、首期装配 | [软件架构 v0.3](软件架构设计.md) |
| 执行字段、时间轴、结束/取消/保持 | [执行契约 v0.2](动作执行契约.md) |
| 候选选型、官方依据、P0–P6 与验收 | [一期建设计划](一期建设计划与开源选型.md) |
| 取舍与被替代建议 | [决策记录](决策记录.md) |

其他文档只链接主定义。修改规则先改主文档，再检查图示；历史记录追加，不拿旧建议覆盖当前版本。具体协作与文件增长规则见 [AGENTS](../AGENTS.md)：优先原地更新，默认每项任务不新增 Markdown。

## 图示与来源

- [总体与模块关系图](系统架构图.md)：六层总体架构＋跨层公共平台。
- [各层架构图](各层架构图.md)：七类职责组的展开视图；并非另一套七层部署。
- [科研汇报 PNG](图片/总体架构图-科研版-v1.png) · [生成记录](图片/总体架构图-科研版-v1-提示词.md)。静态图片不作为接口规范。
- [Galbot 设计对照](Galbot设计对照.md)：早期参考资料的性质及取舍；参考系统与当前实验室机器分开。其他历史笔记：[RoboTwin / PI0.5](../参考资料/Galbot/robotwin_pi05.md)。

## 历史与审查

- [2026-09-16 架构与文档审查](过程记录/2026-09-16-架构与文档审查.md)：问题、处理及验证边界。
- [2026-09-16](过程记录/2026-09-16.md) · [2026-09-15](过程记录/2026-09-15.md)：按日过程记录。

当前实现状态见[进度](当前进度.md)。模拟开发机测试不等于机器人部署或实测；候选模块不能仅凭图示、README 或历史文件名确定控制语义。

## 工程入口

总工程名为 **bindu**，本目录同时是 ROS 2 工作区根目录。本地修改源码，目标开发机 `/home/wxb/bindu` 编译验证。

```text
bindu/
├── src/
│   ├── bindu_interfaces/      # ROS 2 msg / srv / action
│   ├── bindu_core/            # 无 ROS 依赖的任务、策略、执行与后端核心
│   └── bindu_runtime/         # ROS 2 节点、launch/、config/
├── tests/                    # 跨进程集成与故障测试
├── artifacts/                # 构建日志、验证结果与模拟 episode，忽略入库
├── build/ install/ log/      # 目标机 colcon 生成，忽略入库
├── 历史代码/ 代码参考包/      # 参考资产，独立于生产源码
└── README.md 等既有主文档     # 上下文与记录入口
```

模块核心位置、替换接口与当前限制见[架构8.5.4](软件架构设计.md#854-当前代码组织与替换边界)。包内测试放在 `src/<package>/test/`；跨包测试只放 `tests/`。不为每个模块再建说明文档。

在目标开发机执行（仅模拟）：

```bash
cd ~/bindu
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --symlink-install
source install/setup.bash
python3 -m unittest discover -s src/bindu_core/test -v
python3 tests/validate_ros.py --output artifacts/manual-verification

# 交互启动，保持在单机测试域
export ROS_DOMAIN_ID=116 ROS_LOCALHOST_ONLY=1
ros2 launch bindu_runtime skeleton.launch.py
```

另一个已 source 环境且使用相同 ROS 域的终端提交模拟任务：

```bash
ros2 action send_goal /bindu_sim/tasks/fetch_drink bindu_interfaces/action/FetchDrink   "{task_id: demo_1, object_id: drink, strategy: planner}" --feedback
```

`strategy` 可选 `planner` / `chunk`；成功后当前模拟场景保持持物事实，新的独立实验重新启动并使用新 run_id。不要把该状态当成真实灵巧手持续保持会话。构建时显式使用 `--base-paths src`，避免扫描历史目录中的旧 ROS 工程。复现测试会自建隔离命名空间、启动并关闭其自身模拟进程，不调用机器人 SDK。
