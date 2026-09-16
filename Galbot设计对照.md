# Galbot 参考设计与 Bindu 对照

日期：2026-09-15。来源对照形成于 Bindu v0.2；当前取舍见架构 v0.3 与 D012。依据为本项目提供的文档，不代表已审查 Galbot 完整源码；以下 Bindu 方案均为建议。

## 1. 先区分来源性质

| 来源 | 性质 | 使用方式 |
|---|---|---|
| [软硬件系统架构](../参考资料/Galbot/软硬件系统架构.md) | 对具体机器配置、日志、接口的分析，部分内部行为明确为推测 | 复用已描述的分工与时序；不把推测算法、型号参数和控制频率作为本项目事实 |
| [YDH Catering 架构](../参考资料/Galbot/ydh-catering-architecture.md) | 对业务仓库职责、技能、调用路径与算法的说明 | 复用任务/资源生命周期与操作合同；不照搬餐饮业务、固定维度或具体算法库 |
| [日志与系统服务](../参考资料/Galbot/日志与系统服务.md) | 对现场管理服务、配置、进程及日志的只读分析 | 参考就绪、配置、诊断及排障边界，不复制机器特定启动脚本 |
| [control 架构](../参考资料/Galbot/control架构.md) | 文中明确以“我会采用”表达的参考设计建议 | 作为架构原则，而非全部已在 Galbot 落地的事实 |

## 2. 逐项取舍

| Galbot 文档依据 | Bindu 处理 | 类型 |
|---|---|---|
| 软硬件 §11：RobotVLA 管理运行时，PiInferenceAdapter 管协议，Executor 管动作，Transport 管通信 | SkillAttempt 管一次执行；PolicyAdapter/TransportClient 解析语义和通信；ExecutionSession 管运动时间轴 | 沿用职责，调整命名与边界 |
| 软硬件 §11：worker 观测/推理，executor 按时间发布，下层继续上一段动作 | 推理与执行异步；推理线程只提交候选，执行端提交到唯一时间轴 | 沿用并明确最终提交权 |
| 软硬件 §13/§14：未来动作矩阵、时间轴、批次 index 与 stop/状态 RPC | JointReferenceSegment/JointTargetUpdate 带时间、序号、资源与代次；停止是有反馈的状态过程 | ROS 2 适配，不复制二进制格式和维度 |
| 软硬件 §11：Helper 集中观测、图像、TF/FK/IK | Driver、StateHistory、ObservationAssembler、KinematicsProvider 分开 | 调整，避免形成跨层大对象 |
| YDH §5：任务值对象、Context、Session、Attempt、Runtime 分开 | TaskRequest 不可变；TaskContext 记事实；SkillAttempt 唯一持有本轮请求/轨迹/资源；SessionView 只读 | 沿用 |
| YDH §6：行为树→SkillAction→Registry→Adapter→Skill/Attempt | 任务树只组织技能；Registry 调度和取消；技能 Owner 负责物理收敛；Adapter 管接口转换 | 沿用 |
| YDH §11：感知 TaskSession 管请求归属、时间、超时、取消和释放 | PerceptionSession 绑定 observation、request、model instance、attempt；迟到结果隔离 | 沿用 |
| YDH §12：Query Runtime 与 Plan Runtime 分开；连续约束路径 IK | FK/模型查询是库接口；规划异步有 deadline；局部连续 IK/Servo 单独更新，不每周期全局规划 | 沿用角色，算法按已有 MoveIt 核验 |
| YDH §15：worker/instance/request 身份、模型 metadata、RTC 前缀 | 推理握手和身份检查；RTC/固定前缀仅在服务端与客户端契约匹配后启用 | 沿用，可选能力明确化 |
| YDH §16：配置快照、代次/hash，同一尝试不混读新旧配置 | ConfigSnapshot 固定一次任务依赖；模型/标定/拓扑变更要求停止后的显式重装配 | 沿用并简化首期热更新 |
| YDH §17：finish、cancel、进程退出；取消时持物优先保持 | drain、cancel、shutdown 分开；已持物取消不自动张手，受限 recovery 另有许可 | 沿用并细化执行入口 |
| YDH §17：Operation/Safety/Evidence/Report/Acceptance 各有语义 | Operation、PhysicalState、Evidence、DeliveryStatus 分别返回；任务按合同决定接受/继续 | 沿用结果解耦，不宣称软件状态证明物理安全 |
| 日志文档：模式更新≠进程就绪≠业务可执行 | 模式监督按本次技能依赖计算 readiness；启动成功不自动发放运动权限 | 沿用 |
| 软硬件 §17：未见统一 trace ID 贯穿全部层 | run/task/attempt/observation/request/command 身份和转换记录贯通 | 新增，不能声称原系统已具备 |
| 用户明确：Orin、ROS 2、首期轮式后续双足、100 Hz 目标 | 固定上层逻辑采样合同；后端能力声明；ROS 2 包装；底层频率依据真实接口 | 用户需求驱动适配 |

## 3. 保留三个闭环

1. **任务闭环**：任务输入 → 技能执行 → 实测验收 → 更新任务事实/恢复。不能由推理线程改任务成功状态。
2. **策略闭环**：同一份有效观测 → 推理/规划 → 候选动作 → 提交结果/执行反馈 → 下一轮观测。提议、已接受与已执行分别记账。
3. **执行闭环**：有效参考 → 周期下发 → 真实反馈/驱动状态 → 轨迹推进/收敛。底层反馈缺失时，不能用命令回显伪装这个闭环已建立。

## 4. 不直接继承的内容

- HPU/XCU/MCU 的具体计算机划分、Embosa/Fast DDS 的私有 Topic/RPC、20/23/26 维向量、参考配置频率。
- 将 ROI、相机别名补齐或非活动关节填充规则视作所有模型的默认语义；必须按各模型契约校验。
- 凭 deadline 到期把“饮料抓取”算作成功；流程允许降级与实际抓住必须分开。
- 用控制日志的 ERROR 数量推断根因，或把日志/上报失败变成重新执行物理动作。
- 复制多层进程管理器。单台 Orin 首期用一个装配/监督入口和已有 ROS 工具，运动权限仍在本地执行器强制检查。

## 5. 来源冲突与未知项的处理

当参考文档的不同示例对未控制关节、动作频率或停止语义采用不同策略，记录为不同实例/配置，不合成“Galbot 统一行为”。Bindu 选择自己的明确合同，并记录依据。已有控制器的插值、反馈、CSP 周期仍待真实接口核验；当前仅完成文档设计。
