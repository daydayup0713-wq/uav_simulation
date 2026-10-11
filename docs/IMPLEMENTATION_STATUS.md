# Implementation ledger — 2026-10-01 V0.1

User approved the complete in-chat plan and requested execution. Execute inline in the newly cloned empty repository on feat/v0.1-foundation. No additional worktree is needed: this checkout is already separate from all existing projects.

Ruling: deliver V0.1 now, preserve V0.2–V3.0 as staged roadmap; later algorithms require their own dataset and hardware acceptance.
Ruling: freeze PX4 timestamp synchronization off in the simulation profile, and map raw firmware timestamps to simulation time at the adapter boundary. Wall monotonic time remains authoritative for freshness/timeouts.

Tasks: dependency/doctor, simulation/interfaces, controller, CLI/demo, fault/CI/delivery.
Pre-flight: interface package must build before bridge/tools; dependency checkout must complete before simulation build; bridge API is shared by CLI/demo. No legacy overlay is permitted.

## 已实现

- 五个平台 ROS 包和固定提交 px4_msgs，共六个构建包。
- 独立 PX4、Agent 构建副本；Agent 2.4.3 和四个主要源码依赖固定提交，实际 checkout 与私有 CMake 解析路径均核验，P2P 功能关闭。
- 按就绪条件启动的监督进程、每次独立 Gazebo 分区、实例锁、端口检查、配置快照、本次进程组清理。
- ENU/FLU 适配、时间转换、20Hz 心跳、ACK+状态确认、Trigger 服务和 ExecuteFlight Action。
- CLI、三轮演示、RViz 配置、独立适配器退出观察器、Ubuntu 安装脚本和无头 CI 容器。

## 本机验收通过

本机源码构建及六个 ROS 包构建通过。50 项纯策略与真实 ROS 消息回归测试通过；接口测试使用独立 DDS 域和日志目录，并避开活动实例记录的域。八种 PX4 消息静态匹配。实例冲突返回非零，空中普通 DISARM 被拒绝。CLI 中断 GOTO 后进入悬停；中断 LAND 保持降落，未接受的请求不误报继续降落。审查发现的问题已修复并复查。

最终提交 `5cb3c29` 的三轮全部通过，悬停最大误差 0.116m、0.111m、0.107m，每轮 landed + disarmed 确认。显式解锁后延迟起飞通过。适配器退出后独立观察到 armed AUTO_LAND，7.62 秒内确认 landed + disarmed。该 PX4 解除解锁后会恢复旧 nav_state，因此最终 offboard 标签不作为失控降落失败依据。

Gazebo GUI 与 RViz 启动通过，RViz 报告 OpenGL 4.6。没有做界面截图或相机/雷达渲染验收。完整细节和机器可读证据见 [V0.1 本机验收](validation/V0.1-local.md)。

## 限制与发布条件

- 本机尚未安装 Docker；[干净环境 CI #6](https://github.com/daydayup0713-wq/uav_simulation/actions/runs/36913384176)已全部通过：固定源码依赖构建、50 项回归、连续三轮飞行、失控降落。悬停最大误差 0.116m、0.099m、0.093m，失控降落 7.58 秒。初次暴露的飞前高度初始化和重新解锁顺序问题已修复并重新验收。容器兼容无主仓库 .git 的源码归档，由构建参数提供版本标识；系统依赖、固定源码依赖和平台构建分层缓存。
- GUI/GPU 传感器验收与本机无头飞行验收分别记录。nvidia-smi 在正常本机环境枚举 RTX 3050 Laptop GPU / 4096MiB；此前无法枚举是沙箱限制。未宣称 CUDA、相机、雷达渲染通过。
- ACK 拒绝、超时、模式未切换、遥测过期、恢复不续飞通过策略注入验证；独立真实故障实验为适配节点退出。
- V0.1.0 发布门禁通过；标签 `v0.1.0` 保存本次完整交付，代码验收提交为 `5cb3c29`，其后仅补充文档与证据。实现和审查入口为 [PR #1](https://github.com/daydayup0713-wq/uav_simulation/pull/1)，保留开发分支供后续审查。
- 全局 ROS、Agent、旧学习包、Edge/Cloud 工作区和原 PX4 未修改；本次只在新仓库构建副本。

## V0.2.0 交付（2026-10-04）

`feat/v0.2-sensors` 在V0.1基线b823eba上增量建设，功能提交02bd804；六包版本0.2.0。增加通用室内场景、0.05kg学习载荷、360×16三维雷达、200Hz IMU、320×240/15Hz相机、安装TF、独立真值；配置统一生成模型和数据契约。CLI增加实时传感器检查、完整录制、离线复核和独立ROS域回放，记录配置及bag校验值。

本机真实NVIDIA Ogre2渲染、传感器三轮飞行、CPU三轮回归、Gazebo/RViz同时运行的一轮飞行、断桥降落、80秒运动数据录制/回放和空间几何检查均通过。一次独立全分支审查的五项Important已按失败复现→修复→全量回归处理；最终80项测试通过，无延后项。遥测处理串行化修复了多线程回调调度导致的伪源时间倒退，真实时间倒退仍拒绝。

[最终干净CI](https://github.com/daydayup0713-wq/uav_simulation/actions/runs/37199496965)通过固定依赖构建、六包构建、80项回归、三轮飞行与失控降落。完整验收、来源及限制见 [V0.2报告](validation/V0.2-local.md)、[审查修复](validation/V0.2-review.md)。交付入口 [PR #2](https://github.com/daydayup0713-wq/uav_simulation/pull/2) 基于仍待审查的V0.1分支；标签v0.2.0保留本次交付。

执行取舍：实机物理/传感器逼真度仍需未来实测标定，否则不能据此推算实机性能；Mid-360逐点时间、SLAM、无GNSS飞行和避障按后续阶段交付，任意航点尚可能碰撞；本版发布使用实际干净CI作为门禁，不能凭旧版CI或本机结果推断可复现安装。

## V0.3.0 / V0.3.1 交付（2026-10-07 收尾）

在V0.2标签245e8fa上建设feat/v0.3-slam，依赖提交ca55a6a、固定输入基准17f29c5、实时/地图/无GNSS25fbeb0，审查修复87f7321。平台六包版本0.3.1，与固定px4_msgs一起构建7包。私有GLIM v1.1 CPU替代对本机GTSAM4.2/Boost1.74不兼容的较新版本，源码固定且未修改。

本机通过固定输入ICP学习/LIO比较、独立真值六自由度误差评估、实时LIO、3个实际原生非相邻回环约束、地图归档/加载/新扫描粗位姿重定位。审查修复后连续无GNSS三轮通过，悬停最大误差0.106/0.109/0.106m；位置ATE RMSE0.058m、姿态RMSE0.264°、覆盖100%。四个实际ULog确认GPS输入fix0和EV位置/高度/航向融合，原生LIO退出后独立观测7.90s失控降落/解除武装。

一次独立全分支审查的5项问题全部失败复现→修复→全量回归，126项通过，无延后问题。当前模式的startup preflight与Offboard解锁资格分开处理；未禁用电池等飞控检查。默认flight/sensors继续保留既有行为。[实验操作](experiments/03-localization.md)、[实测及来源](validation/V0.3-local.md)、[审查修复](validation/V0.3-review.md)、[完整取舍与代价](validation/V0.3-decisions.md)。增量入口[PR#3](https://github.com/daydayup0713-wq/uav_simulation/pull/3)，基于V0.2分支。

首次[干净CI](https://github.com/daydayup0713-wq/uav_simulation/actions/runs/37219484820)与最终功能修复提交87f7321的[CI](https://github.com/daydayup0713-wq/uav_simulation/actions/runs/37221251013)均已通过；后者为126项回归、标准飞行三轮、断桥降落及私有CPU回放/地图导出，数值和版本见[CI证据](validation/v03/clean-ci.json)。版本标签只在最终文档提交的门禁也通过后创建。通用同步雷达、局部粗位姿重定位、明确EKF噪声下限为当前范围；Mid-360、真实机标定、全局地点识别和三维避障另行验收。地图/录制/重建大文件保持忽略，Git只保存来源和数值证据。

## V0.4.0 交付（2026-10-07）

从v0.3.1建立feat/v0.4-navigation，8个ROS构建包，平台版本0.4.0。新增观测空间内三维占据网格、机体膨胀、26邻域A*、精确supercover碰撞、低速参考轨迹、Navigate/PlanPath、CLI及导航RViz场景。点云由源时间LIO注册至连续odom，真值与世界几何仅供独立评估。未知区域阻挡通行，HOLD/LAND世代、迟到ACK所有权和过期锁定保证不续飞。

一次独立审查的6项Important全部失败复现、一次修复完成，无延后项。ff2eddb最终187项回归通过；原生连续3轮绕障/返回/落地通过，3次障碍内目标拒绝，独立最小机体净距0.6682m。15条执行计划和3拒绝查询的不可覆盖地图/哈希逐条回放通过，30帧实际雷达离线基准通过。导航退出后8.0971秒确认PX4失控降落、落地解除武装，即时恢复数据后仍拒绝重新解锁。

完整[实验04](experiments/04-navigation.md)、[验收及来源](validation/V0.4-local.md)、[审查修复](validation/V0.4-review.md)、[执行取舍](validation/V0.4-decisions.md)。增量[PR #4](https://github.com/daydayup0713-wq/uav_simulation/pull/4)基于待审查的V0.3分支；最终提交CI门禁通过后创建v0.4.0，不自动合并旧PR。当前范围为静态已观测空间，动态障碍、自主探索、完整起降避障、实机标定另行验收。下一阶段V0.5为巡检点与任务执行、姿态/采集动作、暂停和恢复。

## L1 多算法实验台升级（2026-10-10）

`feat/l1-workbench` 从 V0.4 基线 `e295a54` 增量建设，保留旧 CLI/Gazebo/RViz 与原飞行门禁。七种定位方法和四条规划路线均锁定上游版本并实际运行；算法质量失败也保留报告。新增时间参数化 C2 轨迹、50Hz 执行、20Hz 心跳、观测空间内全曲线碰撞检查和受地图约束的 HOLD，以及六个复杂场景和三个独立传感器配置。

React/TypeScript/Three.js 本地实验台提供原始单帧、配准累积、实际全局优化地图、障碍体素四图层，轨迹、机体、视角/着色、文件回放、分组报告和监督器管理的控制入口。实际 Web 起飞、航点、HOLD、LAND 和停机流程通过；实际 47 控制点航线成功，底层仍只有飞控适配器发布 PX4 控制指令。

最终审查后的七种方法均完成回放：GLIM、FAST-LIVO2、RTK 前端通过，另外四种保留质量失败。`2454643` 的三轮正常闭环完成 141 控制点/138 次衔接（P95 0.040397m、最小机体净距 0.701791m），但定位退出故障的冷启动 DDS 接受回复丢失，整套故障未通过，不能继承旧控制资格。客户端无运动拒绝探测已通过五项 RED→GREEN 回归，新控制实现的正常/故障复验尚待完成。RTK EOF 优化通过，但地面实时看门狗失败；EGO、GCOPTER 和其余复杂场景的失败保留。

审查修复代码 `2454643` 的本机 ROS/Python 501 通过、1 条件跳过；前端 13 通过及生产构建通过。[干净 CI](https://github.com/daydayup0713-wq/uav_simulation/actions/runs/38063440951) 已完整通过源码/容器构建、三次飞行、断桥降落和原生算法 CPU 门禁。完整结果见 [L1 验收](validation/l1/acceptance.md)、[当前组合证明摘要](validation/l1/current-pair.json)、[Web 操作证据](validation/l1/web-controls.json) 与 [全部执行取舍](validation/l1/decisions.md)。增量审查入口为 [PR #5](https://github.com/daydayup0713-wq/uav_simulation/pull/5)，基于待审查的 V0.4 分支，未自动合并旧 PR。

本机默认 NVIDIA EGL 路径因驱动/用户库不匹配失败，显式 Mesa 显示渲染通过；这需要 DISPLAY。主机偶发时钟/接收停顿的原因仍未证明，门禁继续拒绝有问题的运行。本轮提供真机参数模板与部署检查，15 个实际测量项尚缺，未授予真实机体试飞资格。实际算法阶段、模拟近似与资源范围分别记录，当前结论不能外推为真机精度或整机性能。
