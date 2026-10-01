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

本机源码构建及六个 ROS 包构建通过。39 项纯策略与真实 ROS 消息回归测试通过；接口测试使用独立 DDS 域和日志目录。八种 PX4 消息静态匹配。实例冲突返回非零，空中普通 DISARM 被拒绝。审查发现的问题已修复并复查。

最终正式三轮全部通过，悬停最大误差 0.100m、0.106m、0.107m，每轮 landed + disarmed 确认。显式解锁后延迟起飞通过。适配器退出后独立观察到 armed AUTO_LAND，7.49 秒内确认 landed + disarmed。该 PX4 解除解锁后会恢复旧 nav_state，因此最终 offboard 标签不作为失控降落失败依据。

Gazebo GUI 与 RViz 启动通过，RViz 报告 OpenGL 4.6。没有做界面截图或相机/雷达渲染验收。完整细节和机器可读证据见 [V0.1 本机验收](validation/V0.1-local.md)。

## 限制与发布条件

- 本机尚未安装 Docker；容器/干净安装由 [GitHub CI](https://github.com/daydayup0713-wq/uav_simulation/actions) 验证，当前构建中。策略 CI 已通过。容器兼容无主仓库 .git 的源码归档，由构建参数提供版本标识；系统依赖、固定源码依赖和平台构建分层缓存。
- GUI/GPU 传感器验收与本机无头飞行验收分别记录。nvidia-smi 在正常本机环境枚举 RTX 3050 Laptop GPU / 4096MiB；此前无法枚举是沙箱限制。未宣称 CUDA、相机、雷达渲染通过。
- ACK 拒绝、超时、模式未切换、遥测过期、恢复不续飞通过策略注入验证；独立真实故障实验为适配节点退出。
- 尚未打 V0.1.0 标签，保留为候选版，待干净安装门禁通过。实现位于 [PR #1](https://github.com/daydayup0713-wq/uav_simulation/pull/1)。
- 全局 ROS、Agent、旧学习包、Edge/Cloud 工作区和原 PX4 未修改；本次只在新仓库构建副本。
