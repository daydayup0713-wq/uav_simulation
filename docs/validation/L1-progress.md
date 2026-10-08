# L1 升级验收进度

设计与任务：[批准设计](../superpowers/specs/2026-10-08-l1-workbench-design.md)、[实施计划](../superpowers/plans/2026-10-08-l1-workbench.md)。基线 `v0.4.0/e295a54`，升级分支 `feat/l1-workbench`。

## 已有证据（2026-10-08）

- 基线 187 项原生测试通过（21.54s）。后端契约完成后 201 项通过；连续轨迹核心完成后 215 项通过；增加轨迹 Action、导航故障和独立验收后 **233 项**完整回归通过（21.58s）。
- 9 个 ROS 包可构建。注册表声明七个定位与四个规划后端，尚未把这些声明视为安装或运行资格。`labctl experiments list` 的阶段依赖固定源码版本和不可覆盖的验收产物哈希。
- `8af9797` 的 GLIM＋连续 A*＋PX4 无 GNSS 仿真，运行 `20261008T042008Z-3964e4dd`，自动往返、障碍内部目标拒绝、降落及落地上锁确认通过。
- 该次独立观测：158 个实际跟踪样本，P95 **0.034857m**，最大 **0.040636m**；1677 个轨迹参考，参考 **50.061Hz**、PX4 setpoint **50.002Hz**、Offboard 心跳 **20.001Hz**。参考速度最大 0.495049m/s，加速度 0.119420m/s²，差分 jerk 0.101833m/s³。
- 独立 Gazebo 真值几何评估：1358 个样本，最小机体净距 **0.752836m**；机体半尺寸 `[0.4,0.4,0.3]`。最大实测速度 0.726989m/s 包含旧起飞阶段，参考限制和实际响应分别记录。
- 原始证据位于 `.runtime/continuous-nav-acceptance/`，地图、状态和 PX4 日志在对应运行目录。初始开发期裸轨迹运行 `20261008T021307Z-ee9df044` 也通过，P95 0.082154m，但源码为 dirty，仅作为调试记录。
- 原有三次自动飞行演示全部通过；飞控适配退出后失控降落通过（7.584582s，落地/上锁确认）。原始完整日志 `/tmp/uav-l1-legacy-flight-gate.log`，运行 `20261008T042638Z-b388d930`。

## 当前操作入口

```bash
./scripts/labctl experiments list
./scripts/labctl experiments capabilities glim
./scripts/start_lab.sh --profile navigation --continuous --headless
./scripts/labctl nav demo --runs 1
```

新的连续导航显式使用 `--continuous`，旧 V0.4 模式保留。自动启动、演示、观测和清理：

```bash
./scripts/env.sh /usr/bin/python3 scripts/accept_continuous.py \
  --profile navigation --runs 1 --output .runtime/my-continuous-acceptance
```

输出目录必须尚不存在，以免覆盖证据。地面后端选择保存下一次运行的配置并要求重启，飞行中、遥测过期或后端没有有效安装证据均拒绝。

## 未完成验收

本轮 L1 **尚未完成**。真实时间扫描传感器和复杂场景、独立 Web、六个新定位算法的构建/实际回放、三个新规划后端、复杂组合三次闭环、完整故障矩阵、CI 和真机部署入口继续按计划实施。当前运行定位仍为 GLIM，规划仍为 A*；新的算法私有源码已开始取得，不代表已经安装或允许运动控制。

## 已发现的环境问题

运行 `20261008T024709Z-39090c79` 在地面待机超过一小时后出现渲染退化：雷达仍为 10Hz、相机 15Hz，但雷达没有有限返回、相机空白；IMU 200Hz 和独立物理位姿正常。定位、导航和飞控健康门禁锁定失败，拒绝解锁。本次已停止，诊断和日志保留。GPU 上下文或渲染异常的具体根因尚未确认，不能认定已经修复。自动验收在 READY 后立即执行，长时间渲染问题仍须在后续故障/稳定性验收中处理。
