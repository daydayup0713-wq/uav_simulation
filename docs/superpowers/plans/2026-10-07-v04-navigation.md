# V0.4 Navigation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: superpowers:executing-plans. Inline implementation, one fresh final whole-branch review.

**Goal:** 实时三维占据地图、三维规划与碰撞检查、低速自动绕障和不可达确定结果。
**Architecture:** 独立Python导航包，连续odom，唯一PX4适配器执行；传感器/估计与真值评估分离。
**Tech Stack:** Humble rclpy、NumPy/SciPy、固定GLIM CPU、Harmonic/PX4。
**Spec:** docs/superpowers/specs/2026-10-07-v04-navigation-design.md

## Global Constraints

- 用户已授权原总体路线及继续V0.4，延续当前feature checkout与inline方式，无额外审批停顿。
- 0.2m网格，范围x/y -6..6、z -1..5m；机体半尺寸(.4,.4,.30)+裕度.25m。
- ENU/FLU与连续odom；只用点云/LIO/冻结适配，禁止真值/世界几何进入算法。
- 0.5m/s、0.5m/s²、0.15m连续2秒；搜索50000展开/2秒，重规划最多3次。
- 未知区域阻止通行；旧profile行为保留；系统Python3.10、私有依赖，无全局安装。
- 每项实质行为RED→GREEN；完整回归、原生3次/故障、最终CI后发布。

## Review Focus

- 斜线擦角和点云边界裁剪不得放行碰撞；Task1/2测试。
- 未知/陈旧数据与坐标变换不一致不得继续飞行；Task3/4测试。
- 外部HOLD/LAND及取消与重规划竞争不得续飞；Task4真实ROS测试。
- 底层拒绝/超时/节点退出不得误报成功；Task4/6测试。
- 预算耗尽不得误称不可达，几何只供评估；Task2/5测试。

### Task 1: Occupancy and fixed observation benchmark

**Files:** navigation/occupancy.py、benchmark.py、tests/test_occupancy.py、configs/navigation.json。
**Interfaces:** VoxelMap(resolution,lower,upper); integrate(origin,endpoints)、observe_body(position,halfsize)、snapshot(halfsize)->CollisionMap；cell state UNKNOWN/FREE/OCCUPIED；version/last source metadata。

- [ ] 测试手工射线空闲/端点占据/遮挡未知、端点优先、裁剪、恶意数值、边界/未知膨胀、真实机体体积不能清障碍。运行pytest tests/test_occupancy.py，Expected: missing feature。
- [ ] 实现有界网格/射线/体积查询并跑聚焦测试，Expected: pass。
- [ ] 保存固定解析观测、哈希及数值基准；真实ROS对齐另在Task3验证。
- [ ] Commit occupancy and fixed-input contract。

### Task 2: 3D planner and constrained trajectory

**Files:** navigation/planner.py、bridge/trajectory.py、controller.py、tests/test_navigation_planner.py、tests/test_trajectory.py。
**Interfaces:** plan(collision,start,goal,max_expansions=50000,timeout=2)->Plan(success,reason,points,expanded); collision.segment_clear(a,b)；MotionProfile(start,goal,speed,acceleration).sample(t)->position,velocity。

- [ ] 测试绕挡板、真正三维路线、封闭/未知目标、非有限目标、角切碰撞、预算独立原因、平滑所有线段安全。Expected: missing feature。
- [ ] 实现有界A*与验证后的shortcut，聚焦测试Expected: pass。
- [ ] 测试三角/梯形短长段的速度/加速度界、端点无超调、零长度、停稳及旧控制器默认行为；实现opt-in平滑轨迹。
- [ ] 固定数据地图→路径基准，记录展开/时间/路径长度和所有段检查；commit。

### Task 3: Sensor-time aligned navigation map

**Files:** navigation/poses.py、node.py、interfaces/PlanPath.srv、localization_node.py、sensor_model.py、configs/navigation-sensors.json、simulation/worlds/navigation.sdf、tests/test_navigation_poses.py、tests/ros/test_navigation_map.py。
**Interfaces:** PoseHistory.add(stamp,pose)/at(stamp)；冻结lio_odom←odom话题；NavigationNode map/status/path，PlanPath(goal)->result。

- [ ] 测试插值/外参/冻结变换/不外推；真实ROS序列化和源时间地图、新鲜与过期、禁止真值订阅；导航专用雷达/世界生成结果。Expected: missing feature。
- [ ] 实现定位适配话题、宽视场profile和源时间地图；地图计算/服务独立回调组，心跳不因规划阻塞。
- [ ] 构建并运行聚焦ROS测试，Expected: pass，实际读取路径与占据消息。
- [ ] 原生导航profile启动、录制新雷达固定数据和独立地图几何核验，commit。

### Task 4: Navigation execution and control loss gates

**Files:** navigation/node.py、cli.py、Navigate.action、ExecuteFlight.action、bridge/node.py、scripts/labctl、supervise.py、tests/ros/test_navigation_action.py、tests/ros/test_bridge.py。
**Interfaces:** navigation/navigate(goal)->success/reason feedback；ExecuteFlight.navigation默认False；navigation diagnostics ready/fresh给sole bridge，opt-in navigation_required。

- [ ] 真实ROS测试多段航点、重复拒绝、取消/HOLD/LAND不续飞、障碍更新重规划/预算、底层拒绝/超时、陈旧源/诊断锁定与恢复不续飞。Expected: missing feature。
- [ ] 实现Action与CLI以及桥接门禁；只导航profile允许guardedGOTO，无新FMU发布者。
- [ ] 全量回归与真实ROS动作，Expected: pass；commit。

### Task 5: Native bypass and independent evaluation

**Files:** scripts/verify_navigation.py、CLI demo、configs/navigation.rviz、docs/experiments/04-navigation.md、docs/validation/v04/。
**Interfaces:** 原生3轮/不可达、导航进程退出→独立PX4降落；真值/场景几何仅进入verify_navigation。

- [ ] 演示起飞→自动绕障可见目标→返回→落地，记录实际地图/每段/状态/输入版本；Expected: 无碰撞，全部到达/落地/解除武装。
- [ ] 独立轨迹与场景碰撞体比较，记录实际净距/绕行/速度；障碍内目标失败且无运动，重复/cancel分别确认。
- [ ] 原生连续3轮、navigation进程退出失控降落及恢复不续飞；Expected: 全部通过。
- [ ] 文档输入/观察/失败定位、限制及数值证据；commit。

### Task 6: Regression, review and release

**Files:** CI、README、interfaces/roadmap/status、版本、验收及取舍。

- [ ] 全量回归、8ROS包/消息核验、CPU固定观察地图/3D路径CI，保留既有3轮/断桥及CPU LIO门禁。
- [ ] 一次独立最终全分支审查；material问题一次RED→GREEN修复并回归，Expected: 无未解决重要问题。
- [ ] 小步提交/PR#4基于V0.3、最终提交CI、v0.4.0标签、归档当前执行记录，Expected: 验收与限制可追溯。
