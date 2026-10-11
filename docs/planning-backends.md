# 四路连续轨迹规划

四条流水线已在同一份传感器占据地图上实际运行。地图已经按机体半尺寸及0.25m余量膨胀，未知单元禁止通行；Gazebo真值不参与规划。每次请求保存地图摘要、初始P/V/A、参数、上游结果、实际曲线及失败原因。

| 后端 | 实际核心与平台接入 |
|---|---|
| `astar` | 平台26邻域A*、保守直线简化、C2最小jerk五次曲线 |
| `ego` | 官方EGO-Swarm `ros2_version`，单机`drone_id=0`；rebound后调用官方固定边界refine API |
| `fast_planner` | 官方动力学A*、观测地图ESDF梯度、NLopt B-spline优化；ROS1参数、日志接口移植到ROS2 |
| `gcopter` | 平台从观测自由空间生成重叠安全盒，官方GCOPTER/MINCO五次轨迹优化 |

锁定版本、源树摘要与许可证见 [依赖锁](../dependencies/planners.lock.json)。官方源码单独放在`.deps`，算法实现直接编译；平台适配器位于`navigation/native`。没有编译上游fake-drone飞行执行器。EGO及GCOPTER的A*引导由平台提供，FAST-Planner实际使用其动力学搜索重新生成优化初值。比较对象是这四条完整流水线。

轨迹归一化为连续ENU/FLU的`odom`系，保留原生位置曲线，仅单独附加连续航向曲线。分析多项式极值并检查整段曲线；超速需要保持同一初始P/V/A重新求解，不能简单放慢正在衔接的曲线。默认速度0.5m/s、加速度0.5m/s²、jerk1m/s³。求解失败不切换算法，HOLD/LAND和飞控命令仍由原适配器处理。

私有构建：

```bash
./scripts/env.sh /usr/bin/python3 scripts/bootstrap_planners.py --backend ego --jobs 1
./scripts/env.sh /usr/bin/python3 scripts/bootstrap_planners.py --backend fast_planner --jobs 1
./scripts/env.sh /usr/bin/python3 scripts/bootstrap_planners.py --backend gcopter --jobs 1
```

FAST-Planner所需NLopt2.7.1也安装在项目私有目录。原生二进制、实际链接库和接口代码具有摘要；变化后旧阶段证据失效，需要重建、重验。GCOPTER沿用官方C++14头文件契约。其优化参数中的2.05kg质量、9.81m/s²重力和零阻力是学习模型假设，真机必须另行标定。

固定地图对照：

```bash
./scripts/env.sh /usr/bin/python3 scripts/benchmark_planners.py <map.npz> \
  --manifest <所属运行/manifest.json> --output <新目录>
```

目前固定用例为正常绕障、非零初始P/V/A、障碍目标拒绝和地图外目标拒绝。输出目录必须全新，四个后端依次运行。平台保存真实求解耗时，原生进程的CPU和RSS单独报告；A*运行在Python工作进程，未测得与原生核心完全等价的独立内存值，不据此作内存排行榜。净距字段是到膨胀地图中障碍/未知单元的保守余量，独立物理几何和实际跟踪评估留给闭环测试。

阶段登记使用`register_planner.py`，安装证据与固定地图回放证据分开。登记前重新读取曲线、核对实际上游位置系数、初始P/V/A、地图与二进制摘要，不能仅凭汇总中的`passed`字段授予资格。当前三个新后端为同步地图组的`replay_passed`，实时及PX4闭环仍须独立验收。

导航节点增加`planner_backend`参数；新后端要求`continuous_trajectory:=true`，将真实优化轨迹传给现有ExecuteTrajectory Action。监督器的组合选择和闭环验收将在下一增量接通。静态地图是本阶段范围，动态物体预测不支持；GCOPTER所用轴对齐走廊不能覆盖所有可行路线，拒绝时保留明确结果。

2026-10-09同地图报告见 [规划对照](validation/l1/planner-comparison.json)。原生初始状态测试3项通过；真实ROS Action测试涵盖三种核心，以及原有成功、取消、源失败、HOLD与拒绝分支，共8项通过。这些接口测试没有替代飞行仿真。
