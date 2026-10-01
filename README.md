# UAV Simulation Lab

面向 PX4 学习、算法验证和巡检仿真的独立实验室。当前交付 **V0.1 候选版**：单架 x500、Gazebo Harmonic 空旷世界、ROS 2 Humble 飞控适配、CLI 和 RViz 轨迹。后续按[版本路线](docs/ROADMAP.md)接入传感器、SLAM、三维导航、任务、Web、AI 和数字孪生。

系统关系和后续接口边界见[架构说明](docs/ARCHITECTURE.md)。

## 本机直接运行

在 Ubuntu 22.04 / 系统 Python 3.10 中执行。脚本只加载系统 Humble 和本仓库的 ROS 工作区。

```bash
cd /home/pine/workspace/ai/UAV/uav_simulation
/usr/bin/python3 scripts/doctor.py --runtime
./scripts/start_lab.sh --rviz
```

等待终端输出 `LAB READY`。Gazebo 显示飞行器，RViz 使用 `odom` 观察 `/uav001/path`。另一终端执行：

```bash
cd /home/pine/workspace/ai/UAV/uav_simulation
./scripts/labctl status
./scripts/labctl demo --runs 3
```

演示每轮自动完成：显式解锁 → 起飞至相对起点 2m → 悬停 10 秒 → 3m 方形四航点 → 返回起点 → 降落并确认解除解锁。默认航点生成速度上限 1m/s，位置误差不超过 0.3m 并持续 2 秒才完成动作。

无界面运行使用 `./scripts/start_lab.sh --headless`。需要单独打开 RViz 时：

```bash
./scripts/env.sh rviz2 -d configs/lab.rviz
```

先降落，再在启动终端按 Ctrl+C，或执行 `./scripts/env.sh /usr/bin/python3 scripts/stop_lab.py`。停止脚本只操作本次监督进程；关闭整个实验室会同时停止物理仿真。测试飞控失控降落请使用下文的故障验收脚本。

## 独立安装与构建

已有 ROS/Gazebo 和编译工具的本机：

```bash
/usr/bin/python3 scripts/doctor.py
/usr/bin/python3 scripts/bootstrap.py --jobs 2
./scripts/build.sh
/usr/bin/python3 scripts/doctor.py --runtime
```

新 Ubuntu 22.04 主机可执行 `bash scripts/install_host.sh`。该脚本安装 apt 源和依赖，需要 sudo；遇到 Humble 默认 Fortress/Garden 桥接包会停止，要求先处理混装。当前主机已有依赖，本次没有重新运行系统安装脚本。

PX4、px4_msgs、Agent 及 Agent 四个主要源码依赖的提交记录在 [lock.json](dependencies/lock.json)。已有 PX4/px4_msgs 仅作为 Git 对象缓存；下载副本不带入原工作区的修改。Agent 私有安装，关闭本实验室不需要的 P2P 功能；私有 DDS 库仅加载到 Agent 进程。构建并发默认 2，适合本机约 16GB 内存。

Humble + Harmonic 使用 `ros-humble-ros-gzharmonic`，见[官方组合说明](https://gazebosim.org/docs/harmonic/ros_installation/)。Agent 2.4.3 对应 PX4 2.x Client，见[PX4 ROS 2 说明](https://docs.px4.io/main/en/ros2/user_guide)。本仓库以固定源码为准，不随文档 main 分支更新版本。

## 手动实验

```bash
./scripts/labctl status
./scripts/labctl arm
./scripts/labctl takeoff --height 2
./scripts/labctl goto --x 3 --y 0 --z 2 --yaw 0
./scripts/labctl hold
./scripts/labctl land
./scripts/labctl disarm
```

`goto` 使用 `odom` 中的 ENU 米坐标，`--yaw` 为绕上方向逆时针的角度，0° 指向东。`takeoff --height` 是相对当前地面起点的高度。普通 DISARM 仅允许已落地且没有活动动作的飞行器。运动中可用另一个终端发送 hold 或 land；重复运动请求拒绝。取消起飞/航点进入悬停，已经开始的降落继续完成。

所有失败均返回非零退出码并输出原因。更多语义见[接口契约](docs/INTERFACES.md)和[学习实验](docs/EXPERIMENTS.md)。

## 验证与记录

```bash
./scripts/env.sh env PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest -q
./scripts/labctl demo --runs 3
# 以下是有意终止适配节点的独立故障实验，需要已起飞：
./scripts/labctl arm
./scripts/labctl takeoff --height 2
./scripts/env.sh /usr/bin/python3 scripts/verify_fault.py
```

故障验证器直接订阅 PX4 状态；适配器退出后，监督进程保留 Gazebo/PX4 60 秒，供失控降落观测，随后清理本次进程。不会自动重启适配器或续飞。完整自动验收为 `bash scripts/ci_acceptance.sh`；CI 定义见 [.github/workflows/ci.yml](.github/workflows/ci.yml)，容器环境见 [containers/Dockerfile](containers/Dockerfile)。

每次运行保存于 `.runtime/<UTC运行编号>/`，`.runtime/current-run` 指向当前记录：

| 文件 | 内容 |
|---|---|
| `manifest.json` | 依赖、参数、域、Gazebo 分区、监督进程身份 |
| `configuration/` | 世界、启动参数、RViz 和依赖配置副本，manifest 保存 SHA256 |
| `processes.json` | 本次创建的进程组 |
| `events.jsonl` | 状态变化、命令、ACK、遥测、估计器修正 |
| `acceptance.json` | 三次演示及悬停误差 |
| `fault-acceptance.json` | 独立故障观测结果 |
| `px4/log/**/*.ulg` | PX4 飞行日志 |
| `*.log`、`ros/` | 各组件输出及 ROS 日志 |

构建副本、运行日志和数据均不进入 Git。实际验证结果与限制写入[实施记录](docs/IMPLEMENTATION_STATUS.md)。

## 当前范围

V0.1 是使用 PX4 自身定位的飞行基础设施。通用实验室的室内几何、雷达、相机和独立真值评估属于 V0.2；SLAM 和无 GNSS 飞行按后续版本验收。当前 x500 是学习模型，未按真实机体尺寸、惯量、载荷和传感器校准。GPU 枚举可用不代表相机、雷达渲染或 CUDA 算法已验收。

代码采用 Apache-2.0；外部源码保留各自许可证。
