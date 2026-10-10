# UAV Simulation Lab

面向 PX4 学习、算法验证和巡检仿真的独立实验室。L1 升级增加七个定位后端、四条规划流水线、连续轨迹、六种复杂场景和本地 Web 点云/对照/回放实验台。Gazebo、RViz、CLI 与 V0.1～V0.4 回归入口保留；算法阶段与实际失败结果单独记录。

[实验台操作与算法限制](docs/l1-workbench.md) · [本机验收与实际失败结果](docs/validation/l1/acceptance.md) · [真机准备入口](docs/hardware-preparation.md)

```bash
npm --prefix web ci
npm --prefix web run build
./scripts/workbench.sh
# 浏览器打开 http://127.0.0.1:8780
```

正常飞行仅允许当前实现与指定组合通过闭环资格的后端；Web 文件回放只读本机录制，控制禁用。后续按[版本路线](docs/ROADMAP.md)建设巡检任务、AI 和数字孪生。

系统关系和后续接口边界见[架构说明](docs/ARCHITECTURE.md)。

## 三维导航实验

```bash
./scripts/start_lab.sh --profile navigation --rviz
# 另一终端，等待 LAB READY：
./scripts/labctl nav status
./scripts/labctl nav demo --runs 3
```

导航专用通用雷达64×180、约179°垂直视场；目标需处于已观测空间。三维路径绕过实验室挡板，参考速度0.5m/s、加速度0.5m/s²。人工HOLD/LAND和数据失效会阻止续飞。操作、限制及故障定位见[导航实验](docs/experiments/04-navigation.md)，数值见[V0.4验收](docs/validation/V0.4-local.md)。

## 定位与地图实验

```bash
./scripts/env.sh /usr/bin/python3 scripts/bootstrap_slam.py --jobs 2
./scripts/build.sh
./scripts/start_lab.sh --profile slam --rviz
# 另一终端，等待 LAB READY：
./scripts/labctl slam status
./scripts/labctl demo --runs 3
```

`slam` 配置关闭 GNSS 输入/融合，飞控适配只接收通过质量检查的连续 LIO。`localization` 配置保留 GNSS，供比较定位输出。私有 GLIM 后端只使用 CPU 两线程；雷达/相机仍需 Gazebo 渲染。新主机的系统依赖由安装脚本提供，本机已具备 GTSAM4.2、Boost 和 TBB。

离线基准、ICP 学习、地图保存/加载、带粗位姿的重定位和故障实验见[定位实验](docs/experiments/03-localization.md)，算法版本及选择依据见[算法说明](docs/algorithms-v03.md)，实测结果见[V0.3 验收](docs/validation/V0.3-local.md)。

## 传感器实验室

```bash
cd /home/pine/workspace/ai/UAV/uav_simulation
/usr/bin/python3 scripts/doctor.py --runtime --sensors
./scripts/start_lab.sh --profile sensors --rviz
```

等待 `LAB READY`，在另一终端运行：

```bash
./scripts/labctl sensors --duration 10
./scripts/labctl demo --runs 1
# 可在飞行期间从第三个终端录制：
./scripts/labctl record --duration 30
```

录制完成会输出数据集路径（位于 `recordings/`），随后可检查和回放：

```bash
./scripts/labctl bag-check recordings/<数据集目录>
./scripts/labctl replay recordings/<数据集目录> --domain 77
# 回放时在另一终端观察，域需与回放一致：
LAB_DOMAIN_ID=77 ./scripts/env.sh rviz2 -d configs/sensors.rviz
```

回放使用独立 ROS 域，拒绝与活动实验室共域，拒绝已有数据/飞控发布者，也拒绝包内出现白名单之外的话题。只播放已记录的 `/clock`，避免双时钟。记录中断或校验失败的数据集保留为 `complete: false`，命令返回非零。

雷达 360×16 点、10Hz；相机 320×240 RGB、15Hz；IMU 200Hz（比力及角速度，姿态不可用）；独立真值 25Hz。安装参数和噪声见 [sensors.json](configs/sensors.json)，运行时生成模型、外参和桥接配置并归档。Gazebo 真值在 `sim_world` 中，PX4 自身估计在 `odom` 中，二者不能直接当成同一坐标系。学习步骤、话题和故障定位见[传感器实验](docs/experiments/02-sensors.md)。

`--profile sensors --headless` 关闭窗口但仍需 Ogre2 渲染设备；`--profile flight --headless` 不需要相机或 GPU 雷达。相机和点云必须实际收到、验证通过才会就绪。NVIDIA 枚举与 CUDA 能力分别验收，本平台不以 `nvidia-smi` 成功代替渲染测试。

## 本机直接运行

在 Ubuntu 22.04 / 系统 Python 3.10 中执行。脚本只加载系统 Humble 和本仓库的 ROS 工作区。

```bash
cd /home/pine/workspace/ai/UAV/uav_simulation
/usr/bin/python3 scripts/doctor.py --runtime
./scripts/start_lab.sh --rviz
```

等待终端输出 `LAB READY`：遥测新鲜、已落地、未解锁，PX4 飞前检查连续通过 5 秒。Gazebo 显示飞行器，RViz 使用 `odom` 观察 `/uav001/path`。另一终端执行：

`labctl` 是操作客户端，不会启动实验室。电脑重启或实验室退出后，需要重新执行 `start_lab.sh`，并保持启动终端运行；两个终端若设置了 `LAB_DOMAIN_ID`，其值必须相同。

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

RViz 顶部选择 `Move Camera`，在三维画面内按住左键拖动可旋转；按住中键或 `Shift + 左键` 拖动可平移，滚轮可缩放。配置显式加载相机工具，并将其设为默认工具。

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

构建副本、运行日志和数据均不进入 Git。V0.4三维导航与回归证据见[本版验收报告](docs/validation/V0.4-local.md)；V0.3 LIO/SLAM及无GNSS证据见[历史定位报告](docs/validation/V0.3-local.md)；V0.2 的传感器和录制回放证据见[历史报告](docs/validation/V0.2-local.md)；历次交付记录见[实施记录](docs/IMPLEMENTATION_STATUS.md)。

## 当前范围

默认 flight/sensors 配置使用 PX4 原定位；slam 与已取得指定组合资格的实验台配置使用外部里程计。原通用同步雷达保留，另有带实际采样时刻和运动测量的 Livox 类、机械雷达及 RTK 实验配置；这些近似模型尚未证明与真实 Mid-360 一致。

导航只在传感器已观测的静态空间内检查机体包络，未知空间拒绝通行；不包含自主探索、动态障碍预测或完整起降避障。直接飞控航点不经过导航碰撞检查。GLIM 粗位姿重定位与各后端回环、重定位能力分别记录，观测噪声下限及失败门禁仍明确保留。相机、IMU、机体与通信时延尚未完成实机标定；算法回放通过不等于实时或真机飞行通过。

代码采用 Apache-2.0；外部源码保留各自许可证。
