# UAV Simulation Lab

面向 PX4 学习、算法验证和巡检仿真的独立实验室。V0.2.0 增加通用实验室、三维雷达、IMU、相机、标定 TF、独立真值和可验证的 rosbag 录制/回放。V0.1 的空旷飞行配置继续作为无 GPU 回归基线，原版[干净环境验收](docs/validation/V0.1-local.md)保留。后续按[版本路线](docs/ROADMAP.md)接入 Mid-360、SLAM、三维导航、任务、Web、AI 和数字孪生。

系统关系和后续接口边界见[架构说明](docs/ARCHITECTURE.md)。

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

构建副本、运行日志和数据均不进入 Git。V0.2 的80项回归、真实传感器、飞行、录制回放及空间几何证据见[本版验收报告](docs/validation/V0.2-local.md)；历次交付记录见[实施记录](docs/IMPLEMENTATION_STATUS.md)。

## 当前范围

当前飞行仍使用 PX4 自身定位。雷达是通用同步三维扫描，不包含 Mid-360 扫描模式或逐点时间。相机为无畸变针孔模型，IMU 无偏置随机游走；没有真实传感器延迟校准。SLAM 和无 GNSS 飞行按后续版本验收。x500 加 0.05kg 学习载荷，未按真实机体校准。实际渲染验收不代表 CUDA 算法已验收。

代码采用 Apache-2.0；外部源码保留各自许可证。
