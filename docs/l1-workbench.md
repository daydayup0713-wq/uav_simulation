# L1 多算法实验台

本轮在原有 Gazebo、RViz 和 CLI 上增加私有算法后端、连续轨迹、六种固定场景及本地 Web 实验台。所有飞控输入仍由同一个 PX4 适配器发布。算法是否可以控制飞行，由当前源码、实际运行证据和指定组合决定。

## 使用 Web

```bash
cd /home/pine/workspace/ai/UAV/uav_simulation
npm --prefix web ci
npm --prefix web run build
./scripts/workbench.sh
```

打开 <http://127.0.0.1:8780>。服务绑定本机回环地址。观测 WebSocket 为 8765，原有 8080/Vite 观测页面仍可使用。浏览器只接收最新显示帧，5Hz，每层最多十万点；录制保留完整接收数据。离线对照仅在对应视图打开时刷新，慢请求不重复排队；同一次资格刷新中相同证据仅认证一次，每次新认证比较两次完整哈希和文件标识，下次操作重新验证。

左侧保存下一次实验的定位、规划、传感器与场景配置。正常启动要求指定组合通过当前版本的闭环资格；失败的方法保留其回放报告。已有实验需保持监督器运行，并添加 `--web`。解锁、起飞、航线和降落是独立显式操作。执行航线后保持悬停，请显式降落。飞行中禁止切换算法；未知、过期、失败状态禁用控制。

“算法对照”按照实际输入组、数据集指纹、回放速率和源时间范围分组，显示 ATE、RPE、姿态、覆盖率、源时间延迟、分阶段 CPU/内存及能力证据。相机接收计数用于核对数据输送；同一文件不保证消息全部到达。“实验记录”包含失败原因，点击带原始跟踪记录的实验可查看误差、速度和加速度曲线。资源统计说明其采样进程范围，不代表整机总占用。不同输入组没有统一排行榜。

“文件回放”直接只读解析本机 SQLite/CDR 录制，不创建 ROS 上下文、不发布 `/clock` 或飞控消息。可选数据集、拖动时间轴、播放/暂停。历史图像与实时控制分离；活动实验需处于新鲜、落地、未解锁状态才能切换显示模式。回放数据在装载时验证原始校验和、标定与时间支持。

在本机 NVIDIA 驱动/动态库不一致时，已独立验证 Intel/Mesa 的 GLX 显示渲染路径。CLI 添加 `--rendering mesa-display`，或在 Web 明确选择“Intel / Mesa 显示渲染”；此模式需要已有 DISPLAY 会话，`--headless` 仍不打开 Gazebo GUI，但不使用 EGL 无显示路径。原 `auto` 默认保留，不能把此兼容配置当作 NVIDIA/CUDA 验证。渲染方式和环境进入运行归档。参照 [Gazebo 官方渲染诊断](https://gazebosim.org/docs/harmonic/troubleshooting/)。

3D 图层分别为原始单帧、配准累积、真实全局优化地图和障碍体素。不存在某种算法的地图输出时显示无数据。全局地图通过实际匹配的局部/全局位姿进行显示配准；GLIM 使用上游私有 `glim_map → glim_odom` TF，不把回环修正送入飞控。GLIM 全局地图的上游 Header 使用墙钟；观察器保留该时间戳，并以当前有效私有 TF 做显示配准，不把它当成传感器源时间质量测量。固定 GLIM 版本的 `odom_corrected` 仍是局部窗口修正，历史报告该话题的精度按“局部修正”解释，不能作为全局优化精度；其全局地图与独立 TF 显示另行验证。左键旋转、右键平移、滚轮缩放；可复位或跟随当前无人机。

## 后端与实际能力

七个定位后端为 GLIM、FAST-LIO2、FAST-LIVO2、FAST-LIVO2-RTK、ORB-SLAM3、LIO-SAM、VINS-Fusion，依赖固定在 `dependencies/backends.lock.json`。四个规划流水线为 A* 连续轨迹、EGO-Swarm 官方 ROS 2 分支单机、FAST-Planner 和安全走廊 + GCOPTER/MINCO，固定在 `dependencies/planners.lock.json`。

每个后端独立前缀、命名空间、源树摘要和运行证据。阶段区分“已配置、已安装、回放通过、实时通过、允许闭环”；启动成功不构成验收。GLIM 保留基线，FAST-LIVO2 + EGO 为重点候选，未预先指定最优组合。FAST-LIVO2-RTK 的实时前端和序列结束后 RTK 优化分别评估；Web 从真实 `rtk_batch` 结果显示完成状态、质量与测量对象，天线轨迹的后处理精度不能冒充机体实时定位精度。

离线回放为观察器和所有子进程创建同一个项目锁定的 Fast DDS 大消息配置，并将配置内容及摘要归档。它不依赖上一次仿真的 `current-run`。旧版普通回归与 Livox 大相机回放之间曾因隐式继承产生严重图像丢帧，相关失败结果保留为历史证据。0.5× 回放属于独立离线实验，不能提升实时或闭环资格。

当前来源及实测值见 [定位报告](validation/l1/localization-comparison.json) 和 [规划报告](validation/l1/planner-comparison.json)。负结果与历史实现标记保留，源码或二进制变动使阶段证据失效。回环、全局优化、重定位分别展示；里程计输出不直接称为完整 SLAM。部署新主机需重新编译，特别是 ORB-SLAM3 的 Eigen/本机编译 ABI。

实际回归、飞行、故障与主机限制汇总见 [本机验收](validation/l1/acceptance.md)；[原始点云界面](validation/l1/workbench-raw.jpg) 和 [四层显示](validation/l1/workbench-live.jpg) 来自真实连续飞行。

## 算法实验命令

```bash
# 已有主机依赖下，在私有前缀构建指定算法。jobs 限 1 或 2。
./scripts/env.sh /usr/bin/python3 scripts/bootstrap_backends.py --backend fast_livo2 --jobs 1
./scripts/env.sh /usr/bin/python3 scripts/bootstrap_planners.py --backend ego --jobs 1
./scripts/build.sh

# 固定本机数据集回放；输出目录须为新目录，失败报告不会覆盖。
LAB_DOMAIN_ID=77 ./scripts/env.sh /usr/bin/python3 scripts/replay_backend.py \
  --backend fast_livo2 --dataset recordings/<数据集> --output .runtime/<新回放目录>

# 同一观测并膨胀的地图上顺序对照四个真实规划器。
./scripts/env.sh /usr/bin/python3 scripts/benchmark_planners.py <已归档-map.npz> \
  --manifest <所属运行-manifest.json> --output .runtime/<新规划对照目录>

# 受控仿真验收入口，必须已有对应回放证据；不自动提升资格。
./scripts/env.sh /usr/bin/python3 scripts/accept_continuous.py \
  --output .runtime/<新闭环验收目录> --runs 3 --profile navigation \
  --localization-backend glim --planner-backend ego --scene circle-eight --qualification

# 六场景、四规划路线、候选组合与故障顺序执行；保留每项失败。
./scripts/env.sh /usr/bin/python3 scripts/accept_workbench.py --output .runtime/<新矩阵目录>

# Web/CLI 共同使用的分组报告导出。
./scripts/env.sh /usr/bin/python3 scripts/workbench_reports.py --output .runtime/<新报告.json>
```

长航线采用固定 12×12×6m、0.2m 观测地图窗口。远处语义点使用已观测局部目标衔接，不把局部目标算成已完成语义点。未知区域保持阻塞，不扩展自主探索。长走廊另使用 360×128 同步传感器配置；其输入身份与实际算力开销单独记录，旧 180×64 导航回归配置保留。

连续轨迹为时间参数化 P/V/A、yaw/yaw-rate，50Hz 执行，20Hz 心跳。默认速度 0.5m/s、加速度 0.5m/s²、jerk 1m/s³；分析极值并检查整段机体包络、未知区域、地理边界和重规划衔接。每个语义中间点连续通过，最终目标或明确停留点稳定后完成。不可行目标、超时、未知空间或超限轨迹返回明确失败，不自动换算法。

定位与通信失效沿用已验证降落机制，恢复后不会自动续飞。导航中的 HOLD 使用有源时间、已按机体包络膨胀的观测地图检查整段停止曲线；未知、障碍、过期或缺失地图使停止请求失败并进入配置的降落流程。停止曲线及地图另行归档。LAND 可打断运动且启动后继续完成。CLI 在发送真实动作前确认冷启动回复路径：飞行动作使用必拒绝的操作 255，导航使用必拒绝的空坐标系目标或空航线；每次探测最多 1 秒、至多三次。未确认时不提交运动，真实运动命令不重试。

## CI 与本机验收

原有 `ci_acceptance.sh` 的三次飞行及桥接退出降落保留。新增前端测试/构建，以及 `containers/Dockerfile.workbench` 的真实 GLIM 和三个原生规划器 CPU 构建。CPU 解析观测固定基准清楚标注为解析合成，不能代替传感器渲染或真实 PX4 闭环。

本机等价运行：

```bash
./scripts/env.sh env PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest -q
npm --prefix web test
npm --prefix web run build
./scripts/ci_acceptance.sh
./scripts/env.sh /usr/bin/python3 scripts/ci_workbench.py --output .runtime/<新CPU验收目录>
```

所有物理飞行验收顺序运行，同一时间只运行一个定位和一个规划后端。CPU 固定基准不包含相机/GPU 点云渲染。Docker 镜像需由 CI 实际构建后才能宣称容器复现通过；本机没有 Docker CLI。

## 真机准备

[Pixhawk 6X 配置模板](../configs/hardware/pixhawk6x.template.json) 和 [部署准备](hardware-preparation.md) 提供明确入口。当前仅验证仿真，实际机体、相机、RTK、机载计算机和时延参数尚未提供。静态配置完整、仿真资格与真机试飞资格分别处理。
