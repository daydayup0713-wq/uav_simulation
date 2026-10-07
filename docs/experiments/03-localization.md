# 实验03：从配准到 LIO 与 SLAM

输入是当前通用同步点云、无姿态先验的 IMU 和完整固定数据集。先运行离线基准，观察 ICP 的匹配门限、连续 LIO、全局轨迹，以及独立真值误差。

```bash
./scripts/env.sh /usr/bin/python3 scripts/bootstrap_slam.py --jobs 2
./scripts/build.sh
./scripts/labctl slam benchmark recordings/20261004T103659Z-5d426e73-104356-128d1b --output .runtime/my-v03-benchmark
./scripts/labctl slam map-save .runtime/my-v03-benchmark maps/my-lab
```

预期 LIO 基准通过，报告保留原始/对齐误差、覆盖、时间间隔、依赖与输入哈希。ICP 学习基线可能在几何变化或退化时明确停止；不要只看停止之前的小误差。输出目录不可覆盖，重复实验使用新目录。

实时实验先使用 `--profile localization --rviz`，观测局部连续轨迹和地图全局修正。正式无 GNSS实验使用：

```bash
./scripts/start_lab.sh --profile slam --rviz
# 另一终端，收到 LAB READY 后：
./scripts/labctl slam status
./scripts/labctl status
./scripts/labctl slam map-load maps/my-lab
./scripts/labctl slam relocalize --initial 0 0 0 0 0 0
./scripts/labctl demo --runs 3
```

`--initial` 是 map 中机体的 x y z roll pitch yaw，角度单位弧度；此例假设已知的实验室起始区域。地图操作只在新鲜地面未解锁状态接受，匹配失败保留原变换。map→odom 可变化，飞控连续目标不随之跳动。先通过 `external_ready` 检查实际外部位置/高度/航向融合及 GNSS未融合，再显式解锁。PX4 当前地面模式的 preflight=False 并不等于 Offboard无法解锁；ARM仍等待 Offboard ACK、实际模式及其预检，之后才发送解锁命令。

定位失效或心跳过期会锁定失败；恢复通信后必须重新启动实验，不自动续飞。无 GNSS配置关闭GNSS融合及有效卫星输入，使用外部位置/高度/航向；它仍使用 PX4 的惯性数据，不是绕过飞控状态估计。

算法只接收 `localization/input/imu` 和 `localization/input/points`。入口校验源坐标、有限数值以及有效距离内的点数和空间分布。单个空扫描会被丢弃并记录 `rejected_scans`，不送入 GLIM；持续缺测触发源时间 watchdog，之后不自动恢复。错误坐标、IMU 合约和时间倒退直接锁定失败。离线基准遇到无效扫描返回失败，便于定位数据问题。

独立真值评估及定位退出故障实验：

```bash
./scripts/labctl record --duration 175
# 录制完成后，以输出的数据集目录和当前运行轨迹评估：
./scripts/env.sh /usr/bin/python3 scripts/evaluate_live.py recordings/<数据集> .runtime/<运行>/localization.jsonl --output .runtime/live-report.json
./scripts/labctl arm
./scripts/labctl takeoff --height 2
./scripts/env.sh /usr/bin/python3 scripts/verify_fault.py --component lio
```

故障验证器只终止当前运行的 LIO 进程组，独立订阅 PX4 确认空中 AUTO_LAND、落地和解除武装。监督进程保留物理仿真60秒，故障后解锁命令应被拒绝。CI 使用解析生成的无 GPU 观测夹具运行真实 CPU GLIM；该结果仅验证构建/回放，不能代替渲染传感器或真实飞行精度验收。

故障定位顺序：查看 `slam status` 的 source_age/原因 → 运行目录 localization/lio 日志 → bridge 的融合标志和命令ACK → PX4日志与ULog。`source_age`持续增加通常表示计算落后或源时间异常；质量FAILED后不要重复尝试解锁。RViz应使用 MoveCamera 工具，并以map为Fixed Frame。

地图和录制目录不进入普通Git历史。重定位、传感器渲染、原生飞行及CI结果分别记录，仿真通用雷达结果不能替代Mid-360或真实机传感器标定。
