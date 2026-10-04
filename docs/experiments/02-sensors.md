# 第二组学习实验：从传感器到可回放数据

先启动 `./scripts/start_lab.sh --profile sensors --rviz` 并等待 LAB READY。实验室墙与家具都有碰撞几何；默认四航点路线有净空，任意手动航点尚无避障保障。

| 实验 | 输入什么 | 观察什么 | 预期变化 | 失败定位 |
|---|---|---|---|---|
| 检查数据链 | `labctl sensors --duration 10` | 每流 count/source_hz、errors、missing_transforms、实时率 | 雷达10Hz、IMU200Hz、图像及内参约15Hz、真值25Hz，passed=true | sensor-bridge.log 检查桥接，sensors.log 检查节点，gazebo.log 检查渲染 |
| 从点云认识空间 | 起飞至2m，在 RViz 查看 Generic 3D lidar；切换固定系 lidar_link/odom | 近处墙、柜体、立柱；高度导致地面点云变化 | lidar_link 下为局部扫描；odom 下随 PX4 估计位姿累积观察 | 检查 /tf_static、base_link→lidar_link 以及点云 header.frame_id |
| 图像与姿态 | `goto --x 0 --y 0 --z 2 --yaw 45`，再回 yaw0 | Inspection camera 中检查板和墙体改变 | 朝向变化与相机前向一致；光学系 z 前、x 右、y 下 | CameraInfo/image 必须同 optical frame；不要把 FLU 当光学系 |
| 认识 IMU | 查看 `/uav001/imu/data`，悬停并改变航点 | linear_acceleration、angular_velocity、orientation_covariance | 悬停比力范数约9.81m/s²；姿态 covariance[0]=-1，算法不可使用理想姿态 | 重力方向和 FLU；该传感器不是 PX4 姿态估计话题 |
| 认识独立真值 | 查看 `/uav001/ground_truth/odometry` 与 `/uav001/odometry` | sim_world/truth_base_link 与 odom/base_link | 地面真值机体中心约0.227m，PX4 local z约0；不能直接相减当绝对误差 | 先确定坐标原点配准；真值不进入控制 TF 树 |
| 录制一次飞行 | 第三终端 `labctl record --duration 80`，操作终端 `demo --runs 1` | recordings 中 bag、dataset.json、configuration、run-manifest.json、audit.json | 11个显式话题都有数据；配置、标定及依赖可追溯 | recorder.log；complete=false 表示中断或未通过契约，不能当已验收数据 |
| 离线复核 | `labctl bag-check recordings/<目录>` | 源频率、时间顺序、点云结构、相机标定、TF、话题计数 | passed=true，配置/包文件 SHA256 一致 | 编辑或移动外部文件会导致 checksum/path 错误；保留整个数据集目录 |
| 隔离回放 | `labctl replay recordings/<目录> --domain 77`；另一终端 LAB_DOMAIN_ID=77 启 RViz | 已记录 /clock 和传感器，replay-audit.json | 播放器退出0，live audit通过；活动飞行域42保持独立 | 共域或已有数据发布者会拒绝；先关闭该域旧播放器。RViz单独观察可共域 |

命令都通过 `./scripts/labctl` 执行。直接 echo 传感器建议用 `./scripts/env.sh ros2 topic echo /uav001/imu/data --qos-reliability best_effort`。查看静态变换使用 `--qos-durability transient_local`。图像和点云量较大，避免长期在终端打印全文。

验收频率按源时间计算；`real_time_factor` 则比较 /clock 与墙钟。低实时率不等于源频率改变。源数据最大间隔不得超过 `max(0.25秒, 5/配置Hz)`，防止长时间断流被平均频率掩盖。实时检查还要求 /clock 在最近2秒墙钟内推进，传感器接收超时按实时率调整；暂停仿真会明确失败。离线检查不受文件读取速度影响。

回放按 bag 的接收时间轴播放，并据此计算超时；在低实时率下，墙钟录制时长可能大于源时长。回放采用 best effort，报告实际接收数量和最大间隔，不宣称无损传输；算法基准应从 bag 文件离线读取并检查间隔。录制被 SIGINT/SIGTERM 中断时关闭本次 recorder 进程组，数据标记为 complete=false，保留诊断和已写入文件。

通用雷达白噪声0.01m，无逐点时间；IMU只含白噪声；相机无畸变。完整 SLAM、去畸变、Mid-360 近似和标定误差实验将在后续版本交付。
