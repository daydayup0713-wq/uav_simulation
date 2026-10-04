# V0.2.0 传感器实验室设计

本设计细化用户已批准的总体计划和“继续完成开发”指令。用途是观察、记录、回放传感器数据，为后续固定数据集 LIO 基准准备输入。执行方式为当前会话内增量实现。

## 场景与资源

保留 `flight` 配置（原空旷世界、标准 x500），新增 `sensors` 配置：20×16m 实验室、5m 高墙、工作台、柜体、立柱与彩色检查板；原点到 (3,3) 的四航点路线保持净空。配置通过 `start_lab.sh --profile sensors --rviz` 选择。世界使用本地几何和原有固定 PX4 模型，不依赖在线模型下载。

新增固定载荷 0.05kg，安装姿态均相对 PX4 `base_link`（FLU）。通用雷达为 360×16、360°×30°、0.15–20m、10Hz 的同步扫描；不是 Mid-360，不提供逐点时间。相机 320×240 RGB、15Hz、水平视场 90°、无畸变针孔模型。算法 IMU 200Hz，只提供 FLU 角速度和比力；不把 Gazebo 理想姿态作为算法观测。雷达距离白噪声 0.01m；IMU 角速度/加速度白噪声 0.0002rad/s、0.01m/s²，无随机游走或时间延迟。

## 数据契约

| ROS 话题 | 消息 | 坐标系 | 标称频率 |
|---|---|---|---|
| `/uav001/lidar/points` | PointCloud2 | lidar_link（FLU） | 10Hz |
| `/uav001/imu/data` | Imu | imu_link（FLU） | 200Hz |
| `/uav001/camera/image_raw` | Image | camera_optical_frame | 15Hz |
| `/uav001/camera/camera_info` | CameraInfo | camera_optical_frame | 15Hz |
| `/uav001/ground_truth/odometry` | Odometry | sim_world → truth_base_link | 25Hz |

`configs/sensors.json` 是安装外参和数据参数的来源，生成 SDF、桥接配置与静态 TF，运行时归档。雷达位置 (0,0,0.16)m；相机 (0.2,0,0.06)m；IMU 原点。相机机体系为 FLU，光学系为右、下、前。静态 TF 只增加 base_link 的传感器子节点；独立真值不发布到控制 TF 树，`sim_world` 与 PX4 局部 `odom` 不假定重合。

所有传感器保留 Gazebo 源时间，使用 `/clock`；超时按单调时钟计算。桥接仅 Gazebo→ROS，禁止新组件发布 `/fmu/in/*`。真值仅供数据评估，禁止输入飞控、SLAM 或规划器。无渲染设备时传感器启动失败需指出缺失流，并保存日志；`flight` 配置继续支持无 GPU CI。

## 记录、回放与验收

`labctl sensors --duration 10` 核验源频率（标称的 80–120%）、严格递增的源时间、帧名、点云结构与有限点、图像大小/编码、内参、IMU 比力与 TF。同时报告仿真实时率，墙钟低实时率不误判成源频率错误。消息过旧、流缺失、空数据、错帧和时间回退明确失败。

`labctl record --duration 30` 使用显式白名单 rosbag2 sqlite3 录制，含传感器、CameraInfo、/clock、/tf、/tf_static、平台遥测与独立真值；禁止飞控输入和内部原始流。归档运行 manifest、生成模型/场景、外参、依赖版本、QoS 和录制结果。录制开始前要求 sensors 配置已就绪；结束检查全部必需话题非空。

`labctl bag-check <dataset>` 读取并反序列化数据、验证数据契约、时间与静态 TF。`labctl replay <dataset>` 在独立 ROS 域（默认 77）播放显式话题白名单，使用已记录的 /clock；拒绝与活动实验室共域，并拒绝该域存在其他发布者，不播放飞行命令。回放结束退出码反映播放器和数据检查结果。RViz 可用 `LAB_DOMAIN_ID=77 ./scripts/env.sh rviz2 -d configs/sensors.rviz` 观察。

发布条件：V0.1 回归通过；本机实际雷达/IMU/相机数据验收通过；实验室内飞行演示三次；完整数据录制、离线验证和隔离域回放通过；CPU CI 继续使用 flight 配置。渲染验收与 CI 分开记录。V0.2.1 的 Mid-360、SLAM、规划、Web 不属于本版。
