# V0.1 接口契约

## 职责与坐标

只有 `uav_lab_bridge` 发布 `/fmu/in/*`。发现其他发布者时停止控制并锁定故障。平台对外为 ENU/FLU；位置由 PX4 NED `(x,y,z)` 转为 ENU `(y,x,-z)`，姿态同时转换世界坐标和 FRD/FLU 机体坐标。Odometry 的速度在 `base_link` 中表达。

TF 为 `odom → base_link`，当前 odom 跟随 PX4 的固定局部位置原点。控制期间 XY/Z 重置会失败并停发 Offboard 心跳；航向修正超过 0.15rad 同样失败。PX4 正常离地磁对齐产生的小幅航向估计修正允许继续，记录修正值，ENU 目标保持不变。开始控制前允许定位初始化重置。

## 服务

命名空间 `/uav001`，类型均为 `std_srvs/srv/Trigger`。`success=false` 时 `message` 给出原因。

| 名称 | 条件与完成依据 |
|---|---|
| `arm` | 新鲜有效遥测、已落地、未解锁、无活动操作；预热 2 秒 → Offboard ACK+模式+飞前检查通过 → 解锁 ACK+状态 |
| `disarm` | 已落地且无活动操作；等待 ACK 与 disarmed 状态 |
| `hold` | 已解锁的 Offboard 飞行器，且尚未开始降落；打断运动，以当前观测位姿建立悬停目标 |

## 飞行动作

`/uav001/execute_flight`，类型 `uav_lab_interfaces/action/ExecuteFlight`。

| 字段 | 语义 |
|---|---|
| `operation` | TAKEOFF=0、GOTO=1、LAND=2 |
| `height_m` | TAKEOFF 相对显式解锁时的地面位置向上增加的高度 |
| `target` | GOTO 使用 `geometry_msgs/PoseStamped`；frame_id 必须为 odom，姿态仅允许单位四元数表示的水平 yaw |
| 反馈 `phase/current_pose` | 当前状态与 ENU 位姿 |
| 结果 `success/reason` | 实际完成结果及原因 |

起飞必须显式 arm。运动要求空闲的 armed Offboard 状态；目标范围 x/y ±10m、z 0.2～5m。目标生成速度不超过 1m/s，仿真步长最多按 0.1 秒生成增量。位置误差 ≤0.3m、航向误差 ≤0.15rad，连续保持至少 2 秒且生成目标已到达终点，动作才成功。60 秒仍未完成则失败。

首次 TAKEOFF 使用显式解锁时记录的地面起点，只允许当前位置仍在该起点 0.3m 内。这样允许电机运行后 landed 标志清除的近地状态。起飞资格在首个运动、降落、解除解锁或故障时清除；飞行中不能追加一次相对高度起飞。

LAND 打断运动，使用 PX4 原生降落模式；需要命令 ACK、降落模式确认及最终 landed + disarmed 才返回成功。取消 LAND 被拒绝。hold/land 打断旧动作时，旧动作返回失败原因；旧动作的迟到取消不影响新降落。并发请求由控制状态机再次检查，第二个运动请求拒绝或明确失败。

## 遥测

| 话题 | 类型 | 频率与说明 |
|---|---|---|
| `/uav001/odometry` | nav_msgs/Odometry | 约 5Hz，新鲜遥测时发布；pose 为 odom，twist 为 base_link |
| `/uav001/path` | nav_msgs/Path | 约 5Hz，最多保留 3000 个位姿 |
| `/uav001/diagnostics` | diagnostic_msgs/DiagnosticArray | 约 5Hz；状态、原因、fresh、armed、offboard、landed、preflight、position |
| `/clock` | rosgraph_msgs/Clock | 来自 Gazebo 的仿真时间 |

PX4 边界订阅采用 SensorData best effort QoS。话题名根据固定消息的 MESSAGE_VERSION 添加 `_vN`；当前 local_position/status 为 `_v1`。doctor 对八种关键消息的字段、常量和版本作静态匹配。

## 时间和故障

仿真位姿、TF、轨迹使用 `/clock`。通信新鲜度、ACK/动作超时使用单调时钟；位置和姿态 0.5 秒、状态 1 秒、落地检测 2 秒过期。固定 PX4 的落地检测在状态未变化时每秒发布一次，2 秒阈值允许正常传输抖动。心跳使用 steady timer 20Hz，暂停或回退仿真时间不会推进位置目标；时间回退触发故障。

启动就绪要求新鲜、落地、未解锁及 PX4 preflight 连续通过 5 秒；状态查询失败/超时重置该窗口。落地后旧 Offboard 状态可能因心跳停止而不健康，再次 arm 允许重新预热，但只有模式及其飞前检查通过才发送解锁命令。

仿真配置 `UXRCE_DDS_SYNCT=0`，显式将 PX4 原始微秒时间锚定到仿真时钟；不得把 Gazebo 时间直接填写成 PX4 时间。固件时间回退同样锁定故障。

ACK 拒绝、5 秒内无 ACK/状态确认、遥测失效、离开 armed Offboard、越界、坐标重置均返回失败原因。FAILSAFE 停止心跳，`COM_OF_LOSS_T=1`、`COM_OBL_RC_ACT=4` 让 PX4 执行失控降落。通信恢复不清除故障或恢复旧目标，需要重新启动实验室。策略单元测试注入这些故障；本机独立实飞验证适配节点退出场景。

CLI 输出 JSON；成功退出 0，指令失败/超时/取消退出 1，参数格式错误退出 2。`status` 在无新鲜诊断、遥测过期或 FAILSAFE 时退出非零。

Ctrl+C 中断 TAKEOFF/GOTO 时保留 ROS 上下文，有界等待目标响应并请求取消；无法确认取消时明确输出 unconfirmed。中断 LAND 不发送取消，确认已接受才提示降落继续。

## V0.3 定位与地图

`/uav001/localization/odometry`：连续lio_odom中的ENU/FLU六自由度位姿，child_frame=lio_base_link。线速度规范化为机体坐标；当前GLIM不输出角速度，角速度字段未在EV_CTRL=11配置中融合。本版IMU位于机体原点且同向，因此lio_base_link与IMU系重合；改变IMU安装位姿需另行完成机体输出适配。观测协方差为明确的EKF噪声下限，不是算法边缘协方差。

`localization/path`为连续里程计轨迹，`localization/map`为全局优化点云；后者不是三维占据/避障地图。`localization/diagnostics`的READY表示来源新鲜、位姿数值有效且连续性检查通过，不表示实时配准置信度或已知真实误差。真实误差由独立评估器离线量化。

入口只允许校验后的点云/IMU进入`localization/input/*`，独立真值不进入算法。LoadMap和Relocalize服务只接受新鲜地面未解锁状态，失败保留原地图变换。重定位仅改变map→lio_odom；一次地面坐标适配提供map→odom，已存在的控制目标不重算、不跳变。

`slam`运行配置才允许唯一飞控适配发布VehicleOdometry。首次完整外部融合就绪后，外部位置/高度/航向、GNSS未融合标志及诊断持续受检，任何失效锁定失败；恢复数据不会恢复心跳或续飞。默认flight/sensors不启用外部定位。

## V0.4 三维导航

`/uav001/navigation/plan`（PlanPath）：PoseStamped goal，frame=odom；返回success/reason、Path和map_version。仅规划，不移动。

`/uav001/navigation/navigate`（Navigate Action）：同一目标，反馈phase/current_pose/replans，结果success/reason。重复请求拒绝，最多3次障碍更新重规划；取消悬停，人工HOLD/LAND停止，LAND继续完成。未观察/占据起终点、NO_PATH、SEARCH_BUDGET_EXCEEDED和控制拒绝明确区分。

`navigation/occupied`为odom中占据体素中心PointCloud2；`navigation/path`为已验证的Path，两者transient-local。`navigation/diagnostics`含ready、source_stamp、source_age、map_version、frame及失败原因。源时间和单调通信看门狗分别检查，失败锁定。

`localization/control_alignment`是冻结lio_odom←odom TransformStamped，仅为估计后的坐标适配；不增加TF父节点、不输入SLAM。实时地图用源时间插值位姿和归档安装外参注册。

ExecuteFlight新增navigation（默认False）及navigation_epoch（默认0）。navigation配置要求GOTO具有导航标记并匹配人工打断世代；接收和执行均检查。旧profile忽略标记。所有FMU写入仍只有飞控适配器，标记不是安全认证或跨租户访问机制。
