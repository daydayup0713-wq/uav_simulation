# 第一组学习实验

所有实验先等待 `LAB READY`。可在 Gazebo 查看飞行器，在 RViz 查看 odom 中的轨迹，在运行目录查看 events.jsonl 和 PX4 日志。

| 实验 | 输入什么 | 观察什么 | 预期变化 | 失败定位 |
|---|---|---|---|---|
| 解锁与模式 | `labctl arm` | WARMUP → ENTERING_OFFBOARD → ARMING → HOLDING，两个命令各自的 ACK | 有模式和解锁状态双确认 | 查 bridge.log 和 px4.log；预检失败不能只看 ACK |
| 起飞与定点 | `takeoff --height 2` | z 向上增加，MOVING → HOLDING | 达到相对起点 2m；连续 2 秒在容差内 | 检查 ENU、遥测有效性和时钟 |
| 坐标方向 | `goto --x 3 --y 0 --z 2`，再改为 y=3 | RViz 东、北方向轨迹 | ENU x 东、y 北；yaw 0° 朝东 | 查看目标 frame_id 与 ENU/NED 转换 |
| 中断与悬停 | 航点运行时在另一终端 `hold` | 原动作失败，当前位姿变为悬停目标 | 停止继续走向旧航点 | 检查旧动作结果和 HOLDING 状态 |
| 取消 | 航点 CLI 中按 Ctrl+C | CLI 非零退出，适配器保持悬停 | 起飞/航点取消不继续运动 | 查看 cancel 结果；LAND 无法取消 |
| 空中拒绝解锁解除 | 已起飞时 `disarm` | 服务失败原因 | 保持飞行，不普通断电 | 若退出码为 0，应作为回归缺陷 |
| 重复运动 | 运行航点时再发航点 | 第二次请求失败 | 原动作保持执行 | 对照 active operation 与动作结果 |
| 飞控失控 | 起飞后执行 `scripts/verify_fault.py` | 适配器退出，独立 observer 订阅飞控状态 | PX4 离开 Offboard，降落并解除解锁 | 看 fault-acceptance.json 和 px4.log；不要同时关闭 Gazebo |
| 实例冲突 | 已运行时再次 start_lab | 非零退出及 runtime lock 提示 | 原实例保持运行 | 不应出现第二个 Agent/物理世界 |

命令统一通过 `./scripts/labctl` 执行。可用 `./scripts/env.sh ros2 topic list` 查看话题，`./scripts/env.sh ros2 interface show uav_lab_interfaces/action/ExecuteFlight` 查看接口。直接观察 PX4 时使用 best effort QoS，并依据消息版本选择话题。

故障后先确认 landed 与 disarmed，再停止监督进程。重新启动才开始下一组实验。V0.1 中的 odometry 来自 PX4 自身估计，不是 SLAM；后续会增加独立真值和误差曲线，避免把估计输出当成真值。
