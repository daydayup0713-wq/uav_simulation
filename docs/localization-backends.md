# FAST 系列定位实验

本页描述 L1 升级的第三方定位接入。实时里程计、RTK 回放后的全局优化、PX4 闭环飞行分别验收；后端能启动不代表通过定位或飞行门禁。

## 固定来源

所有源版本、移植补丁、补丁摘要和源树摘要见 `dependencies/backends.lock.json`。依赖安装到 `.deps/backends/<backend>/install`，系统 ROS 为 Humble，Python 为系统 3.10。每个进程只加载自己的依赖前缀。

| 后端 | 算法来源 | ROS 2 接口 | 当前能力边界 |
|---|---|---|---|
| FAST-LIO2 | hku-mars/FAST_LIO 的 ROS2 分支 | 官方 ROS2 接口，补充传感器 QoS | 雷达惯性里程计；不宣称回环、重定位 |
| FAST-LIVO2 | hku-mars/FAST-LIVO2 | Intel robotics-ai-suite 的 ROS2 移植补丁，加平台输入/时间接口 | 雷达、IMU、单目视觉融合里程计；不宣称完整 SLAM |
| FAST-LIVO2-RTK | xuankuzcr/FAST-LIVO2-RTK 的 sb-im 社区 ROS2 移植 | 社区 ROS2 接口，加私有依赖和平台时间转换 | 实时 LIVO 与序列结束后的 RTK 批处理优化分别记录 |

算法核心求解器保持上游实现。接口补丁处理私有安装、ROS 消息/QoS、源时间和输出命名空间。RTK 地图导出补丁跳过合法的空彩色点云，保留相应位姿关键帧，避免 PCL 1.12 空点云变换除零。

```bash
./scripts/env.sh /usr/bin/python3 scripts/bootstrap_backends.py --backend fast_lio2 --jobs 1
./scripts/env.sh /usr/bin/python3 scripts/bootstrap_backends.py --backend fast_livo2 --jobs 1
./scripts/env.sh /usr/bin/python3 scripts/bootstrap_backends.py --backend fast_livo2_rtk --jobs 1
```

项目不将第三方源码、二进制、录制数据、地图和大型日志加入普通 Git 历史。构建前核对源树和补丁，运行前核对二进制摘要；每次实验另存配置、锁定文件、平台代码快照、轨迹和报告。

## 数据与坐标契约

算法只接收雷达、IMU、相机及适用的 GNSS 输入。Gazebo 物理位姿仅供传感器模拟器与独立评估器使用。适配器不读取真值，默认不发布公共控制里程计；所有算法输出位于 `/uav001/backends/<backend>`。

Livox 近似点云通过真实逐点时间转换为 `CustomMsg`，保留扫描起始源时间、有效回波和采样偏移。机械雷达保持实际 ring 与秒单位逐点时间。时效检查使用最后一个有效回波的测量时刻，原始点云时间戳不变；超时仍按现有严格门槛判定。

上游输出的 IMU 位姿按照声明安装外参变为机体 FLU 位姿。连续局部里程计与全局修正分开。速度来自连续估计位姿差分；发布的协方差是明确配置的融合噪声下限，不能作为精度证明。源时间、跳变、相机过期或通信过期失败后状态锁存，通信恢复不会自行恢复飞行。

RTK 使用本机仿真数据。GPS week/TOW 与仿真源时间通过声明的固定历元一一对应；两个上游回调均使用相同逆变换。消息速度由连续已接纳 GNSS 位置差分产生，并标注推导方式与不确定度。失锁、非固定解及不合理跳变不进入批处理因子；本规则无法识别缓慢且一致的 GNSS 偏差。

## 固定回放

ROS 回放使用独立域，例如 77。输入话题白名单不包含真值或飞控话题。评估器单独读取真值，记录 ATE、RPE、姿态、覆盖率、源时间延迟和采样 CPU/RSS。不同传感器输入组合分组比较；不把 FAST-LIO2 的雷达惯性输入和视觉/RTK 融合结果放入单一排行榜。

```bash
./scripts/backend_env.sh fast_livo2 env ROS_DOMAIN_ID=77 /usr/bin/python3 scripts/replay_backend.py \
  --backend fast_livo2 --dataset recordings/<运行数据目录> \
  --output .runtime/backend-bench/<唯一实验编号>
```

RTK 回放后才发送一次批处理触发。必须看到上游完成标记、有效优化轨迹和独立质量验收，才判定 RTK 全流程成功。优化轨迹的天线参考点按标定单独评估，不用全局优化输出替换连续控制位姿。

```bash
./scripts/env.sh /usr/bin/python3 scripts/export_rtk_posterior.py .runtime/backend-bench/<RTK实验编号>
```

导出 `posterior/global_body.tum` 和 `posterior/manifest.json`，包含标定杆臂转换后的机体轨迹、最新时刻的独立 `rtk_map → fast_livo2_rtk_odom` 修正、优化点云路径及文件摘要。该刚性修正不能替代对整张旧地图的非刚性优化，也不会改写连续控制轨迹。

已下载的 NTU VIRAL `eee_01` 可生成固定 120 秒片段，用于 FAST-LIO2/FAST-LIVO2 公共数据验证。传感器保留原始源时间；Ouster 的扫描结束时间约定及上游标定进入配置和数据清单。

```bash
./scripts/env.sh .deps/data-tools/bin/python scripts/prepare_ntu.py \
  --source .deps/datasets/ntu-eee01.bag --output .deps/datasets/ntu-eee01-new --duration 120
./scripts/backend_env.sh fast_livo2 env ROS_DOMAIN_ID=77 /usr/bin/python3 scripts/replay_backend.py \
  --backend fast_livo2 --dataset .deps/datasets/ntu-eee01-new \
  --public-config .deps/datasets/ntu-eee01-new/configuration --position-only \
  --output .runtime/backend-bench/<唯一实验编号>
```

FAST-LIO2 使用该片段的 `configuration/fast_lio2`。Leica 只有棱镜位置参考，不报告姿态、旋转 RPE 或依赖参考姿态的机体平移 RPE；棱镜到 IMU 的未补偿杆臂作为误差限制保存。该片段不代表完整序列验收。

## 实时验证与阶段

```bash
./scripts/backend_env.sh fast_livo2 /usr/bin/python3 scripts/live_backend.py \
  --backend fast_livo2 --output .runtime/backend-bench/<唯一实时实验编号>
```

该命令拥有仿真进程并在数据有效后执行八字航线，检查实际算法订阅中没有 GNSS/真值。PX4 在此定位实验中仍使用原有控制方案，不能据此宣称新定位后端已经通过无 GNSS 闭环飞行。仿真退出、定位失效或指令失败会保存失败报告并清理本次进程。

后端阶段依次为 `configured → installed → replay_passed → realtime_passed → closed_loop_qualified`。质量失败报告保留，且不提升阶段。默认控制组合只能由通过正常场景及故障验收的后端产生。

## 已测结果（2026-10-09）

实际报告索引见 `docs/validation/l1/fast-backends-qualified.json`。下面按实际输入列出，不形成混合输入排行榜。

| 方法与输入 | 实验 | ATE RMSE | 结果与当前阶段 |
|---|---|---:|---|
| FAST-LIO2，雷达＋IMU | 本机完整八字录制，同 RTK 场景输入的传感器子集 | 0.959m | 质量失败；installed，禁止据此用于闭环 |
| FAST-LIVO2，雷达＋IMU＋相机 | 无 GNSS 传感器录制，固定 200 秒回放 | 0.0129m | 回放通过 |
| FAST-LIVO2，雷达＋IMU＋相机 | 本机实时 48 控制点八字航线 | 0.0239m | realtime_passed；当前飞行仍由原有 PX4 方案控制 |
| FAST-LIVO2-RTK，雷达＋IMU＋相机＋RTK | 本机完整录制，约 198 秒前端及后优化 | 前端 0.00790m | replay_passed；后优化和地图导出单独通过 |

公开 NTU VIRAL 120 秒片段的历史验证：FAST-LIO2 位置 ATE 0.190m、FAST-LIVO2 0.0374m。该数据只有棱镜位置参考，未验证姿态误差与完整序列；本机仿真仍是当前 RTK 验证的数据来源。

当前三个后端均未取得 `closed_loop_qualified`。FAST-LIO2 的复杂仿真精度失败、早期 RTK 消息字段错误和空点云导出崩溃均有保留记录。当前版本修复的是接口问题，不以修改上游求解器或放宽定位门槛消除失败结果。
