# L1 实验传感器与复杂场景

旧 `flight/sensors/localization/slam/navigation` 配置保留。三个新配置只在传感器实验模式启用；定位后端取得相应输入契约和运行证据后再进入无 GNSS 闭环。

```bash
./scripts/start_lab.sh --profile sensors --sensor-profile mechanical --scene circle-eight --headless
./scripts/labctl sensors --duration 5
./scripts/labctl record --duration 35
./scripts/labctl replay recordings/<dataset> --domain 77
```

`--sensor-profile` 可选 `livox`、`livox-rtk`、`mechanical`。相机配置为 640×480、30Hz，IMU 200Hz；配置频率和实际接收频率分别保存，不能把名义频率当作完整数据覆盖率。

## 逐点测量

扫描器使用私有 `/_lab/sensor_physics/odometry` 的 200Hz 物理位姿，在每束激光的采样时刻插值位姿并进行场景射线求交。扫描完成后输出原始测量；XYZ 保留对应束时刻的雷达坐标，尚未去畸变。帧头表示扫描起点，FLOAT32 `time` 表示相对起点的秒数，范围约 0～0.1s。机械扫描额外包含 UINT16 `ring`（16 个物理通道）；Livox 近似包含 UINT16 `line`（4 个声明的光学通道），不会伪造机械 ring。

场景几何和物理位姿只供传感器生成及独立评估使用，不能接入定位或规划。公共真值独立降频至 25Hz。普通六轴 IMU 不提供姿态；机械雷达配置的虚拟 AHRS 提供叠加 0.005rad 旋转噪声的姿态及非零协方差。其陀螺仪和加速度来自带噪 Gazebo IMU。

近似范围：场景只有静态箱体；位姿在物理采样之间采用平移线性插值和四元数 SLERP；未模拟机身自遮挡、透明表面、多路径和真实材质反射。Livox 非重复覆盖和四通道模式是学习模型，不代表 Mid-360 的专有光学扫描。强度采用距离衰减的合成值。相机为理想全局快门，不包含真实镜头、运动模糊或曝光延迟。纹理固定种子生成；长走廊刻意重复纹理，其余对象分别生成。

新配置使用项目内 64MiB 分段共享内存与仅 loopback 的 UDP 发现，避免大图像在默认传输中丢片；设置只影响项目进程，不修改系统网络。配置按 [Fast DDS 2.6 传输契约](https://fast-dds.docs.eprosima.com/en/2.6.x/fastdds/xml_configuration/transports.html) 归档。录制保存完整接收数据，显示降采样不影响录制。

## RTK 与场景

GNSS 输出带源时间、状态和协方差的 `NavSatFix`，质量状态单独发布并录制。ENU 测量按局部 WGS84 曲率转换；声明适用范围≤1km。默认固定解噪声标准差 0.02m，float 为 0.5m。室外场景按运行仿真时间在 50～80s 失锁、80～90s float、100～102s 注入 30m 东向异常，之后恢复。失锁坐标为 NaN，不能当作有效观测；异常数据不自动删去，留给后端鲁棒性实验处理。

| 场景 | 控制点 | 目的 |
|---|---:|---|
| circle-eight | 48 | 圆形转弯、八字、自交回访 |
| helix | 33 | 多高度螺旋、连续升降 |
| multi-room | 40 | 门洞、多房间闭环 |
| corridor | 40 | 60m 重复结构往返 |
| dense | 36 | 密集柱体与狭窄通道 |
| outdoor-rtk | 42 | 120m 长路线与 RTK 退化 |

生成世界、纹理、射线几何、语义路线及标定存入每次运行的 `configuration/`，同时保留哈希。控制点数量与 50Hz 飞行参考采样数分别记录。长路线暂不使用旧 ±10m 飞行边界；后续复杂导航验收将从场景配置显式加载边界和局部地图，不能绕过当前飞控门禁直接发送超界目标。

## 观察和定位失败

输入：固定场景、传感器配置、显式解锁后的运动。观察：频率、帧时间、逐点时间范围、ring/line、相机内容、IMU 姿态协方差、GNSS 质量及完整回放。

预期：每束激光测量与对应时刻的运动一致；按束时刻变换后的静态表面残差小于把整帧当作扫描起点的残差。独立脚本 `scripts/verify_timed_dataset.py` 使用录制真值与几何检查这一点，不向算法提供真值。

失败定位：先查 `sensor-readiness.json`、`timed-sensors.jsonl` 和各进程日志；检查源时间和物理位姿间隔，再查缺帧、渲染内容与 DDS。不要通过补假时间、注入理想姿态或放宽过期门禁让实验通过。显式 `replay --rate 0.5` 可以减轻数据检查的资源负担，报告标注回放速率，此证据不授予实时运行资格。
