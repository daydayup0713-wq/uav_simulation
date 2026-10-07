# 实验04：观察空间内的三维导航

输入是导航场景的通用雷达、IMU及CPU LIO。连续odom用于控制，冻结LIO适配用于点云注册，真值只供独立评估。导航专用雷达64×180、约179°垂直视场，用于可观测性学习；不是Mid-360扫描模型。

第一终端启动并等待 `LAB READY`：

```bash
cd /home/pine/workspace/ai/UAV/uav_simulation
./scripts/start_lab.sh --profile navigation --rviz
```

第二终端运行：

```bash
./scripts/labctl nav status
./scripts/labctl nav demo --runs 3
```

演示显式解锁、相对地面起飞2m、拒绝挡板内部目标、自动到达(2.5,3.6,2)、返回(0,0,2)、降落并确认落地解除武装。到点需0.15m内连续2秒；参考轨迹速度0.5m/s、加速度0.5m/s²。实际机体响应另在真值评估中记录。

手动实验先用普通 `arm` 和 `takeoff --height 2`。导航通过 `nav plan X Y Z` 只检查路径，通过 `nav goto X Y Z` 执行路径；坐标为odom米。普通 `hold` 和 `land` 可打断导航；普通 `goto` 在navigation场景被拒绝。重复导航请求拒绝，取消后保持悬停；人工打断的世代编号使已排队的旧航段也无法续飞。

在RViz观察占据体素、计划路径和实际轨迹。机体包络(.4,.4,.3)m加.25m裕度，按.2m网格向外取整；未知空间也阻止通行。目标在障碍或未观测区，返回明确原因；搜索预算耗尽单独返回。V0.4仅验证静态、已观测空间，尚无自主探索或动态障碍预测。

每次运行保存配置、依赖、状态/动作、源时间、地图版本、实际ULog、注册扫描及体素快照于.runtime运行目录。每条计划另保存不可覆盖的navigation-plans/会话/map-v版本.npz，navigation.jsonl记录文件及SHA256，支持核验当时的碰撞地图。若导航或定位过期，控制链失效锁定并停止Offboard，PX4按配置降落；恢复数据不会续飞，需重启实验。查看source_failure、navigation_plan、replan和navigation_result定位失败。当前避障覆盖空中导航段，起降沿用已有飞控逻辑。

CPU固定观察基准可独立执行：

```bash
./scripts/labctl nav fixture .runtime/my-observations
./scripts/labctl nav benchmark .runtime/my-observations .runtime/my-benchmark
```

解析扫描只验证安装/回放与算法契约，不代表渲染精度。独立真值测量脚本 `scripts/verify_navigation.py` 不属于算法节点，场景几何和真值不会输入地图或规划器。
