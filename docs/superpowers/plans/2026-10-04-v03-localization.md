# V0.3 Localization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: superpowers:executing-plans. Inline execution, one fresh final branch review.

**Goal:** 完成固定数据集 LIO 基准、实时六自由度定位、回环与地图重定位，以及实际无 GNSS 飞行闭环。

**Architecture:** 私有 CPU GLIM；独立 Python 学习/评估与地图工具；ROS 定位适配；唯一飞控输入节点接外部里程计。

**Spec:** docs/superpowers/specs/2026-10-04-v03-localization-design.md

## Global Constraints

- ENU/FLU，连续 odom；真值只用于评估，算法仅点云/IMU。
- 固定提交；不修改依赖原始源码，不全局安装；CPU 两线程默认。
- 正确仿真与 PX4 时间边界，单调时钟 watchdog；默认保留 flight/sensors 行为。
- 每项实质功能以行为测试 RED→GREEN；每项验收保留实际输出。
- 用户已授权继续 V0.3；沿用独立路径和 feature branch，小步交付，未通过门禁不发布标签。

## Review Focus

真值泄漏、错坐标/时间、重复 TF、全局跳变进入控制、定位失效后自动续飞、地图错误接受、假回环证据、无 GNSS 实际融合标志。

### Task 1: Private CPU backend and sensor adaptation

**Files:** dependencies/lock.json, scripts/bootstrap_slam.py, scripts/env.sh, configs/slam/, tests/test_slam_config.py.

**Interfaces:** Produces verified private backend manifest and CPU config. Consumes fixed sensor extrinsics and only lidar/IMU topics.

- [ ] Write tests for physical IMU→lidar transform, global shutter semantics, topic allowlist and CPU backend selection; run focused tests. Expected: absent functionality failure.
- [ ] Implement config generation/private bootstrap with fixed dependencies; run tests. Expected: pass.
- [ ] Build CPU backend with existing GTSAM; execute native help/smoke. Expected: working executable, no GPU/viewer dependencies.
- [ ] Commit dependency/backend integration.

### Task 2: Fixed dataset learning and localization benchmark

**Files:** uav_lab_localization package, registration.py, evaluation.py, benchmark.py, tests/test_registration.py, tests/test_localization_metrics.py.

**Interfaces:** Consumes complete sensor dataset/config; produces isolated input bag, GLIM trajectory/dump, ICP baseline and hash-bound numeric evaluation.

- [ ] Write tests for true six-DOF registration, partial/no overlap and degenerate geometry; one fixed SE(3) alignment, strict timestamps and RPE. Expected: fail.
- [ ] Implement learning ICP and independent truth evaluator; run tests. Expected: pass.
- [ ] Run GLIM CPU on complete fixed motion dataset; measure coverage, position/attitude error and runtime. Expected: benchmark gates pass, compare and freeze algorithm/config.
- [ ] Commit benchmark plus learning experiment and algorithm selection report.

### Task 3: Continuous ROS localization and quality

**Files:** localization_node.py, quality.py, scripts/supervise.py, configs/slam.rviz, tests/test_localization_quality.py, tests/ros/test_localization_node.py.

**Interfaces:** Consumes raw LIO/IMU/cloud and clock; produces continuous normalized odometry, diagnostics and path; no truth subscriber and no fmu publisher.

- [ ] Write invalid/reset/stale source tests and real ROS normalize/quality tests. Expected: fail.
- [ ] Implement normalization and latched failure watchdog; add localization profile/readiness. Expected: focused tests pass.
- [ ] Run actual sensors and live localization; compare independent truth. Expected: quality stable and no competing control TF.
- [ ] Commit runtime integration.

### Task 4: Loop closure, map archive and relocalization

**Files:** maps.py, map_cli.py, tests/test_maps.py, integration acceptance scripts/docs.

**Interfaces:** Consumes native optimized submaps and observed scan; produces validated map archive and map→odom registration, preserving odom.

- [ ] Write archive corruption/calibration and actual scan matching acceptance/rejection tests. Expected: fail.
- [ ] Implement native map export/archive/load and coarse-prior relocalization with geometric quality gates. Expected: focused tests pass.
- [ ] Demonstrate accepted non-adjacent loop factors; save/load/relocalize real scans and preserve continuous targets. Expected: numeric evidence passes.
- [ ] Commit SLAM/map integration.

### Task 5: PX4 external odometry and no GNSS flight

**Files:** bridge conversion/time/quality integration, no GNSS profile, tests/test_external_odometry.py, ROS integration tests.

**Interfaces:** Consumes validated continuous localization; sole bridge produces version-correct VehicleOdometry. PX4 status proves EV fusion and no GPS fusion.

- [ ] Write full orientation/vector/covariance/time conversion and loss/reset gating tests. Expected: fail.
- [ ] Implement opt-in external input and fixed-version parameters; run tests. Expected: pass.
- [ ] Native no GNSS three-flight acceptance and localization exit fault. Expected: three passes, confirmed fusion, land/disarm and no auto resume.
- [ ] Commit no GNSS closed loop.

### Task 6: Regression, documentation and release

**Files:** README, docs/experiments, docs/validation, package versions, CI.

**Interfaces:** Consumes verified earlier outputs; produces reproducible commands, separate V0.3.0/0.3.1 report, reviewable incremental PR and gated tags.

- [ ] Build platform; full regression, learning/benchmark and headless CI acceptance. Expected: all required gates pass.
- [ ] One fresh-context final review; material fixes RED→GREEN, full regression. Expected: no unresolved material defects.
- [ ] Commit evidence/docs; push feature branch, create/attach incremental PR; tag only achieved versions. Expected: exact commit validation links and candid limitations.
