# L1 workbench implementation plan

Spec: `docs/superpowers/specs/2026-10-08-l1-workbench-design.md` (user approved).

## Global Constraints

- Protected baseline v0.4.0/e295a54; execute inline on feat/l1-workbench in existing native checkout to reuse private dependencies. No changes to main or existing tags.
- Only bridge publishes PX4 commands. No truth input to algorithms. Strict dependency and stage evidence, no simulated qualification. System Python 3.10, Humble/Harmonic, bounded CPU/memory, one backend per role.
- Behavioral tests RED→GREEN; full existing Python/ROS suite at each task closure. Native experiments separately capture manifests/logs. Never weaken frequency, collision, timing, or failure gates to get green.
- A task is complete only with its explicit runtime gate. Document build/runtime failures and resolve them rather than marking backend ready.

### Task 1: Backend registry and experiment contracts (A)

Interfaces: produces backend capabilities, sensor groups, stage evidence, safe ground selection consumed by tasks 3, 5–10.

1. Write tests for unsupported sensor groups, absent/tampered evidence, selection with armed/stale telemetry, source-version mismatch; run RED.
2. Implement registry and CLI list/capabilities/select with atomic run configuration; status cannot promote itself merely on startup.
3. Run GREEN and whole suite; commit.

Expected: selection fails closed while armed or telemetry stale; stages depend on traceable reports.
Verify: `./scripts/env.sh env PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest -q`.

### Task 2: Continuous trajectory core (B)

Interfaces: consumes route controls; produces C2 polynomial trajectories with analytic p/v/a/jerk, bounded derivative maxima, collision samples, stop/replan state; tasks 3/8 consume.

1. RED: fly-through corner, endpoint rest, initial p/v/a, analytic derivative extrema, invalid/nonfinite inputs and collision between controls.
2. Implement quintic minimum-jerk interpolation/time scaling, explicit dwell, certified derivative limits and adaptive collision validation; preserve legacy MotionProfile.
3. GREEN/full suite; commit.

Expected: no intermediate rest, C2 joins, default limits and independent full-curve collision checking.
Verify: whole Python/ROS suite.

### Task 3: PX4 trajectory action and continuous navigation (B)

Interfaces: consumes task 2 trajectories and existing bridge gate/epoch; produces 50Hz sole-publisher execution, p/v/a feedforward, 20Hz heartbeat, Action result/feedback; planner and Web consume.

1. RED ROS acceptance tests ownership, cancel/HOLD/LAND, final settle, stale source, replan transition, invalid trajectory.
2. Add messages/action and executor, CLI route, integrate A* full curve collision and replan; old ExecuteFlight remains.
3. Build all ROS packages, full suite and native curved route; commit.

Expected: smooth reference and actual flight, controls remain fail closed, old flight gate passes.
Verify: whole suite plus native trajectory acceptance report.

### Task 4: Moving scan sensors and six reproducible scenarios (A)

Interfaces: produces authentic timed scans, calibrated camera/IMU/RTK contracts and route/scene assets for tasks 5–9; preserves old profiles.

1. RED: moving target sampled at distinct beam times, point fields/ring contracts, GNSS disruption events, route reproducibility and calibration audit.
2. Implement measurement-time scan acquisition, Livox approximation, mechanical scanner, noisy attitude sensor, textured scenes, six routes with 20–50 controls; record approximation limits.
3. Sensor frequency/time/calibration native audits and recording replay, full suite; commit.

Expected: real intra-scan motion, no fabricated ring/time/truth IMU; old profiles regression passes.
Verify: whole suite plus native sensor/dataset audits.

### Task 5: Independent ROS observatory and Web point clouds (B)

Interfaces: consumes standard observed topics and registry, produces bounded binary cloud frames/telemetry/Web; task 10 adds comparison/replay control.

1. RED bridge tests latest-frame replacement, point budget, malformed clouds, disconnection, no truth leakage; frontend tests actual selection/layer interactions.
2. Build React/TS/Three.js dark 3D UI, raw/registered/global/voxel layers, planned/actual paths, camera tools; isolated loopback bridge and supervisor lifecycle.
3. Build/test UI, inspect real browser receiving native ROS clouds, disconnect fault, full suite; commit.

Expected: 5Hz/100k bounded latest frames, labeled display decimation, original recording unchanged; flight independent.
Verify: Python/ROS suite, npm tests/build, native Web acceptance.

### Task 6: FAST-LIO2, FAST-LIVO2, RTK adapters and replay (C)

Interfaces: consumes task 1/4 inputs, produces normalized odometry/status/global correction and reports; isolated pinned upstream builds.

1. RED contract and quality failures; select/pin real sources and implement ROS2 interface adapters without changing algorithm math.
2. Install/build three methods, run official examples and common compatible recorded sensor group; RTK post-optimization distinct from real-time odometry.
3. Export unified quality/resource reports, no-GNSS real-time test, full suite; commit.

Expected: all three actually output poses; fixed source/data/config; stage status proves only measured capability.
Verify: whole suite and per-method native replay reports.

### Task 7: ORB-SLAM3, LIO-SAM, VINS-Fusion and GLIM comparison (C)

Interfaces: consumes calibrated mono-inertial/mechanical groups and task 1 report contract; outputs group-scoped metrics/capability reports.

1. RED input/scale/frame/quality contracts. Build isolated pinned cores and ROS2 adapters.
2. Run official/public examples and compatible platform datasets; GLIM baseline rerun; verify loops/relocalization separately.
3. Resource and accuracy comparisons, realtime eligibility tests, full suite; commit.

Expected: seven total methods really run, different sensor inputs not ranked together; quality failures explicit.
Verify: whole suite plus seven-method replay matrix.

### Task 8: EGO, FAST-Planner, GCOPTER pipelines (D)

Interfaces: consumes sensor-derived fixed/rolling maps and task 2 trajectory contract; outputs source-identified collision-checked time trajectories.

1. RED identical-map benchmark contracts, unknown/clearance/replan and derivative failures.
2. Build/pin official EGO ROS2 branch single drone, port FAST-Planner ROS interface, corridor+GCOPTER wrapper; connect real map/odometry, no fake-drone physics.
3. Same-map offline four-way benchmark and actual adapter trajectory tests, full suite; commit.

Expected: all four use real algorithm cores and same admissible observed geometry, independent curve collision check.
Verify: whole suite plus four-planner native reports.

### Task 9: Complex closed-loop and fault acceptance (D/E)

Interfaces: consumes eligible backend pairs, sensors, trajectories and supervisor; produces repeated run manifest/collision/tracking/fault evidence.

1. RED long-route bounded local map and source/RTK failures; implement local-map resource policy and experiment runner.
2. Fixed-route localization, fixed-map planning, candidate combinations three PX4 repeats; independent clearance/P95/limits checks.
3. HOLD/LAND, source exit/stale, RTK abnormal, Web disconnect, restart/no-auto-resume; retain failures and select normal-scene-qualified default.

Expected: all success/failure reports traceable, P95 goal ≤0.3m, no midpoint stops/replan jumps; full old gates intact.
Verify: whole suite plus native closed-loop/fault matrix.

### Task 10: Comparison UI, CI and hardware preparation (E)

Interfaces: consumes immutable reports, stage evidence and owned experiment lifecycle; produces complete Web controls/replay/comparison, CI and deployment entry.

1. RED report grouping, safe controls, replay isolation and configuration selection tests; implement views/CLI export.
2. Add headless checks/builds/algorithm fixture CI, hardware configuration template and exact deployment/limitations docs.
3. Full suites/native regressions/clean CI; final fresh-context whole-branch review and one tested fix pass; preserve evidence and PR.

Expected: reviewable final artifact with actual algorithm stage matrix, all gates and limitations, no unsupported claim of real-hardware flight.
Verify: whole Python/ROS suite, frontend tests/build, CI, native acceptance matrix.

## Review Focus

Algorithm core authenticity and dependency provenance; temporal/frame conversions without truth contamination; armed/stale selection and ownership races; trajectory C2 and analytic limits including replan/stop; full curve conservative unknown collision; slow consumers bounded and isolated; posterior RTK correction control continuity; resource bounds on long routes; metrics fair by input group and immutable evidence; genuine PX4 simulation rather than upstream fake drone; failure gates not weakened; original 187 regression and flight acceptance retained.
