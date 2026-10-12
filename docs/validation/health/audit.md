# Historical health audit and implementation log

Current results: [final report](report.md), [machine-readable acceptance](acceptance.json). Pending statements below describe their historical point in the work; they are superseded by the final report. Failed raw evidence is preserved.

# Health audit baseline

Source: 908b3655b957362b6e3e6cf3ca9985d41f5029cb (plus worktree ignore setup).

Tracked repository: 536 files / 4,311,092 bytes. Runtime 25 GiB; private dependencies 23 GiB (datasets 15 GiB); recordings 35 GiB. These categories are not unused source code.

Entrypoints checked: scripts/labctl dispatch; ROS setup.py console_scripts; lab.launch.py generated entrypoint; supervisor processes; ROS services/actions; native CMake; GitHub CI; React imports.

Static candidates are not deletion authority: localization evaluation/frames/quality and slam_config are compatibility exports; ROS subscription references keep objects alive; HTTP do_POST/log_message and launch generate_launch_description are framework callbacks. Native licenses, testing fixtures, alternative CLI/launch paths stay.

Confirmed candidates: frontend unused backends state, ordinary unused imports, unused result variables. Integration/evaluation/planner outputs require regression checks before consolidation.

UI findings: API/observation health conflated; FAILSAFE lacks immediate cause/action; static 8080 presents inactive control options; comparison API failures are swallowed and remain loading forever; request timeouts absent; unchanged paths rebuilt on every telemetry update; narrow layout hides navigation.

Baseline frontend: 13 passed; production build passed; existing act warnings and 808.51 kB JS chunk warning. Native baseline requires the new worktree's generated ROS interfaces and is being built; bare Python first attempt failed collection on missing uav_lab_interfaces (setup issue).

## Implementation dispositions

| Area | Evidence / decision | Verification |
|---|---|---|
| Python imports and result variables | Ordinary unused symbols removed; compatibility modules explicitly export their names. No algorithm/plugin package deleted. | pyflakes on tracked and new platform sources; ROS/native regression |
| Voxel ray integration | Static and timed scans delegate to one existing bounded integrator. Empty static scan retains its old version-increment behavior; empty timed returns still reject. | identical fixed-input occupied/free/observed masks; freshness/unknown-space tests |
| Native subprocess environment | Canonical and alias legacy underlays excluded; each planner attempt owns ROS log directory. Private binary confinement unchanged. | actual three native core builds and four same-map replay pipelines |
| Frontend state | Removed unused backend state, separated connection/deadline/report hooks, moved tools/types/tests to dev dependencies. | TS unused-symbol gates and Web tests/build |
| Three.js | Reuses point/path buffers, bounds capacity, disposes registered buffers before growth, hides missing layers/paths and releases resources on unmount. Rendering capped at 30Hz, background tabs skip frames. | buffer identity/draw-range/disposal tests; actual browser observation |
| Web feedback | Real supervisor stages and operation blockers; independent API/observation/freshness indicators; visible FAILSAFE cause; read-only 8080; report/metric/recording/error/timeout states. | API contract and frontend tests, responsive screenshots |
| Retention | 7 days + recent three successes/failures per group. Unknown outcomes, active runs, recursive evidence, raw recordings and dependency state retained. | reference/symlink/change/process/partial-failure/audit tests; primary-root immutable dry plan |

Native baseline in original checkout: 506 passed / 1 skipped. Worktree full regression before the heartbeat scheduling follow-up: 531 passed / 3 skipped. A parallel-live full regression had one Web goto rejection; standalone and idle full rechecks passed, failed evidence is retained. Setup/DDS sandbox failures are retained in `.runtime/health-baseline/` and are not reported as passing checks.

The changed backend adapter and planner source identities have fresh native builds/replay evidence. Old flight qualification does not authorize this branch. Long ground runs and current-source flight/fault gates are required before publishing a repaired/stable claim.

The first valid 30-minute no-Web session passed: run `20261012T060636Z-890db7ad`, wall-clock duration 1800.066s, no flight/localization/navigation failures. The first sampler attempt failed on Humble diagnostic byte levels and is explicitly excluded.

Primary storage inventory (regular files, no symlink following): dependencies 23,967,915,224 bytes; runtime 26,528,119,560 bytes; recordings 37,103,937,547 bytes. This is retained experimental data, not evidence of unused source. Default retention currently finds no eligible deletion; historical ambiguous outcomes and retained evidence are not sacrificed for a target GB count.

Historical acceptance documents apply to the implementations stated in each report; use this health report for the current branch's gates. Algorithm limits remain in [L1 acceptance](../l1/acceptance.md).

## Flight-discovered scheduling issue (acceptance still pending)

The first current-branch three-flight run completed two routes and then entered FAILSAFE: navigation map/heartbeat expired. Tracking P95 was 0.0435m, but the full repetition gate failed. Navigation itself did not latch a source failure; a committed map was logged 58ms before the bridge failure, while external pose maximum receipt gap remained below 0.178s. These facts point to health publication/ingress scheduling and do not justify relaxing expiry thresholds.

An actual ROS test with 300ms default-group input work reproduced delayed health publication (4 samples in 1.3s). Health now has its own callback group, ordered publication, immediate committed-map publication and serialized latest-only bridge ingress. Existing expiry and permanent recovery-latch checks remain. The scheduling/map/freshness subset passed 8 tests. Final-source repeated flight, fault and long-ground acceptance must be rerun; the earlier three ground passes are retained as pre-fix evidence.

Browser captures used 1920×1080, 1366×768 and narrow viewport overrides. The in-app browser applies a display scale: measured CSS widths were 1780, 1262 and 350px, respectively, with no horizontal document overflow. JPEG captures are stored in `screenshots/`. Exact unscaled browser resolution checks remain a limitation, not a claimed pixel-perfect gate.

Final health-scheduling code regression: 533 passed / 3 skipped; frontend 23 passed and production build passed. Post-fix repeated flight is running. Parent-directory replacement protection uses directory file descriptors and safe `rmtree`; the partial-failure test now removes a real leaf before raising and confirms subsequent candidates still proceed. Recording mounts are canonicalized to the explicitly owned data root and symlink escapes beneath that root still reject.

## Review follow-up and current acceptance

One independent full-range review found four gaps: retained runtime report/directory references did not protect their raw runs; interrupted deletion lacked a durable audit; obsolete replay reads were no longer cancelled; leaving an unfinished report kept its spinner. Regressions reproduced each behavior before the fixes. Retained report containers and policy-retained runs now close their references recursively. Apply persists a deletion intent before mutation and per-item results afterward, retains interruption state, and stops on audit write failure. Playback waits for its current frame and coalesces scrubbing; callers can cancel read requests alongside their deadline. Inactive report queries cancel and clear loading. Native maintenance/ground/contract subset: 32 passed; frontend: 26 passed/build passed. Replay loading is separated from the main component and file failures have a read-only retry. Final full regression remains required.

Post-scheduling normal flight run `20261012T092225Z-4ca2efcc` passed three repetitions, all 141 controls, P95 tracking 0.040255m, full-body minimum clearance 0.702m, and actual landed/disarmed confirmation each time. Five actual fault cases have passing reports: `health-faults-final` contains HOLD/LAND/localization-exit/Web-disconnect, and `health-faults-source-retry` contains the source-stale case with resume and rejected rearm.

The first source-stale attempt was rejected before arming: bridge source age briefly exceeded 1s during startup and its gate latched, while navigation later returned READY. No fault was injected, so that attempt is excluded and retained. This is still a known startup timing limitation, not proof that every timeout was eliminated. Ground sampling now rejects explicit `navigation_ready=False` or `external_ready=False` even when phase reads READY. Web previews the same reported rejection rather than offering an arm button that the bridge will reject; safe ground shutdown remains permitted. Thresholds and bridge latch behavior are unchanged. Final-controller three 1800-second ground sessions are underway.


The first final-controller no-Web session passed 1800.0176s. The following observer session exposed a READY-phase/failed-navigation-gate mismatch and was stopped at168s; it failed, and neither phase labels nor later restored output clear the latch. Exact first-rejection logging measured healthy=True with receipt age0.6728s and map source age1.132s, despite continuing ~50–65ms map updates. Ten short starts with passive diagnostic subscriptions reproduced one failure, whereas five ordinary short starts passed. This narrows the cause to diagnostic transport/reception during subscriber discovery. The bridge now requests latest-only best-effort navigation health while the existing reliable publisher remains compatible with CLI/observation subscribers. No age threshold or permanent latch was changed. The real ROS compatibility and unchanged expiry subset passed8 tests; ten identical starts and all final-source long/flight/fault gates remain pending. Previous pass reports are historical snapshots, not qualification for this changed receiver.

Reception follow-up verification: identical ten-start conditions changed from9/10 passes with the reliable bridge reader to10/10 with the latest-only best-effort reader. See `startup-reception.json` for immutable run IDs, rejected-state fields and artifact hashes. This is evidence for the observed startup symptom, not a guarantee against all load/transport failures. Final-source native regression:544 passed/3 skipped; frontend:27 passed and production build passed. Long-ground/three-flight/five-fault qualification for this receiver is still pending.

Final-receiver three-flight run `20261012T105421Z-e65c4aac` passed all141 controls, P95 tracking0.040045m and body clearance0.700714m. The fault matrix has four valid passes; source-stale setup failed before injection due a localization input interruption (GLIM logged IMU source gaps188ms/128ms), and remains preserved and excluded. Separate valid source-stale run `20261012T111613Z-c84df9e9` passed pause/landing/recovery-latch checks; an independent raw-IMU/normalized-IMU/points/pose probe found no input gaps before the intentional pause. This does not identify the earlier sensor gap's cause or justify relaxing freshness gates. Three final-source ground sessions have started; pair promotion remains withheld until they pass.

Responsive DOM checks now reached actual CSS1920×1080,1366×768 and390×844, with no document horizontal overflow and keyboard navigation. In-app browser zoom required calibrated viewport and screenshot clipping dimensions; `control-css-1920-complete.jpg` is a full1920×1080 capture. Earlier captures retain the documented scale limitation.
