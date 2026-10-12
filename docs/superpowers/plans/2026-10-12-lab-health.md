# UAV Lab health and slimming implementation plan

Goal: preserve all backends and public entrances, resolve unexpected runtime latches, explain every wait/failure, and remove only unreferenced old runtime data.
Spec: the user-approved plan in this conversation (2026-10-12).
Constraints: unchanged safety gates; seven days retention; three successes and failures per pair/input/scene; protect active runs, reference closures, recordings, dependencies and licenses. No merge/release/hardware changes.

## Task 1: Baseline and traceable audit
- Record clean source identity, tests, build, static findings and disk inventory.
- Resolve worktree setup before treating build/interface absence as regression.
- Produce docs/validation/health/audit.md with entrypoint evidence and disposition.
- Verify baseline native Python suite and frontend test/build, commit the baseline.

## Task 2: Runtime stability
- Add bounded source/receipt/callback/map/fusion timing evidence and host resource sampling.
- Reproduce and distinguish simulator clock stalls, source loss and callback contention.
- Write failing runtime tests for the proved cause, fix it while retaining thresholds/latches.
- Validate static/timed maps and freshness/fault behavior; commit independently.

## Task 3: Code and render slimming
- Audit dynamic entrypoints before removing unused imports/variables; explicit compatibility exports.
- Share equivalent voxel integration, preserving observed-space behavior.
- Test render resource reuse before implementation; separate frontend development dependencies.
- Add Python and TypeScript static gates; run the suites and commit.

## Task 4: Web feedback
- Extend status compatibly with operation blockers, diagnostics and real startup stages/timing.
- Tests first: read-only 8080, fail-safe explanation, loading/error/empty/stale results, bounded requests and uncertain commands.
- Preserve backend authorization; no motion retries; accessible visible reasons and responsive navigation.
- Verify frontend/API tests, build and three viewport captures; commit.

## Task 5: Retention and acceptance
- Test inspect/plan/apply reference closure, active ownership, retention, symlinks, changed identity and partial failures first.
- Add explicit maintenance CLI with immutable plans and per-item rechecks, audit retained bytes and released bytes.
- Run three 30-minute native ground sessions, three complex-route flights, five fault cases and identity qualification.
- Validate retained artifacts/datasets, deliver complete report with limitations, commit and one independent whole-branch review.
