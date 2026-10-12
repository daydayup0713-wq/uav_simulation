# Explicit maintenance

Run from the checkout whose managed data you intend to inspect:

```bash
./scripts/labctl maintenance inspect
./scripts/labctl maintenance plan --output /tmp/uav-clean-plan.json
./scripts/labctl maintenance apply --plan /tmp/uav-clean-plan.json
```

A plan lists exact candidates, sizes, identities, retention reasons and protected paths. Inspect it before applying. An empty plan is valid and releases zero bytes. Apply rechecks activity, references and file identity per item; changed or newly referenced items are refused. It continues after an individual failure and returns a nonzero exit status for refusals/errors. Every apply saves an audit in `.runtime/maintenance/`.

Defaults preserve seven days and three successes/three failures per localization/planning/input/scene/profile group. A success requires explicit acceptance evidence; an ordinary events log is insufficient. Ambiguous outcomes remain protected. Current, active and unclosed runs, recursively referenced report/qualification/configuration artifacts, original recordings, installed algorithms, dependency libraries and benchmark data remain protected.

Only old unreferenced closed runs, source `__pycache__`/`.pytest_cache` and explicitly owned closed source checkouts are eligible. Temporary source checkouts must have `maintenance-disposable.json` with schema 1, kind `source-checkout`, the exact project `root` and `closed: true`. Checkouts containing Git/dependency/build/install/recording/map state are preserved. Unknown temporary directories are preserved; do not add ownership metadata to arbitrary files merely to force their deletion.

The tool never follows symlinks during deletion or size estimation. Logical bytes and actual filesystem free-space delta are reported separately; concurrent experiment writes can change free space while maintenance runs. There is no automatic deletion job.

Retained report containers and policy-retained runs recursively protect the runs and artifacts they cite, including directory links. Candidate manifests do not self-protect. Unreadable or oversized reference documents conservatively protect runs instead of being skipped. Deletion-audit containers are operational history, not new evidence-reference roots.

Apply writes and fsyncs its audit before the first deletion, and before/after each item. Ctrl-C records an interrupted state; an unfinished pending item may be partially or fully removed and requires inspection. Even abrupt termination leaves the last durable deletion intent. Audit-storage failure stops further deletion. Directory descriptors anchor both deletion and audit updates against ancestor replacement.
