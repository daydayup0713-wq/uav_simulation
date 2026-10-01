#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
mkdir -p .runtime
./scripts/start_lab.sh --headless > .runtime/ci-supervisor.log 2>&1 &
SUPERVISOR=$!
trap 'kill -TERM "$SUPERVISOR" 2>/dev/null || true; wait "$SUPERVISOR" 2>/dev/null || true' EXIT
READY=0
for ((n=0; n<120; n++)); do
  kill -0 "$SUPERVISOR" || { cat .runtime/ci-supervisor.log; exit 1; }
  if PYTHONPATH=scripts /usr/bin/python3 -c 'import sys; from pathlib import Path; from supervise import owned_run_ready; sys.exit(0 if owned_run_ready(Path(".runtime").resolve(), int(sys.argv[1])) else 1)' "$SUPERVISOR"; then
    READY=1
    break
  fi
  sleep 1
done
[ "$READY" -eq 1 ] || { cat .runtime/ci-supervisor.log; echo 'CI readiness timeout'; exit 1; }
kill -0 "$SUPERVISOR"
./scripts/env.sh env PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest -q
./scripts/labctl demo --runs 3
./scripts/labctl arm
./scripts/labctl takeoff --height 2
./scripts/env.sh /usr/bin/python3 scripts/verify_fault.py
