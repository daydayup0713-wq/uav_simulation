#!/usr/bin/env bash
# Analytical observation replay validates CPU backend reproducibility, not rendered sensor accuracy.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
./scripts/env.sh /usr/bin/python3 scripts/check_messages.py
./scripts/env.sh /usr/bin/python3 scripts/make_lio_fixture.py .runtime/ci-lio-fixture
./scripts/labctl slam benchmark .runtime/ci-lio-fixture --output .runtime/ci-lio-benchmark --skip-learning
./scripts/labctl slam map-save .runtime/ci-lio-benchmark .runtime/ci-lio-map
