#!/usr/bin/env bash
# CPU sensor-observation benchmark; rendered navigation is verified on the host.
set -euo pipefail
./scripts/labctl nav fixture .runtime/ci-navigation-observations
./scripts/labctl nav benchmark .runtime/ci-navigation-observations .runtime/ci-navigation-benchmark
