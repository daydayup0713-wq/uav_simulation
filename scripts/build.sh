#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec env -i HOME="$HOME" USER="${USER:-pine}" PATH=/usr/bin:/bin LANG=C.UTF-8 LAB_ROOT="$ROOT" \
  /bin/bash --noprofile --norc -c '
    set -e
    source /opt/ros/humble/setup.bash
    cd "$LAB_ROOT/ros2_ws"
    colcon build --symlink-install --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release
  '
