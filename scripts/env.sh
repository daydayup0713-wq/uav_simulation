#!/usr/bin/env bash
# Execute only the system ROS underlay and this project's overlay.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec env -i HOME="$HOME" USER="${USER:-pine}" LANG=C.UTF-8 PATH=/usr/bin:/bin \
  DISPLAY="${DISPLAY:-}" XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}" \
  LAB_ROOT="$ROOT" LAB_REVISION="${LAB_REVISION:-}" ROS_DOMAIN_ID="${LAB_DOMAIN_ID:-42}" ROS_LOCALHOST_ONLY=1 \
  GZ_PARTITION="uav-lab-${USER:-pine}" GZ_IP=127.0.0.1 \
  OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=2 \
  /bin/bash --noprofile --norc -c '
    set -e
    source /opt/ros/humble/setup.bash
    if [ -f "$LAB_ROOT/.deps/slam/install/local_setup.bash" ]; then
      source "$LAB_ROOT/.deps/slam/install/local_setup.bash"
    fi
    if [ -d "$LAB_ROOT/.deps/slam/install/lib" ]; then
      export LD_LIBRARY_PATH="$LAB_ROOT/.deps/slam/install/lib:${LD_LIBRARY_PATH:-}"
      export CMAKE_PREFIX_PATH="$LAB_ROOT/.deps/slam/install:${CMAKE_PREFIX_PATH:-}"
      export AMENT_PREFIX_PATH="$LAB_ROOT/.deps/slam/install:${AMENT_PREFIX_PATH:-}"
    fi
    source "$LAB_ROOT/ros2_ws/install/setup.bash"
    export PYTHONUNBUFFERED=1
    if [ -f "$LAB_ROOT/.runtime/current-run" ]; then
      export LAB_RUN_DIR="$(cat "$LAB_ROOT/.runtime/current-run")"
      export ROS_LOG_DIR="$LAB_RUN_DIR/operator-logs"
    fi
    exec "$@"
  ' lab-env "$@"
