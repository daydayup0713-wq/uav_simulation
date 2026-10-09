#!/usr/bin/env bash
set -euo pipefail
case "${1:-}" in fast_lio2|fast_livo2|fast_livo2_rtk|lio_sam|orb_slam3|vins_fusion|glim) backend="$1";; *) echo 'unsupported private backend' >&2;exit 2;;esac
shift
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec "$ROOT/scripts/env.sh" /bin/bash --noprofile --norc -c '
 set -e
 backend="$1";shift
 # Keep the legacy baseline available to labctl, but exclude its native
 # libraries and package index from every isolated comparison backend.
 for key in LD_LIBRARY_PATH CMAKE_PREFIX_PATH AMENT_PREFIX_PATH; do
   IFS=: read -ra entries <<< "${!key:-}"
   kept=()
   for entry in "${entries[@]}"; do
     case "$entry" in ""|"$LAB_ROOT/.deps/slam/install"|"$LAB_ROOT/.deps/slam/install/"*) ;; *) kept+=("$entry");; esac
   done
   value="$(IFS=:; printf "%s" "${kept[*]}")"
   export "$key=$value"
 done
 if [ -f "$LAB_ROOT/.deps/backends/common/install/local_setup.bash" ]; then
   source "$LAB_ROOT/.deps/backends/common/install/local_setup.bash"
 fi
 prefix="$LAB_ROOT/.deps/backends/$backend/install"
 if [ -f "$prefix/local_setup.bash" ]; then source "$prefix/local_setup.bash";fi
 export CMAKE_PREFIX_PATH="$prefix:${CMAKE_PREFIX_PATH:-}"
 export LD_LIBRARY_PATH="$prefix/lib:${LD_LIBRARY_PATH:-}"
 exec "$@"
' backend-env "$backend" "$@"
