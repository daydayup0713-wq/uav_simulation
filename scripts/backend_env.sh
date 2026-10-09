#!/usr/bin/env bash
set -euo pipefail
case "${1:-}" in fast_lio2|fast_livo2|fast_livo2_rtk) backend="$1";; *) echo 'unsupported private backend' >&2;exit 2;;esac
shift
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec "$ROOT/scripts/env.sh" /bin/bash --noprofile --norc -c '
 set -e
 backend="$1";shift
 source "$LAB_ROOT/.deps/backends/common/install/local_setup.bash"
 prefix="$LAB_ROOT/.deps/backends/$backend/install"
 if [ -f "$prefix/local_setup.bash" ]; then source "$prefix/local_setup.bash";fi
 export CMAKE_PREFIX_PATH="$prefix:${CMAKE_PREFIX_PATH:-}"
 export LD_LIBRARY_PATH="$prefix/lib:${LD_LIBRARY_PATH:-}"
 exec "$@"
' backend-env "$backend" "$@"
