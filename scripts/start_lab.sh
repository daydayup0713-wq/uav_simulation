#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
DOCTOR_ARGS=(--runtime)
for arg in "$@"; do
  case "$arg" in sensors|localization|slam) DOCTOR_ARGS+=(--sensors) ;; esac
done
/usr/bin/python3 "$ROOT/scripts/doctor.py" "${DOCTOR_ARGS[@]}"
exec "$ROOT/scripts/env.sh" /usr/bin/python3 "$ROOT/scripts/supervise.py" "$@"
