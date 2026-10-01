#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
/usr/bin/python3 "$ROOT/scripts/doctor.py" --runtime
exec "$ROOT/scripts/env.sh" /usr/bin/python3 "$ROOT/scripts/supervise.py" "$@"
