#!/usr/bin/python3
"""Request graceful shutdown only of the current lab-owned supervisor."""
import json
import os
from pathlib import Path
import signal

ROOT = Path(__file__).resolve().parents[1]

def main():
    run = Path((ROOT/'.runtime/current-run').read_text().strip())
    if run.parent != ROOT/'.runtime':
        raise RuntimeError('invalid runtime directory')
    pid = json.loads((run/'manifest.json').read_text())['supervisor_pid']
    try:
        command = Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\0')
    except FileNotFoundError:
        print('lab already stopped')
        return
    if str(ROOT/'scripts/supervise.py').encode() not in command:
        raise RuntimeError('supervisor PID identity mismatch')
    os.kill(pid, signal.SIGTERM)

if __name__ == '__main__':
    main()
