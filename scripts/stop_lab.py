#!/usr/bin/python3
"""Request graceful shutdown only of the current lab-owned supervisor."""
import json
import os
from pathlib import Path
import signal

ROOT = Path(__file__).resolve().parents[1]

def supervisor_identity(command, cwd, script):
    if len(command) < 2 or Path(command[0]).name not in ('python3', 'python3.10'):
        return False
    candidate = Path(command[1])
    if not candidate.is_absolute():
        candidate = Path(cwd)/candidate
    return candidate.resolve() == Path(script).resolve()

def main():
    run = Path((ROOT/'.runtime/current-run').read_text().strip())
    if run.parent != ROOT/'.runtime':
        raise RuntimeError('invalid runtime directory')
    pid = json.loads((run/'manifest.json').read_text())['supervisor_pid']
    try:
        command = [v.decode() for v in Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\0') if v]
        cwd = Path(f'/proc/{pid}/cwd').resolve()
    except FileNotFoundError:
        print('lab already stopped')
        return
    if not supervisor_identity(command, cwd, ROOT/'scripts/supervise.py'):
        raise RuntimeError('supervisor PID identity mismatch')
    os.kill(pid, signal.SIGTERM)

if __name__ == '__main__':
    main()
