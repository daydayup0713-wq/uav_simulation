from pathlib import Path

from stop_lab import supervisor_identity


def test_owned_relative_supervisor_is_recognized_but_other_commands_are_rejected():
    root = Path('/workspace/lab')
    script = root/'scripts/supervise.py'
    assert supervisor_identity(['/usr/bin/python3', 'scripts/supervise.py', '--headless'], root, script)
    assert supervisor_identity(['/usr/bin/python3', str(script)], Path('/tmp'), script)
    assert not supervisor_identity(['/usr/bin/python3', 'scripts/supervise.py'], Path('/tmp'), script)
    assert not supervisor_identity(['/usr/bin/python3', '-c', str(script)], root, script)
