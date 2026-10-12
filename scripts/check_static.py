#!/usr/bin/python3
"""Check tracked platform Python sources, including explicit compatibility exports."""
import io
from pathlib import Path
import subprocess
from pyflakes.api import checkPath
from pyflakes.reporter import Reporter


def findings(paths):
    out=io.StringIO()
    reporter=Reporter(out,out)
    for path in paths:checkPath(str(path),reporter=reporter)
    return out.getvalue().splitlines()


def main():
    root=Path(__file__).resolve().parents[1]
    tracked=subprocess.check_output(['git','ls-files','--cached','--others','--exclude-standard'],cwd=root,text=True).splitlines()
    paths=[root/name for name in tracked if name.endswith('.py') and
        (name.startswith('scripts/') or name.startswith('ros2_ws/src/'))]
    result=findings(paths)
    print('\n'.join(result) if result else 'Static Python checks passed')
    return bool(result)


if __name__=='__main__':raise SystemExit(main())
