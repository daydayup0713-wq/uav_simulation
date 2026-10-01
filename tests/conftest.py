import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for path in ('scripts', 'ros2_ws/src/uav_lab_bridge', 'ros2_ws/src/uav_lab_tools'):
    sys.path.insert(0, str(ROOT / path))
