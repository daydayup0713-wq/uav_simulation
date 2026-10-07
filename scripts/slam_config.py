"""Bootstrap-compatible import of the shared recorded-calibration generator."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ros2_ws/src/uav_lab_localization'))
from uav_lab_localization.configuration import prepare_slam, read_json_comments
