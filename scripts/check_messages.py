#!/usr/bin/python3
"""Check the eight PX4 boundary schemas, including MESSAGE_VERSION constants."""
from pathlib import Path

MESSAGES = ('VehicleLocalPosition', 'VehicleStatus', 'VehicleAttitude',
            'VehicleLandDetected', 'VehicleCommandAck', 'VehicleCommand',
            'OffboardControlMode', 'TrajectorySetpoint')

def declarations(path):
    return [' '.join(text.split()) for line in Path(path).read_text().splitlines()
            if (text := line.split('#', 1)[0].strip())]

def check_messages(root):
    root = Path(root)
    result = {}
    for name in MESSAGES:
        matches = list((root/'.deps/px4/msg').rglob(name+'.msg'))
        other = root/'ros2_ws/src/px4_msgs/msg'/(name+'.msg')
        result[name] = len(matches) == 1 and other.is_file() and declarations(matches[0]) == declarations(other)
    return result

if __name__ == '__main__':
    import json
    results = check_messages(Path(__file__).resolve().parents[1])
    print(json.dumps(results, indent=2))
    raise SystemExit(0 if all(results.values()) else 1)
