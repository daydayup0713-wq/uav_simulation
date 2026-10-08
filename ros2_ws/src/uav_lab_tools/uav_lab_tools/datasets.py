"""Dataset integrity and replay policy, independent of a ROS context."""
import hashlib
import json
from pathlib import Path
from .sensor_contract import RECORD_TOPICS, SENSOR_TOPICS, stamp_ns, sensor_topics, record_topics
from .sensor_audit import SensorAudit

TOPIC_TYPES={t:v[0] for t,v in SENSOR_TOPICS.items()}
TOPIC_TYPES.update({'/uav001/gnss/fix':'sensor_msgs/msg/NavSatFix','/uav001/sensors/timed_diagnostics':'diagnostic_msgs/msg/DiagnosticArray'})
TOPIC_TYPES.update({'/clock':'rosgraph_msgs/msg/Clock','/tf':'tf2_msgs/msg/TFMessage',
                    '/tf_static':'tf2_msgs/msg/TFMessage','/uav001/odometry':'nav_msgs/msg/Odometry',
                    '/uav001/path':'nav_msgs/msg/Path','/uav001/diagnostics':'diagnostic_msgs/msg/DiagnosticArray'})

def replay_topics(types):
    for topic,typename in types.items():
        if topic not in TOPIC_TYPES: raise ValueError('forbidden replay topic: '+topic)
        if TOPIC_TYPES[topic]!=typename: raise ValueError('wrong replay message type: '+topic)
    return sorted(types)

def check_replay_domain(domain, active_domains):
    if not 0<=domain<=232 or domain in active_domains:
        raise ValueError('replay domain must be 0..232 and different from every active lab domain')

def file_hash(path):
    digest=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):digest.update(block)
    return digest.hexdigest()

def check_files(directory, hashes):
    directory=Path(directory).resolve()
    for relative,want in hashes.items():
        path=(directory/relative).resolve()
        if not path.is_relative_to(directory) or Path(relative).is_absolute():
            raise ValueError('invalid archive path: '+relative)
        if not path.is_file() or file_hash(path)!=want: raise ValueError('archive checksum mismatch: '+relative)

def load_dataset(directory, allow_incomplete=False):
    directory=Path(directory)
    metadata=json.loads((directory/'dataset.json').read_text())
    if not allow_incomplete and not metadata.get('complete'):
        raise ValueError('dataset incomplete: '+metadata.get('reason','recording not finalized'))
    check_files(directory/'configuration',metadata.get('configuration_sha256',{}))
    check_files(directory/'bag',metadata.get('bag_sha256',{}))
    return metadata

def inspect_bag(directory, allow_incomplete=False):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    directory=Path(directory)
    metadata=load_dataset(directory,allow_incomplete)
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(directory/'bag'),storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
    types={t.name:t.type for t in reader.get_all_topics_and_types()}
    replay_topics(types)
    classes={t:get_message(v) for t,v in types.items()}
    audit=SensorAudit(metadata['calibration']);counts={t:0 for t in types};clock=None
    first_receive=None;last_receive=None
    while reader.has_next():
        topic,data,received_ns=reader.read_next();counts[topic]+=1
        if first_receive is None:first_receive=received_ns
        last_receive=received_ns
        if topic not in sensor_topics(metadata['calibration']) and topic not in ('/clock','/tf_static'):continue
        msg=deserialize_message(data,classes[topic])
        if topic=='/clock':clock=stamp_ns(msg.clock);audit.observe_clock(clock)
        elif topic=='/tf_static':audit.observe_static(msg)
        else:audit.observe(topic,msg,clock)
    report=audit.report()
    missing=[t for t in record_topics(metadata['calibration']) if not counts.get(t)]
    report.update(topic_counts=counts,missing_recording_topics=missing,dataset=str(directory))
    report['bag_duration_s']=(last_receive-first_receive)/1e9 if first_receive is not None else 0
    report['passed']=report['passed'] and not missing
    # Wall-time replay/read speed is not the simulated real-time factor.
    report.pop('real_time_factor',None)
    del reader
    return report,types
