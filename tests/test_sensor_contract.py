import importlib.util
import json
import struct
from pathlib import Path
from types import SimpleNamespace as N
import pytest

ROOT = Path(__file__).resolve().parents[1]

def module():
    assert importlib.util.find_spec('uav_lab_tools.sensor_audit') is not None, 'sensor audit missing'
    from uav_lab_tools import sensor_audit
    return sensor_audit

def calibration(tmp_path):
    from sensor_model import prepare_sensors
    return prepare_sensors(ROOT,tmp_path)['calibration']

def stamp(ns, frame):
    return N(stamp=N(sec=ns//10**9,nanosec=ns%10**9), frame_id=frame)

def cloud(ns=10**9, frame='lidar_link'):
    # Hand-derived valid 5760 points, including genuinely three-dimensional returns.
    data=b''.join(struct.pack('<fff',1.,2.,(i%16)*.1) for i in range(5760))
    return N(header=stamp(ns,frame),width=360,height=16,point_step=12,row_step=4320,
             is_bigendian=False,fields=[N(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate('xyz')],data=data)

def test_frequency_uses_simulation_time_not_delivery_speed():
    stats = module().StreamStats(10)
    for ns in (10**9,1100000000,1200000000):
        stats.observe(ns)
    assert stats.report()['source_hz'] == pytest.approx(10)
    assert stats.report()['passed']

@pytest.mark.parametrize('stamps',[(10**9,10**9),(10**9,900000000),(10**9,2*10**9)])
def test_time_duplicate_regression_and_gaps_fail(stamps):
    stats=module().StreamStats(10)
    for ns in stamps: stats.observe(ns)
    assert not stats.report()['passed']

def test_missing_streams_and_tf_fail(tmp_path):
    audit=module().SensorAudit(calibration(tmp_path))
    audit.observe('/uav001/lidar/points',cloud(),10**9)
    report=audit.report()
    assert not report['passed']
    assert '/uav001/imu/data' in report['missing']
    assert 'camera_optical_frame' in report['missing_transforms']

def test_wrong_frame_and_empty_cloud_rejected(tmp_path):
    audit=module().SensorAudit(calibration(tmp_path))
    audit.observe('/uav001/lidar/points',cloud(frame='base_link'),10**9)
    msg=cloud(1100000000);msg.data=b''
    audit.observe('/uav001/lidar/points',msg,1100000000)
    assert any('frame' in e for e in audit.report()['errors'])
    assert any('buffer' in e for e in audit.report()['errors'])

def test_sensor_message_stale_is_not_accepted(tmp_path):
    audit=module().SensorAudit(calibration(tmp_path))
    audit.observe('/uav001/lidar/points',cloud(),3*10**9)
    assert any('clock' in e for e in audit.report()['errors'])

def test_image_size_and_camera_calibration_are_checked(tmp_path):
    audit=module().SensorAudit(calibration(tmp_path))
    audit.observe('/uav001/camera/image_raw',N(header=stamp(10**9,'camera_optical_frame'),width=320,height=240,
                  encoding='rgb8',step=960,data=b''),10**9)
    audit.observe('/uav001/camera/camera_info',N(header=stamp(10**9,'camera_optical_frame'),width=320,height=240,
                  k=[1.,0,160.,0,1.,120.,0,0,1.],d=[0.]*5,p=[0.]*12,distortion_model='plumb_bob'),10**9)
    errors=audit.report()['errors']
    assert any('image buffer' in e for e in errors)
    assert any('intrinsics' in e for e in errors)

def test_invalid_extrinsic_rejected(tmp_path):
    audit=module().SensorAudit(calibration(tmp_path))
    tf=N(header=N(frame_id='base_link'),child_frame_id='lidar_link',
         transform=N(translation=N(x=0.,y=0.,z=1.),rotation=N(x=0.,y=0.,z=0.,w=1.)))
    audit.observe_static(N(transforms=[tf]))
    assert any('extrinsic' in e for e in audit.report()['errors'])

def test_imu_orientation_is_not_an_algorithm_observation():
    assert importlib.util.find_spec('uav_lab_tools.sensor_contract') is not None, 'sensor contract missing'
    from uav_lab_tools.sensor_contract import remove_orientation
    msg=N(orientation=N(x=.3,y=.2,z=.1,w=.9),orientation_covariance=[0.]*9,
          header=stamp(123,'imu_link'),angular_velocity=N(x=1.,y=2.,z=3.),linear_acceleration=N(x=0.,y=0.,z=9.81))
    remove_orientation(msg)
    assert msg.orientation_covariance[0] == -1
    assert (msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w)==(0,0,0,1)
    assert msg.header.stamp.nanosec==123 and msg.angular_velocity.x==1

def test_long_sensor_outage_cannot_hide_in_average_frequency():
    stats=module().StreamStats(10)
    for index in range(1001):
        if not 500<=index<550:stats.observe(10**9+index*100000000)
    report=stats.report()
    assert 9<report['source_hz']<10
    assert report['maximum_gap_s']>5
    assert not report['passed']

def test_paused_live_clock_fails_but_offline_read_speed_does_not(tmp_path,monkeypatch):
    audit_module=module();now=[0.]
    monkeypatch.setattr(audit_module.time,'monotonic',lambda:now[0])
    audit=audit_module.SensorAudit(calibration(tmp_path))
    audit.observe_clock(10**9)
    audit.observe('/uav001/lidar/points',cloud(),10**9)
    audit.observe('/uav001/lidar/points',cloud(1100000000),1100000000)
    for topic,(frame,hz) in {'/uav001/imu/data':('imu_link',200),'/uav001/camera/image_raw':('camera_optical_frame',15),'/uav001/camera/camera_info':('camera_optical_frame',15),'/uav001/ground_truth/odometry':('sim_world',25)}.items():
        for ns in (10**9,10**9+int(1e9/hz)):
            header=stamp(ns,frame)
            if topic.endswith('/data'):msg=N(header=header,angular_velocity=N(x=0.,y=0.,z=0.),linear_acceleration=N(x=0.,y=0.,z=9.81),orientation_covariance=[-1.]+[0.]*8)
            elif topic.endswith('/image_raw'):msg=N(header=header,width=320,height=240,encoding='rgb8',step=960,data=b'\x00\x7f\xff'*76800)
            elif topic.endswith('/camera_info'):msg=N(header=header,width=320,height=240,k=[160.,0,160.,0,160.,120.,0,0,1],d=[0.]*5,p=[160.,0,160.,0,0,160.,120.,0,0,0,1.,0],distortion_model='plumb_bob')
            else:msg=N(header=header,child_frame_id='truth_base_link',pose=N(pose=N(position=N(x=0.,y=0.,z=.227),orientation=N(x=0.,y=0.,z=0.,w=1.))))
            audit.observe(topic,msg,ns)
    transforms=[]
    for t in audit.calibration['transforms']:
        transforms.append(N(header=N(frame_id=t['parent']),child_frame_id=t['child'],transform=N(translation=N(**dict(zip('xyz',t['xyz']))),rotation=N(**dict(zip('xyzw',t['xyzw']))))))
    audit.observe_static(N(transforms=transforms));audit.observe_clock(1100000000)
    now[0]=59.
    assert audit.report()['passed'], 'offline source validation must ignore reader speed'
    assert not audit.report(live=True)['passed']
