import json
import struct
from types import SimpleNamespace as N
import numpy as np
import pytest


def cloud(count=100):
    values=np.zeros((count,4),dtype='<f4');values[:,0]=np.arange(count);values[:,2]=2;values[:,3]=50
    return N(width=count,height=1,point_step=16,row_step=count*16,data=values.tobytes(),is_bigendian=False,
             fields=[N(name=n,datatype=7,count=1,offset=i*4) for i,n in enumerate(('x','y','z','intensity'))],
             header=N(frame_id='lidar_link',stamp=N(sec=3,nanosec=0)))


def decode(packet):
    size=struct.unpack_from('<I',packet)[0]
    metadata=json.loads(packet[4:4+size])
    offset=4+((size+3)//4)*4
    return metadata,np.frombuffer(packet,dtype='<f4',offset=offset).reshape(-1,4)


def test_binary_cloud_budget_and_label_leave_original_message_untouched():
    from uav_lab_experiments.observatory import cloud_packet
    msg=cloud(250001);original=msg.data
    header,points=decode(cloud_packet(msg,'raw',max_points=100000))
    assert len(points)<=100000 and header['source_count']==250001
    assert header['display_decimated'] and header['kind']=='cloud'
    assert header['source_frame']=='lidar_link' and header['stamp']==3
    assert msg.data is original
    assert points[0]==pytest.approx([0,0,2,50])


def test_display_transform_and_invalid_points_do_not_mutate_sensor_data():
    from uav_lab_experiments.observatory import cloud_packet
    msg=cloud(3);v=np.frombuffer(msg.data,dtype='<f4').copy().reshape(3,4);v[1,0]=np.nan;msg.data=v.tobytes()
    transform=np.eye(4);transform[:3,3]=[1,2,3]
    header,points=decode(cloud_packet(msg,'raw',transform=transform,frame='odom'))
    assert header['frame']=='odom' and len(points)==2
    assert not header['display_decimated'] and header['invalid_sample_points']==1
    assert points[0]==pytest.approx([1,2,5,50])
    assert np.isnan(np.frombuffer(msg.data,dtype='<f4')[4])


@pytest.mark.parametrize('fault',['buffer','field','stride'])
def test_malformed_clouds_are_rejected_without_allocating_unbounded_output(fault):
    from uav_lab_experiments.observatory import cloud_packet
    msg=cloud()
    if fault=='buffer':msg.data=b''
    elif fault=='field':msg.fields[0].offset=-4
    else:msg.row_step=4
    with pytest.raises(ValueError,match='cloud'):
        cloud_packet(msg,'raw')


def test_latest_frames_replace_instead_of_queueing_and_reject_truth():
    from uav_lab_experiments.observatory import LatestFrames
    store=LatestFrames()
    for i in range(1000):store.put('raw',str(i).encode())
    assert store.snapshot()=={'raw':(1000,b'999')}
    with pytest.raises(ValueError,match='evaluation'):
        store.put('truth',b'private evaluation pose')
    assert len(store.snapshot())==1
    enabled=LatestFrames(evaluation=True);enabled.put('truth',b'ok')
    assert 'truth' in enabled.snapshot()


def test_layer_allowlist_cannot_be_used_for_control_messages():
    from uav_lab_experiments.observatory import LatestFrames
    with pytest.raises(ValueError,match='layer'):
        LatestFrames().put('/fmu/in/vehicle_command',b'invalid')


def test_accumulated_registration_is_bounded_and_reset_on_frame_change():
    from uav_lab_experiments.observatory import Accumulation
    accumulator=Accumulation(max_points=100)
    for i in range(20):
        points=np.ones((20,4),dtype=np.float32);points[:,0]=i
        accumulator.add('odom',points)
    assert len(accumulator.points)<=100
    accumulator.add('map',np.ones((10,4),dtype=np.float32))
    assert accumulator.frame=='map' and len(accumulator.points)==10
