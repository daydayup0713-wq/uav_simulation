import numpy as np
import pytest
from sensor_msgs.msg import PointCloud2,PointField


def cloud():
    msg=PointCloud2();msg.header.frame_id='lidar_link';msg.header.stamp.sec=10
    msg.height=1;msg.width=3;msg.point_step=24;msg.row_step=72
    msg.fields=[PointField(name=n,offset=o,datatype=t,count=1) for n,o,t in
        [('x',0,7),('y',4,7),('z',8,7),('intensity',12,7),('time',16,7),('ring',20,4),('line',22,4)]]
    dtype=np.dtype({'names':['x','y','z','intensity','time','line'],
        'formats':['<f4']*5+['<u2'],'offsets':[0,4,8,12,16,22],'itemsize':24})
    values=np.zeros(3,dtype=dtype);values['x']=[1,np.nan,2];values['time']=[0,.05,.099];values['line']=[0,1,3]
    values['intensity']=[12,50,300];msg.data=values.tobytes();return msg


def test_livox_conversion_preserves_source_and_real_offsets_filters_invalid_returns():
    from uav_lab_experiments.algorithm_inputs import livox_records
    stamp,points=livox_records(cloud(),lines=4)
    assert stamp==10000000000
    assert points['offset_ns'][0]==0 and abs(int(points['offset_ns'][1])-99000000)<=4
    assert points['line'].tolist()==[0,3]
    assert points['reflectivity'].tolist()==[12,255]


def test_livox_input_rejects_fake_or_regressing_timing_and_wrong_lines():
    from uav_lab_experiments.algorithm_inputs import livox_records
    msg=cloud();msg.fields=[f for f in msg.fields if f.name!='time']
    with pytest.raises(ValueError,match='time'):livox_records(msg,lines=4)
    msg=cloud();data=bytearray(msg.data);data[16:20]=np.float32(.2).tobytes();msg.data=bytes(data)
    with pytest.raises(ValueError,match='time'):livox_records(msg,lines=4)
    with pytest.raises(ValueError,match='line'):livox_records(cloud(),lines=2)


def test_rtk_rejection_lost_and_implausible_jump_then_recovery():
    from uav_lab_experiments.algorithm_inputs import RtkAdmission
    gate=RtkAdmission()
    assert gate.observe(1,[0,0,0],.02,2)
    assert not gate.observe(1.1,[30,0,0],.02,2)
    assert not gate.observe(1.2,[np.nan]*3,.02,-1)
    assert gate.observe(1.3,[.1,0,0],.02,2)
    assert not gate.observe(1.2,[.1,0,0],.02,2)


def test_ouster_conversion_preserves_actual_channels_offsets_and_range():
    from uav_lab_experiments.algorithm_inputs import ouster_cloud
    msg = cloud()
    data = bytearray(msg.data)
    for index, ring in enumerate([2, 1, 15]):
        data[index * 24 + 20:index * 24 + 22] = np.uint16(ring).tobytes()
    msg.data = bytes(data)
    output = ouster_cloud(msg, lines=16)
    assert output.header == msg.header
    assert output.width == 2 and output.is_dense and not output.is_bigendian
    fields = {f.name: f for f in output.fields}
    assert fields['ring'].datatype == PointField.UINT8
    assert fields['t'].datatype == PointField.UINT32
    def values(name, dtype):
        return np.ndarray((output.width,), dtype=dtype, buffer=output.data,
                          offset=fields[name].offset, strides=(output.point_step,))
    assert values('ring', 'u1').tolist() == [2, 15]
    assert values('t', '<u4')[0] == 0
    assert abs(int(values('t', '<u4')[1]) - 99000000) <= 4
    assert values('x', '<f4').tolist() == [1., 2.]
    assert values('range', '<u4').tolist() == [1000, 2000]
    assert values('intensity', '<f4').tolist() == [12., 300.]
    assert values('noise', '<u2').tolist() == [0, 0]


def test_ouster_conversion_rejects_invalid_timing_and_channel():
    from uav_lab_experiments.algorithm_inputs import ouster_cloud
    msg=cloud();msg.fields=[f for f in msg.fields if f.name != 'time']
    with pytest.raises(ValueError, match='time'):
        ouster_cloud(msg, lines=16)
    msg=cloud();data=bytearray(msg.data);data[20:22]=np.uint16(20).tobytes();msg.data=bytes(data)
    with pytest.raises(ValueError, match='line'):
        ouster_cloud(msg, lines=16)
