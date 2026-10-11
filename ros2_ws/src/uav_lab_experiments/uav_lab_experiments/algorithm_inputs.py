"""Sensor-contract admission; never reads simulator geometry or truth."""
import math
import numpy as np


def livox_records(msg,lines=4,channel='line'):
    fields={field.name:field for field in msg.fields}
    count=msg.width*msg.height
    if msg.header.frame_id!='lidar_link' or not 0<count<=200000 or msg.height!=1 or len(msg.data)!=msg.row_step or msg.row_step<count*msg.point_step:
        raise ValueError('Livox cloud buffer/frame contract invalid')
    arrays={}
    for name in ('x','y','z','intensity','time','line'):
        datatype=4 if name=='line' else 7
        size=2 if name=='line' else 4
        f=fields.get(channel if name=='line' else name)
        if f is None or f.datatype!=datatype or f.count!=1 or not 0<=f.offset<=msg.point_step-size:
            raise ValueError('Livox required '+name+' field invalid')
        arrays[name]=np.ndarray((count,),dtype=('>' if msg.is_bigendian else '<')+('u2' if name=='line' else 'f4'),
                               buffer=msg.data,offset=f.offset,strides=(msg.point_step,))
    times=arrays['time'].astype(np.float64)
    if not np.isfinite(times).all() or np.any(times<0) or np.any(times>=.100001) or np.any(np.diff(times)<0) or times[-1]-times[0]<.05:
        raise ValueError('Livox source point time missing, regressing or out of scan interval')
    if not 1<=lines<=32 or np.any(arrays['line']>=lines):raise ValueError('Livox line contract invalid')
    valid=np.isfinite(np.column_stack([arrays[k] for k in 'xyz'])).all(axis=1)
    result=np.empty(np.count_nonzero(valid),dtype=[('x','<f4'),('y','<f4'),('z','<f4'),
                    ('reflectivity','u1'),('offset_ns','<u4'),('line','u1')])
    for name in 'xyz':result[name]=arrays[name][valid]
    result['reflectivity']=np.clip(np.nan_to_num(arrays['intensity'][valid]),0,255).astype('u1')
    result['offset_ns']=np.rint(times[valid]*1e9).astype('<u4');result['line']=arrays['line'][valid]
    stamp=msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
    if stamp<=0:raise ValueError('Livox source stamp invalid')
    return stamp,result


def ouster_cloud(msg, lines=16):
    """Encode measured mechanical scans for LIO-SAM's channel-preserving path.

    Timing is the measured beam offset, rounded to nanoseconds. Reflectivity
    is a synthetic intensity proxy; the sensor has no noise-channel estimate.
    Range is computed from measured XYZ, never from simulator geometry.
    """
    from sensor_msgs.msg import PointCloud2, PointField
    _, records = livox_records(msg, lines=lines, channel='ring')
    dtype = np.dtype({'names': ['x', 'y', 'z', 'intensity', 't', 'ring',
                               'reflectivity', 'noise', 'range'],
                     'formats': ['<f4'] * 4 + ['<u4', 'u1', '<u2', '<u2', '<u4'],
                     'offsets': [0, 4, 8, 12, 16, 20, 22, 24, 28], 'itemsize': 32})
    points = np.zeros(len(records), dtype=dtype)
    for name in 'xyz':
        points[name] = records[name]
    fields = {field.name: field for field in msg.fields}
    intensity = np.ndarray((msg.width,), dtype=('>' if msg.is_bigendian else '<')+'f4',
                           buffer=msg.data, offset=fields['intensity'].offset,
                           strides=(msg.point_step,))
    xyz = [np.ndarray((msg.width,), dtype=('>' if msg.is_bigendian else '<')+'f4',
                      buffer=msg.data, offset=fields[name].offset,
                      strides=(msg.point_step,)) for name in 'xyz']
    valid = np.isfinite(np.column_stack(xyz)).all(axis=1)
    points['intensity'] = np.nan_to_num(intensity[valid])
    points['t'] = records['offset_ns']
    points['ring'] = records['line']
    points['reflectivity'] = records['reflectivity']
    points['range'] = np.rint(np.linalg.norm(np.column_stack([points[n] for n in 'xyz']),
                                           axis=1) * 1000).astype('<u4')
    output = PointCloud2(header=msg.header, height=1, width=len(points), is_bigendian=False,
                         point_step=32, row_step=len(points)*32, is_dense=True,
                         data=points.tobytes())
    types = [PointField.FLOAT32]*4 + [PointField.UINT32, PointField.UINT8,
                                    PointField.UINT16, PointField.UINT16, PointField.UINT32]
    output.fields = [PointField(name=name, offset=dtype.fields[name][1], datatype=kind, count=1)
                     for name, kind in zip(dtype.names, types)]
    return output


class RtkAdmission:
    """Reject invalid fixes and physically implausible jumps before RTK factors.

    A failed observation never moves the accepted position/time. Bounds refer
    to the configured experiment speed plus measurement uncertainty, not truth.
    This does not detect coherent slow GNSS bias; report that limitation.
    """
    def __init__(self,max_speed=3.):
        self.max_speed=max_speed;self.last=None;self.last_source=None;self.reason='initializing'

    def observe(self,stamp,position,sigma,status):
        position=np.asarray(position,dtype=float)
        if not math.isfinite(stamp) or self.last_source is not None and stamp<=self.last_source:
            self.reason='GNSS time regression';return False
        self.last_source=stamp
        if status not in (0,2) or position.shape!=(3,) or not np.isfinite(position).all() or not math.isfinite(sigma) or not 0<sigma<=1.:
            self.reason='GNSS fix lost or invalid covariance';return False
        if self.last is not None:
            dt=stamp-self.last[0]
            uncertainty=5*math.hypot(sigma,self.last[2])
            if np.linalg.norm(position-self.last[1])>self.max_speed*dt+uncertainty:
                self.reason='GNSS innovation exceeds motion/uncertainty bounds';return False
        self.last=(stamp,position.copy(),sigma);self.reason='accepted';return True
