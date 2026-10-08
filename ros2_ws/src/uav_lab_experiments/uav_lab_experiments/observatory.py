"""Bounded observation protocol; contains no command or flight publishing API."""
import json
import math
import struct
import threading
import numpy as np

LAYERS=('raw','registered','global','voxels')
CHANNELS=(*LAYERS,'telemetry','planned','actual','registry')


def read_cloud(msg,max_points=100000,transform=None,frame=None):
    count=msg.width*msg.height
    if not 1<=max_points<=100000 or not 0<=count<=20_000_000:
        raise ValueError('cloud point budget invalid')
    if msg.height<1 or msg.width<0 or msg.point_step<12 or msg.row_step<msg.width*msg.point_step or len(msg.data)!=msg.height*msg.row_step:
        raise ValueError('cloud buffer/stride invalid')
    fields={f.name:f for f in msg.fields}
    for name in 'xyz':
        if name not in fields or fields[name].datatype!=7 or fields[name].count!=1 or not 0<=fields[name].offset<=msg.point_step-4:
            raise ValueError('cloud FLOAT32 xyz layout invalid')
    stride=max(1,math.ceil(count/max_points));dtype='>f4' if msg.is_bigendian else '<f4'
    values=[]
    for name in ('x','y','z','intensity'):
        f=fields.get(name)
        if f is None:
            values.append(np.zeros(math.ceil(count/stride),dtype=np.float32));continue
        if f.datatype!=7 or f.count!=1 or not 0<=f.offset<=msg.point_step-4:
            if name=='intensity':values.append(np.zeros(math.ceil(count/stride),dtype=np.float32));continue
            raise ValueError('cloud field layout invalid')
        values.append(np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,offset=f.offset,
                                 strides=(msg.row_step,msg.point_step)).ravel()[::stride])
    points=np.column_stack(values).astype('<f4',copy=False)
    selected_count=len(points)
    points=points[np.isfinite(points[:,:3]).all(axis=1)]
    points[:,3]=np.nan_to_num(points[:,3],nan=0.,posinf=0.,neginf=0.)
    if transform is not None:
        transform=np.asarray(transform)
        if transform.shape!=(4,4) or not np.isfinite(transform).all():raise ValueError('cloud display transform invalid')
        points[:,:3]=points[:,:3]@transform[:3,:3].T+transform[:3,3]
    metadata={'schema':1,'kind':'cloud','source_count':count,'display_decimated':stride>1,
              'invalid_sample_points':selected_count-len(points),
              'frame':frame or msg.header.frame_id,'source_frame':msg.header.frame_id,
              'stamp':msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,'format':'little-endian float32 xyzi'}
    return points,metadata


def pack_points(points,metadata):
    points=np.asarray(points,dtype='<f4')
    if points.ndim!=2 or points.shape[1]!=4 or len(points)>100000:raise ValueError('cloud display budget exceeded')
    header=json.dumps({**metadata,'count':len(points)},allow_nan=False,separators=(',',':')).encode()
    if len(header)>8192:raise ValueError('cloud metadata too large')
    return struct.pack('<I',len(header))+header+b' '*((-len(header))%4)+points.tobytes()


def cloud_packet(msg,layer,max_points=100000,transform=None,frame=None):
    if layer not in LAYERS:raise ValueError('cloud display layer invalid')
    points,metadata=read_cloud(msg,max_points,transform,frame)
    return pack_points(points,{**metadata,'layer':layer})


class LatestFrames:
    def __init__(self,evaluation=False):
        self.evaluation=evaluation;self.frames={};self.lock=threading.Lock()

    def put(self,layer,packet):
        if layer=='truth' and not self.evaluation:raise ValueError('truth requires evaluation mode')
        if layer not in CHANNELS and layer!='truth':raise ValueError('unknown observation layer')
        if not isinstance(packet,(bytes,str)) or len(packet)>1_610_000:raise ValueError('observation frame too large')
        with self.lock:
            version=self.frames.get(layer,(0,None))[0]+1
            self.frames[layer]=(version,packet)

    def snapshot(self):
        with self.lock:return dict(self.frames)


class Accumulation:
    """Finite display-only window; estimator maps/recordings remain untouched."""
    def __init__(self,max_points=100000):
        if not 1<=max_points<=100000:raise ValueError('accumulation point budget invalid')
        self.max_points=max_points;self.frame=None;self.points=np.empty((0,4),dtype='<f4')

    def add(self,frame,points):
        if frame!=self.frame:
            self.frame=frame;self.points=np.empty((0,4),dtype='<f4')
        self.points=np.concatenate([self.points,points])[-self.max_points:]
        return self.points
