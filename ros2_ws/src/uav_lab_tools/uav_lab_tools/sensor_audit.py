"""Bounded, source-time sensor contract checks, reusable on live data and bags."""
import math
import struct
import time
from .sensor_contract import SENSOR_TOPICS, stamp_ns

class StreamStats:
    def __init__(self, hz):
        self.hz, self.count, self.first, self.last = hz,0,None,None
        self.regressions, self.maximum_gap_ns = 0,0

    def observe(self, ns):
        if self.last is not None:
            self.regressions += int(ns <= self.last)
            self.maximum_gap_ns=max(self.maximum_gap_ns,ns-self.last)
        if self.first is None:
            self.first=ns
        self.last=ns; self.count+=1

    def report(self):
        span=(self.last-self.first)/1e9 if self.count>1 else 0
        hz=(self.count-1)/span if span>0 else 0
        return {'count':self.count,'source_hz':hz,'expected_hz':self.hz,
                'first_ns':self.first,'last_ns':self.last,'regressions':self.regressions,
                'maximum_gap_s':self.maximum_gap_ns/1e9,
                'passed':self.count>=2 and self.first>0 and not self.regressions and .8*self.hz<=hz<=1.2*self.hz}

class SensorAudit:
    def __init__(self, calibration):
        self.calibration=calibration
        self.stats={topic:StreamStats(calibration[section]['hz']) for topic,(_,section,_) in SENSOR_TOPICS.items()}
        self.errors=set(); self.transforms=set()
        self.clock_first=None; self.clock_last=None; self.clock_count=0
        self.wall_first=time.monotonic()
        self.cloud_summary={}; self.imu_norm_sum=0.;self.imu_count=0

    def error(self, message):
        if len(self.errors)<30: self.errors.add(message)

    def observe_clock(self, ns):
        if self.clock_last is not None and ns<self.clock_last:
            self.error('simulation clock regression')
        if self.clock_first is None: self.clock_first=ns
        self.clock_last=ns;self.clock_count+=1

    def observe_static(self, msg):
        expected={v['child']:v for v in self.calibration['transforms']}
        for tf in msg.transforms:
            if tf.child_frame_id not in expected: continue
            target=expected[tf.child_frame_id]
            p,q=tf.transform.translation,tf.transform.rotation
            position=[p.x,p.y,p.z];quat=[q.x,q.y,q.z,q.w]
            distance=math.dist(position,target['xyz'])
            rotation=min(math.dist(quat,target['xyzw']),math.dist([-v for v in quat],target['xyzw']))
            if tf.header.frame_id!=target['parent'] or not math.isfinite(distance+rotation) or distance>1e-6 or rotation>1e-6:
                self.error('extrinsic mismatch: '+tf.child_frame_id)
            else:
                self.transforms.add(tf.child_frame_id)

    def observe(self, topic, msg, clock_ns=None):
        if topic not in self.stats: return
        _,section,frame_key=SENSOR_TOPICS[topic]
        c=self.calibration[section]
        ns=stamp_ns(msg.header.stamp)
        self.stats[topic].observe(ns)
        if msg.header.frame_id!=c[frame_key]: self.error(topic+': frame mismatch '+msg.header.frame_id)
        if clock_ns is not None and abs(ns-clock_ns)>500000000:
            self.error(topic+': source time differs from clock by >0.5s')
        try:
            if section=='lidar': self.cloud(msg,c)
            elif topic.endswith('/image_raw'): self.image(msg,c)
            elif topic.endswith('/camera_info'): self.camera_info(msg,c)
            elif section=='imu':
                values=[getattr(v,axis) for v in (msg.angular_velocity,msg.linear_acceleration) for axis in 'xyz']
                if not all(math.isfinite(v) for v in values): self.error('nonfinite IMU measurement')
                if msg.orientation_covariance[0]!=-1: self.error('IMU ideal orientation exposed to algorithm')
                self.imu_norm_sum+=math.sqrt(sum(v*v for v in values[3:]));self.imu_count+=1
            else:
                if msg.child_frame_id!=c['child_frame']: self.error('truth child frame mismatch')
                p,q=msg.pose.pose.position,msg.pose.pose.orientation
                values=[p.x,p.y,p.z,q.x,q.y,q.z,q.w]
                if not all(math.isfinite(v) for v in values) or abs(sum(v*v for v in values[3:])-1)>.01:
                    self.error('invalid ground truth pose')
        except (AttributeError,ValueError,struct.error,OverflowError,IndexError) as exc:
            self.error(topic+': malformed data '+str(exc))

    def cloud(self,msg,c):
        count=msg.width*msg.height
        if count!=c['horizontal_samples']*c['vertical_samples']: self.error('point cloud sample count mismatch')
        if msg.point_step<12 or msg.row_step<msg.width*msg.point_step or len(msg.data)!=msg.row_step*msg.height:
            self.error('point cloud buffer layout mismatch');return
        fields={v.name:v for v in msg.fields}
        if not all(n in fields and fields[n].datatype==7 and fields[n].count==1 and 0<=fields[n].offset<=msg.point_step-4 for n in 'xyz'):
            self.error('point cloud FLOAT32 xyz fields missing');return
        data=memoryview(msg.data);fmt='>f' if msg.is_bigendian else '<f'
        finite=0;zs=[]
        for i in range(0,count,max(1,count//256)):
            offset=(i//msg.width)*msg.row_step+(i%msg.width)*msg.point_step
            xyz=[struct.unpack_from(fmt,data,offset+fields[n].offset)[0] for n in 'xyz']
            if all(math.isfinite(v) for v in xyz): finite+=1;zs.append(xyz[2])
        sampled=len(range(0,count,max(1,count//256)))
        if not zs or finite/max(1,sampled)<.1: self.error('point cloud has insufficient finite returns')
        if zs and max(zs)-min(zs)<.05: self.error('point cloud lacks vertical extent')
        self.cloud_summary={'points_per_scan':count,'finite_sample_fraction':finite/max(1,sampled),
                            'sampled_z_extent_m':max(zs)-min(zs) if zs else 0}

    def image(self,msg,c):
        if (msg.width,msg.height)!=(c['width'],c['height']) or msg.encoding!='rgb8': self.error('image size/encoding mismatch')
        if msg.step!=msg.width*3 or len(msg.data)!=msg.step*msg.height: self.error('image buffer mismatch')
        elif max(msg.data,default=0)-min(msg.data,default=0)<10: self.error('image rendering appears blank')

    def camera_info(self,msg,c):
        fx=c['width']/(2*math.tan(c['horizontal_fov_rad']/2))
        expected=[fx,0,c['width']/2,0,fx,c['height']/2,0,0,1]
        if (msg.width,msg.height)!=(c['width'],c['height']) or any(not math.isfinite(v) or abs(v-e)>.51 for v,e in zip(msg.k,expected)):
            self.error('camera intrinsics mismatch')
        if msg.distortion_model!='plumb_bob' or any(v!=0 for v in msg.d): self.error('camera distortion mismatch')

    def report(self):
        streams={t:s.report() for t,s in self.stats.items()}
        missing=[t for t,r in streams.items() if r['count']<2]
        missing_tf=[v['child'] for v in self.calibration['transforms'] if v['child'] not in self.transforms]
        if self.clock_count>=2:
            for topic,s in self.stats.items():
                if s.last is not None and self.clock_last-s.last>max(500000000,int(3e9/s.hz)):
                    self.error(topic+': stream stale at final clock')
        mean_imu=self.imu_norm_sum/self.imu_count if self.imu_count else None
        if mean_imu is not None and not 5<mean_imu<20: self.error('IMU mean specific force outside flight envelope')
        span=(self.clock_last-self.clock_first)/1e9 if self.clock_count>=2 else 0
        return {'passed':not self.errors and not missing and not missing_tf and self.clock_count>=2 and all(v['passed'] for v in streams.values()),
                'streams':streams,'missing':missing,'missing_transforms':missing_tf,'errors':sorted(self.errors),
                'clock_samples':self.clock_count,'sim_duration_s':span,'real_time_factor':span/max(.001,time.monotonic()-self.wall_first),
                'lidar':self.cloud_summary,'imu_mean_specific_force_m_s2':mean_imu}
