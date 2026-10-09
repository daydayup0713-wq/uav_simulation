"""Measured-time Livox and admitted GNSS inputs; no simulator-truth input."""
import json,math,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2,NavSatFix
from diagnostic_msgs.msg import DiagnosticArray,DiagnosticStatus,KeyValue
from .algorithm_inputs import livox_records,RtkAdmission

class AlgorithmSensorAdapter(Node):
    def __init__(self):
        super().__init__('algorithm_sensor_adapter',namespace='uav001')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
        self.backend=self.declare_parameter('backend','fast_lio2').value
        if self.backend not in ('fast_lio2','fast_livo2','fast_livo2_rtk'):raise ValueError('unsupported backend')
        calibration=json.loads(Path(self.declare_parameter('calibration','').value).read_text())
        self.mechanical=calibration['lidar']['kind']=='mechanical'
        self.lines=calibration['lidar']['vertical_samples']
        if self.mechanical and self.backend!='fast_lio2':raise ValueError('backend mechanical input unsupported')
        if self.mechanical:CustomMsg,CustomPoint=PointCloud2,None
        elif self.backend=='fast_livo2_rtk':
            from livox_ros_driver.msg import CustomMsg,CustomPoint
        else:
            from livox_ros_driver2.msg import CustomMsg,CustomPoint
        self.CustomMsg,self.CustomPoint=CustomMsg,CustomPoint
        self.prefix='/uav001/backends/'+self.backend
        self.pub=self.create_publisher(CustomMsg,self.prefix+('/points' if self.mechanical else '/livox'),8)
        self.create_subscription(PointCloud2,'/uav001/lidar/points',self.cloud,qos_profile_sensor_data)
        self.diag=self.create_publisher(DiagnosticArray,self.prefix+'/input_diagnostics',10)
        self.last=None;self.clouds=0;self.rejects=0;self.error='initializing';self.conversion_ms=0.
        self.gnss=RtkAdmission();self.gnss_counts={'accepted':0,'rejected':0};self.gnss_origin=None;self.previous_fix=None
        self.trace=None
        output=self.declare_parameter('trace_file','').value
        if output:self.trace=Path(output).open('x')
        if self.backend=='fast_livo2_rtk':
            from gnss_comm.msg import GnssPVTSolnMsg
            self.PVT=GnssPVTSolnMsg
            self.gnss_origin=calibration['gnss']['origin']
            self.gnss_pub=self.create_publisher(GnssPVTSolnMsg,self.prefix+'/gnss_pvt',16)
            self.create_subscription(NavSatFix,'/uav001/gnss/fix',self.fix,qos_profile_sensor_data)
        self.create_timer(1.,self.diagnostics)

    def record(self,**value):
        if self.trace:self.trace.write(json.dumps(value,allow_nan=False)+'\n');self.trace.flush()

    def cloud(self,msg):
        try:
            start=time.perf_counter();stamp,records=livox_records(msg,lines=self.lines,channel='ring' if self.mechanical else 'line')
            if self.last is not None and stamp<=self.last:raise ValueError('source cloud clock regression')
            if self.mechanical:output=msg
            else:
                output=self.CustomMsg();output.header=msg.header;output.timebase=stamp;output.point_num=len(records)
                output.points=[self.CustomPoint(x=float(p['x']),y=float(p['y']),z=float(p['z']),
                    offset_time=int(p['offset_ns']),reflectivity=int(p['reflectivity']),line=int(p['line']),tag=0x10) for p in records]
            self.pub.publish(output);self.last=stamp;self.clouds+=1;self.error=''
            self.conversion_ms=(time.perf_counter()-start)*1000
            self.record(kind='cloud',source_ns=stamp,points=len(records),conversion_ms=self.conversion_ms)
        except ValueError as error:self.rejects+=1;self.error=str(error);self.record(kind='rejected_cloud',reason=self.error)

    def fix(self,msg):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        lat,lon,alt=msg.latitude,msg.longitude,msg.altitude;origin=self.gnss_origin
        variances=[msg.position_covariance[i] for i in (0,4,8)]
        sigma=math.sqrt(max(variances)) if min(variances)>0 else math.nan
        position=[math.radians(lon-origin[1])*6378137*math.cos(math.radians(origin[0])),
                  math.radians(lat-origin[0])*6378137,alt-origin[2]]
        status=msg.status.status
        if isinstance(status,bytes):status=int.from_bytes(status,'little',signed=True)
        admitted=(msg.header.frame_id=='gnss_link' and msg.position_covariance_type!=0 and self.gnss.observe(stamp,position,sigma,status))
        if admitted and status!=2:
            admitted=False;self.gnss.reason='float fix withheld: backend factors assume fixed RTK accuracy'
        self.gnss_counts['accepted' if admitted else 'rejected']+=1
        self.record(kind='gnss',source_s=stamp,admitted=admitted,reason=self.gnss.reason)
        if not admitted:return
        output=self.PVT();output.valid_fix=True;output.fix_type=3;output.diff_soln=True
        output.carr_soln=2 if status==2 else 1
        output.latitude=lat;output.longitude=lon;output.altitude=alt;output.height_msl=alt
        output.h_acc=output.v_acc=sigma;output.num_sv=16;output.p_dop=1.
        # NavSatFix supplies position only. Declare finite-difference velocity
        # derived from admitted observations; never insert a fake zero velocity.
        if self.previous_fix is not None:
            previous_stamp,previous_position,previous_sigma=self.previous_fix
            dt=stamp-previous_stamp
            velocity=[(position[i]-previous_position[i])/dt for i in range(3)]
            output.vel_e,output.vel_n,output.vel_d=velocity[0],velocity[1],-velocity[2]
            output.vel_acc=math.hypot(sigma,previous_sigma)/dt
        else:
            self.previous_fix=(stamp,position,sigma)
            self.record(kind='gnss_velocity',source_s=stamp,reason='first admitted position; velocity unavailable')
            return
        self.previous_fix=(stamp,position,sigma)
        self.record(kind='gnss_velocity',source_s=stamp,enu=velocity,sigma_mps=output.vel_acc,method='admitted-position finite difference')
        # Declared epoch maps small simulation times into valid GPS week/tow.
        gps_seconds=1609459200.+stamp-315964800.+18.
        output.time.week=int(gps_seconds//604800);output.time.tow=gps_seconds%604800
        self.gnss_pub.publish(output)

    def diagnostics(self):
        status=DiagnosticStatus(name=self.prefix+'/inputs',level=DiagnosticStatus.WARN if self.error else DiagnosticStatus.OK,
            message=self.error or 'source timing preserved')
        status.values=[KeyValue(key=k,value=str(v)) for k,v in {'clouds':self.clouds,'rejected_clouds':self.rejects,
            'conversion_ms':self.conversion_ms,**self.gnss_counts,'gnss_reason':self.gnss.reason}.items()]
        self.diag.publish(DiagnosticArray(status=[status]))

    def destroy_node(self):
        if self.trace:self.trace.close()
        super().destroy_node()

def main():
    rclpy.init();node=None
    try:node=AlgorithmSensorAdapter();rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
