#!/usr/bin/python3
"""Owned, isolated, sensor-only replay with source-time pose/resource reports."""
import argparse,json,os,signal,subprocess,time
from pathlib import Path
import numpy as np
import psutil
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2,PointCloud,Imu,Image,CameraInfo,NavSatFix
from visualization_msgs.msg import MarkerArray
from uav_lab_experiments.loop_evidence import loop_edge_count
from uav_lab_experiments.backend_contract import input_topics,consumed_input_group,normalized_pose,trajectory_report
from uav_lab_tools.datasets import load_dataset,file_hash,check_replay_domain
from uav_lab_localization.benchmark import read_dataset
from backend_configs import write_config
from backend_launch import core_commands
from backend_provenance import save_provenance,verify_runtime_artifacts
from bootstrap_backends import verify_sources
from vins_map import snapshot_vins_map,graph_body_trajectory
from uav_lab_experiments.benchmark_report import resource_summary

ROOT=Path(__file__).resolve().parents[1]
OWNED_OUTPUT=None

class PoseObserver(Node):
    def __init__(self,backend,calibration,directory):
        super().__init__('backend_evaluator')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
        self.imu=calibration['lidar'] if backend=='lio_sam' else calibration['imu']
        self.backend=backend;self.clock=None;self.rows=[];self.latencies=[];self.expected=[];self.failures=[]
        self.pose_time_offset=calibration.get('pose_time_offset_s',{}).get(backend,0.)
        self.scan_end_offset=0. if backend in ('glim','lio_sam') else calibration.get('scan_end_offset_s',.1)
        self.counts={};self.pose_stream=(directory/'estimate.tum').open('x')
        self.normalized=0;self.normalizer_states={};self.normalizer_events=[]
        self.loop_edges=0;self.keyframes=0;self.global_rows=[]
        self.create_subscription(Clock,'/clock',self.tick,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/uav001/backends/'+backend+'/raw_odometry',self.pose,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/uav001/backends/'+backend+'/odometry',self.normalized_pose,qos_profile_sensor_data)
        private='/uav001/backends/'+backend
        self.create_subscription(Odometry,private+'/global_odometry',self.global_pose,qos_profile_sensor_data)
        if backend=='vins_fusion':
            self.create_subscription(PointCloud,private+'/keyframe_point',lambda msg:setattr(self,'keyframes',self.keyframes+1),qos_profile_sensor_data)
        if backend in ('vins_fusion','lio_sam'):
            self.create_subscription(MarkerArray,private+('/pose_graph' if backend=='vins_fusion' else '/loop_constraints'),
                lambda msg:setattr(self,'loop_edges',max(self.loop_edges,loop_edge_count(self.backend,msg))),qos_profile_sensor_data)
        from diagnostic_msgs.msg import DiagnosticArray
        self.create_subscription(DiagnosticArray,'/uav001/backends/'+backend+'/diagnostics',self.normalizer,10)
        from std_msgs.msg import String
        self.core_states=[]
        self.create_subscription(String,'/uav001/backends/'+backend+'/core_state',lambda msg:self.core_states.append({'source_s':self.clock,'state':json.loads(msg.data)}),10)
        for topic,kind in [('/uav001/lidar/points',PointCloud2),('/uav001/imu/data',Imu),('/uav001/camera/image_raw',Image),('/uav001/camera/camera_info',CameraInfo),('/uav001/gnss/fix',NavSatFix)]:
            self.create_subscription(kind,topic,lambda msg,t=topic:self.input(t,msg),qos_profile_sensor_data)

    def tick(self,msg):self.clock=msg.clock.sec+msg.clock.nanosec/1e9

    def normalized_pose(self,msg):self.normalized+=1

    def global_pose(self,msg):
        p,q=msg.pose.pose.position,msg.pose.pose.orientation
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9+self.pose_time_offset
        try:self.global_rows.append(normalized_pose(stamp,[p.x,p.y,p.z],[q.x,q.y,q.z,q.w],self.imu))
        except ValueError:pass

    def normalizer(self,msg):
        for status in msg.status:
            self.normalizer_states[status.message]=self.normalizer_states.get(status.message,0)+1
            values={value.key:value.value for value in status.values}
            signature=(status.message,values.get('reason',''))
            if not self.normalizer_events or signature!=tuple(self.normalizer_events[-1]['signature']):
                self.normalizer_events.append({'signature':signature,'source_s':self.clock,'values':values})

    def input(self,topic,msg):
        self.counts[topic]=self.counts.get(topic,0)+1
        if topic.endswith('/image_raw') and self.backend in ('orb_slam3','vins_fusion'):
            self.expected.append(msg.header.stamp.sec+msg.header.stamp.nanosec/1e9)
        elif topic.endswith('/points') and self.backend not in ('orb_slam3','vins_fusion'):
            self.expected.append(msg.header.stamp.sec+msg.header.stamp.nanosec/1e9+self.scan_end_offset)

    def pose(self,msg):
        try:
            stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9+self.pose_time_offset
            p,q=msg.pose.pose.position,msg.pose.pose.orientation
            row=normalized_pose(stamp,[p.x,p.y,p.z],[q.x,q.y,q.z,q.w],self.imu)
            if self.rows and stamp<=self.rows[-1][0]:raise ValueError('duplicate or regressed estimator timestamp')
            self.rows.append(row);self.pose_stream.write(' '.join(f'{x:.9f}' for x in row)+'\n');self.pose_stream.flush()
            if self.clock is not None:self.latencies.append(self.clock-stamp)
        except ValueError as error:self.failures.append(str(error))

    def destroy_node(self):
        self.pose_stream.close();super().destroy_node()


def stop_owned(process):
    if process is None:return
    try:os.killpg(process.pid,signal.SIGINT)
    except ProcessLookupError:return
    try:process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        try:os.killpg(process.pid,signal.SIGTERM)
        except ProcessLookupError:pass
        try:process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            try:os.killpg(process.pid,signal.SIGKILL)
            except ProcessLookupError:pass
            process.wait()


def log_tail(path,limit=20000):
    with Path(path).open('rb') as stream:
        stream.seek(max(0,Path(path).stat().st_size-limit));return stream.read().decode(errors='replace')


def configure_replay_transport(configuration,environment):
    """Own the large-sensor transport before either observer or child initializes ROS."""
    profile=Path(configuration).resolve()/'fastdds-local.xml'
    profile.write_bytes((ROOT/'configs/fastdds-local.xml').read_bytes())
    transport={'ROS_LOCALHOST_ONLY':'0','RMW_IMPLEMENTATION':'rmw_fastrtps_cpp',
        'FASTRTPS_DEFAULT_PROFILES_FILE':str(profile)}
    os.environ.update(transport)
    return dict(environment,**transport)


def main():
    global OWNED_OUTPUT
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend',choices=['glim','fast_lio2','fast_livo2','fast_livo2_rtk','lio_sam','orb_slam3','vins_fusion'],required=True)
    parser.add_argument('--dataset',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--rate',type=float,default=1.);parser.add_argument('--duration',type=float,default=0.)
    parser.add_argument('--public-config',type=Path);parser.add_argument('--position-only',action='store_true')
    parser.add_argument('--vins-map',type=Path,help='load an owned copy of a previously saved VINS pose_graph.txt')
    parser.add_argument('--debug-core',action='store_true',help='capture a native crash backtrace; diagnostic run only')
    args=parser.parse_args()
    if not .1<=args.rate<=1. or not 0<=args.duration<=1800:parser.error('rate .1..1, duration 0..1800')
    if args.vins_map and (args.backend!='vins_fusion' or args.public_config):parser.error('VINS map loading requires local VINS configuration')
    if args.output.exists():parser.error('output exists; evidence is immutable')
    domain=int(os.environ.get('ROS_DOMAIN_ID','42'))
    active=[]
    for p in (ROOT/'.runtime').glob('*/manifest.json'):
        if (p.parent/'ready').exists():active.append(int(json.loads(p.read_text())['environment']['ROS_DOMAIN_ID']))
    check_replay_domain(domain,active)
    dataset=args.dataset.resolve();output=args.output.resolve();metadata=load_dataset(dataset)
    output.mkdir(parents=True);OWNED_OUTPUT=output;(output/'ros-logs').mkdir()
    lock=json.loads((ROOT/'dependencies/backends.lock.json').read_text())
    verify_sources(ROOT,lock,lock['backends'][args.backend]['repositories'])
    import sys
    provenance=save_provenance(ROOT,lock,args.backend,output,sys.argv)
    env=dict(os.environ,ROS_LOG_DIR=str(output/'ros-logs'),OMP_NUM_THREADS='2',OPENBLAS_NUM_THREADS='1')
    if args.public_config:
        import shutil
        shutil.copytree(args.public_config,output/'configuration');config=output/'configuration/parameters.yaml'
    else:config=write_config(args.backend,metadata['calibration'],output/'configuration',output.name)
    env=configure_replay_transport(output/'configuration',env)
    map_snapshot=None
    if args.vins_map:
        map_snapshot=snapshot_vins_map(args.vins_map,output/'configuration/pose_graph')
        path=output/'configuration/algorithm.yaml'
        path.write_text(path.read_text().replace('load_previous_pose_graph: 0','load_previous_pose_graph: 1'))
        (output/'map-input-snapshot.json').write_text(json.dumps(map_snapshot,indent=2)+'\n')
    truth=[]
    for topic,msg,_,_ in read_dataset(dataset,['/uav001/ground_truth/odometry']):
        p,q=msg.pose.pose.position,msg.pose.pose.orientation
        truth.append([msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,p.x,p.y,p.z,q.x,q.y,q.z,q.w])
    truth=np.asarray(truth);np.savetxt(output/'truth.tum',truth,fmt='%.9f')
    (output/'calibration.json').write_text(json.dumps(metadata['calibration'],indent=2)+'\n')
    prefix=ROOT/'.deps/backends'/args.backend/'install'
    source=json.loads((prefix/(args.backend+'-build-manifest.json')).read_text())
    verify_runtime_artifacts(source)
    binary=Path(source['binary']).resolve()
    if not binary.is_relative_to(prefix.resolve()):raise ValueError('backend binary outside private prefix')
    if file_hash(binary)!=source['binary_sha256']:raise ValueError('backend binary changed since build')
    (output/'build-manifest.json').write_text(json.dumps(source,indent=2)+'\n')
    import yaml
    qos={t:{'reliability':'reliable','durability':'volatile','history':'keep_last','depth':512 if t.endswith('/data') else 8} for t in input_topics(args.backend)}
    (output/'playback-qos.yaml').write_text(yaml.safe_dump(qos))
    children=[];logs=[];node=None;error='';resources=[];exit_code=None;start=time.monotonic();batch=None
    def launch(command,name,stdin=None):
        log=(output/(name+'.log')).open('w');logs.append(log)
        child=subprocess.Popen([str(v) for v in command],stdout=log,stderr=subprocess.STDOUT,stdin=stdin,env=env,start_new_session=True,cwd=ROOT)
        children.append(child);return child
    def resource_sample(phase,origin):
        samples=[]
        for child in children:
            if 'bag' in child.args and 'play' in child.args:continue
            try:
                p=psutil.Process(child.pid);c=p.cpu_times();samples.append({'pid':child.pid,'rss':p.memory_info().rss,'cpu_s':c.user+c.system})
            except psutil.NoSuchProcess:pass
        resources.append({'wall_s':time.monotonic()-origin,'source_s':node.clock,'phase':phase,'processes':samples})
    try:
        rclpy.init();node=PoseObserver(args.backend,metadata['calibration'],output)
        if any(node.count_publishers(t) for t in input_topics(args.backend)):raise ValueError('replay domain has occupied input topics')
        private='/uav001/backends/'+args.backend
        if args.backend=='fast_livo2':
            launch(['/opt/ros/humble/lib/demo_nodes_cpp/parameter_blackboard','--ros-args','--params-file',output/'configuration/camera.yaml'],'camera-parameters')
        if not args.public_config:
            if args.backend in ('fast_lio2','fast_livo2','fast_livo2_rtk','lio_sam'):
                launch(['/usr/bin/python3','-m','uav_lab_experiments.algorithm_sensor_node','--ros-args','-p','backend:='+args.backend,
                        '-p','trace_file:='+str(output/'input-trace.jsonl'),'-p','calibration:='+str(output/'calibration.json')],'input-adapter')
            launch(['/usr/bin/python3','-m','uav_lab_experiments.backend_node','--ros-args','-p','backend:='+args.backend,
                    '-p','calibration:='+str(output/'calibration.json')],'normalizer')
        backend=None
        for i,command in enumerate(core_commands(args.backend,binary,output/'configuration')):
            if args.debug_core:command=['gdb','-q','-batch','-ex','run','-ex','thread apply all bt','--args',*command]
            backend=launch(command,'backend' if i==0 else 'backend-'+str(i),subprocess.PIPE)
        discovery=time.monotonic()+3.
        while time.monotonic()<discovery:
            rclpy.spin_once(node,timeout_sec=.05)
            if any(p.poll() is not None for p in children):raise RuntimeError('backend or input adapter exited during startup')
        if args.backend in ('orb_slam3','vins_fusion'):
            deadline=time.monotonic()+90.
            while not any(event['state'].get('input_ready') for event in node.core_states):
                rclpy.spin_once(node,timeout_sec=.05)
                if any(p.poll() is not None for p in children):raise RuntimeError('visual core exited before input readiness')
                if time.monotonic()>deadline:raise RuntimeError('visual input readiness timed out')
        player=launch(['ros2','bag','play',dataset/'bag','--rate',str(args.rate),'--read-ahead-queue-size','100',
                       '--disable-keyboard-controls','--qos-profile-overrides-path',output/'playback-qos.yaml','--topics',*input_topics(args.backend)],'player',subprocess.DEVNULL)
        playback_start=time.monotonic();deadline=playback_start+(args.duration or 1900)/args.rate+20
        first_clock=None;last_resource=0.
        while player.poll() is None:
            rclpy.spin_once(node,timeout_sec=.01)
            if first_clock is None and node.clock is not None:first_clock=node.clock
            for child in children[:-1]:
                if child.poll() is not None:raise RuntimeError('owned backend dependency exited '+str(child.args[0])+': '+str(child.returncode))
            if args.duration and first_clock is not None and node.clock-first_clock>=args.duration:stop_owned(player);break
            if time.monotonic()>deadline:raise RuntimeError('playback timeout')
            if time.monotonic()-last_resource>=.5:
                last_resource=time.monotonic();resource_sample('replay',playback_start)
        exit_code=player.returncode
        drain=time.monotonic()+3
        while time.monotonic()<drain:rclpy.spin_once(node,timeout_sec=.05)
        if args.backend=='fast_livo2_rtk':
            backend.stdin.write(b'\n');backend.stdin.flush()
            (output/'rtk-batch-trigger.json').write_text(json.dumps({'after_replay':True,'source_s':node.clock})+'\n')
            drain=time.monotonic()+180
            completed=False
            while time.monotonic()<drain:
                rclpy.spin_once(node,timeout_sec=.05)
                if backend.poll() is not None:break
                if '[Offline Optimization] Finished.' in log_tail(output/'backend.log'):completed=True;break
                if time.monotonic()-last_resource>=.5:
                    last_resource=time.monotonic();resource_sample('rtk_batch',playback_start)
            batch={'completed':completed,'backend_exit_code':backend.poll(),'reason':'completed' if completed else 'batch timeout or backend exit'}
            if completed:
                optimized=output/'configuration/rtk-output/TUM/opt_trajectory_after.txt'
                global_truth=truth.copy()
                from scipy.spatial.transform import Rotation
                global_truth[:,1:4]+=Rotation.from_quat(truth[:,4:]).apply(metadata['calibration']['gnss']['xyz'])
                try:batch['quality']=trajectory_report(np.loadtxt(optimized,ndmin=2),global_truth,[t for t in node.expected if t>=node.expected[0]+3.])
                except (OSError,ValueError) as failure:batch.update(completed=False,reason=str(failure))
                batch['pose_target']='GNSS antenna before final lever-arm conversion; separate from local control pose'
    except (ValueError,RuntimeError,KeyboardInterrupt) as failure:error=str(failure) or 'interrupted'
    finally:
        for child in reversed(children):stop_owned(child)
        for log in logs:log.close()
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    expected=[t for t in node.expected if t>=node.expected[0]+3] if node and node.expected else []
    quality=trajectory_report(node.rows if node else [],truth,expected,not args.position_only)
    global_quality=trajectory_report(node.global_rows,truth,expected,not args.position_only) if node and node.global_rows else None
    pose_graph=None
    if args.backend=='vins_fusion':
        path=output/'configuration/pose_graph/pose_graph.txt'
        try:
            rows,cross_session=graph_body_trajectory(path,metadata['calibration']['imu'],map_snapshot['source_keyframes'] if map_snapshot else 0)
            np.savetxt(output/'optimized-keyframe-body.tum',rows,fmt='%.9f')
            pose_graph={'saved':True,'current_keyframes':len(rows),'cross_session_constraints':cross_session,
                'quality':trajectory_report(rows,truth,[row[0] for row in rows],not args.position_only),
                'scope':'optimized keyframes only; separate from online odometry and dense coverage'}
        except (OSError,ValueError) as failure:pose_graph={'saved':False,'reason':str(failure)}
    latency=np.asarray(node.latencies if node else [])
    report={'schema':1,'backend':args.backend,'success':not error and not (node and node.failures) and quality['passed'] and (batch is None or batch.get('completed',False) and batch.get('quality',{}).get('passed',False)),
        'error':error,'quality':quality,'dataset':str(dataset),'dataset_sha256':metadata['bag_sha256'],
        'input_topics':input_topics(args.backend),'input_group':consumed_input_group(args.backend,metadata['calibration']),
        'input_counts':node.counts if node else {},'invalid_poses':node.failures if node else [],
        'normalized_output_count':node.normalized if node else 0,'normalizer_states':node.normalizer_states if node else {},
        'normalizer_events':node.normalizer_events if node else [],'rtk_batch':batch,'debug_core':args.debug_core,
        'core_states':node.core_states if node else [],
        'pose_graph':pose_graph,'map_input_snapshot':map_snapshot,'global_quality':global_quality,
        'capability_observations':{'keyframe_point_messages':node.keyframes if node else 0,
            'confirmed_loop_edges':node.loop_edges if node else 0,'global_pose_messages':len(node.global_rows) if node else 0,
            'loop_closure':'observed' if node and node.loop_edges else 'not observed in this run',
            'relocalization':'cross-session constraints observed' if pose_graph and pose_graph.get('cross_session_constraints',0)>0 else 'not verified by this replay; no control qualification'},
        'playback_rate':args.rate,'player_exit_code':exit_code,'requested_source_duration_s':args.duration or None,
        'transport':{'profile':'configuration/fastdds-local.xml',
            'profile_sha256':file_hash(output/'configuration/fastdds-local.xml'),
            'ROS_LOCALHOST_ONLY':env['ROS_LOCALHOST_ONLY'],'RMW_IMPLEMENTATION':env['RMW_IMPLEMENTATION']},
        'latency_source_s':{'p50':float(np.median(latency)),'p95':float(np.quantile(latency,.95)),'min':float(latency.min())} if len(latency) else None,
        'runtime_s':time.monotonic()-start,'resources':resources,'resource_summary':resource_summary(resources),
        'implementation_sha256':provenance['implementation_sha256'],
        'scope':'offline replay; no realtime or flight qualification','build':source}
    (output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    if node and node.global_rows:np.savetxt(output/'global-estimate.tum',node.global_rows,fmt='%.9f')
    print(json.dumps({k:v for k,v in report.items() if k not in ('resources','build')},indent=2))
    return 0 if report['success'] else 1

if __name__=='__main__':
    try:code=main()
    except (OSError,ValueError,RuntimeError) as error:
        failure={'success':False,'reason':str(error),'scope':'preflight failed; no qualification'}
        if OWNED_OUTPUT:
            with (OWNED_OUTPUT/'failure.json').open('x') as stream:json.dump(failure,stream,indent=2)
        print(json.dumps(failure));code=1
    raise SystemExit(code)
