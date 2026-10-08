"""Source-time registered obstacle map in continuous odom; no ground-truth input."""
import json
import hashlib
from collections import deque
from concurrent.futures import ThreadPoolExecutor,ProcessPoolExecutor,TimeoutError as WorkerTimeout
from concurrent.futures.process import BrokenProcessPool
import multiprocessing
import os
from pathlib import Path
import threading
import time
import uuid
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.action import ActionServer,ActionClient,GoalResponse,CancelResponse
from rclpy.node import Node
from rclpy.clock import Clock,ClockType
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup,ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
from diagnostic_msgs.msg import DiagnosticArray,DiagnosticStatus,KeyValue
from geometry_msgs.msg import PoseStamped,TransformStamped
from nav_msgs.msg import Odometry,Path as NavPath
from sensor_msgs.msg import PointCloud2,PointField
from uav_lab_interfaces.srv import PlanPath
from uav_lab_interfaces.action import Navigate,ExecuteFlight,ExecuteTrajectory
from uav_lab_interfaces.msg import TrajectoryReference
from uav_lab_localization.ingress import validated_scan,RejectedScan
from uav_lab_localization.benchmark import stamp
from .occupancy import VoxelMap
from .poses import PoseHistory,register_scan,remove_self_returns
from .planner import plan


class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation',namespace='uav001')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
        self.declare_parameter('continuous_trajectory', False)
        self.continuous_enabled = bool(self.get_parameter('continuous_trajectory').value)
        self.reference = None
        self.reference_at = 0.
        root=Path(os.environ.get('LAB_ROOT',Path.cwd()));run=Path(os.environ['LAB_RUN_DIR']) if os.environ.get('LAB_RUN_DIR') else None
        config_file=run/'configuration/navigation.json' if run and (run/'configuration/navigation.json').exists() else root/'configs/navigation.json'
        self.config=json.loads(config_file.read_text())
        calibration_file=run/'configuration/calibration.json' if run and (run/'configuration/calibration.json').exists() else root/'configs/navigation-sensors.json'
        self.calibration=json.loads(calibration_file.read_text())
        self.extrinsic=np.eye(4);self.extrinsic[:3,3]=self.calibration['lidar']['xyz']
        self.extrinsic[:3,:3]=Rotation.from_euler('xyz',self.calibration['lidar']['rpy']).as_matrix()
        self.grid=VoxelMap(self.config['resolution_m'],self.config['lower'],self.config['upper'])
        self.envelope=np.asarray(self.config['body_halfsize_m'])+self.config['clearance_m']
        self.history=PoseHistory();self.alignment=None;self.pending_clouds=deque(maxlen=8)
        self.map_at,self.map_stamp,self.pose_at,self.quality_at=0.,None,0.,0.
        self.quality_ready=False;self.failure='';self.was_ready=False
        self.current=None;self.current_at=0.;self.flight_status={};self.flight_at=0.
        self.map_lock=threading.RLock();self.data_lock=threading.RLock()
        self.motion_lock=threading.RLock();self.busy=False;self.accepted_epoch=None;self.leg_timeout_s=75.;self.acceptance_timeout_s=5.
        self.map_group=MutuallyExclusiveCallbackGroup();self.action_group=ReentrantCallbackGroup()
        self.trace=(run/'navigation.jsonl').open('a',buffering=1) if run else None
        self.run=run;self.saved_at=0.;self.storage=ThreadPoolExecutor(max_workers=1);self.save_future=None
        self.session_id=uuid.uuid4().hex;self.archive_lock=threading.Lock()
        self.planning_worker=ProcessPoolExecutor(max_workers=1,mp_context=multiprocessing.get_context('spawn'))
        self.planning_lock=threading.Lock()
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.occupied_pub=self.create_publisher(PointCloud2,'navigation/occupied',qos)
        self.path_pub=self.create_publisher(NavPath,'navigation/path',qos)
        self.diagnostic_pub=self.create_publisher(DiagnosticArray,'navigation/diagnostics',10)
        self.create_subscription(TransformStamped,'localization/control_alignment',self.on_alignment,qos)
        self.create_subscription(Odometry,'localization/odometry',self.on_pose,20)
        self.create_subscription(PointCloud2,'localization/input/points',self.on_cloud,qos_profile_sensor_data)
        self.create_subscription(DiagnosticArray,'localization/diagnostics',self.on_quality,10)
        self.create_subscription(Odometry,'odometry',self.on_flight_pose,10)
        self.create_subscription(DiagnosticArray,'diagnostics',self.on_flight_status,10)
        self.create_service(PlanPath,'navigation/plan',self.plan_service,callback_group=self.action_group)
        from std_srvs.srv import Trigger
        self.create_service(Trigger,'navigation/abort',self.abort_service,callback_group=self.action_group)
        self.flight_client=ActionClient(self,ExecuteFlight,'execute_flight',callback_group=self.action_group)
        self.trajectory_client=ActionClient(self,ExecuteTrajectory,'execute_trajectory',callback_group=self.action_group)
        self.create_subscription(TrajectoryReference,'trajectory/reference',self.on_reference,10)
        self.server=ActionServer(self,Navigate,'navigation/navigate',execute_callback=self.navigate,
            goal_callback=self.navigation_goal,cancel_callback=lambda handle:CancelResponse.ACCEPT,callback_group=self.action_group)
        self.create_timer(self.config['map_period_s'],self.map_cycle,callback_group=self.map_group,clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.create_timer(.1,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def record(self,event,**values):
        if self.trace:self.trace.write(json.dumps({'event':event,'wall_monotonic':time.monotonic(),**values},allow_nan=False)+'\n')

    def on_alignment(self,msg):
        try:
            if msg.header.frame_id!='lio_odom' or msg.child_frame_id!='odom':raise ValueError('invalid frozen alignment frame')
            t,q=msg.transform.translation,msg.transform.rotation
            position=np.array([t.x,t.y,t.z]);quaternion=np.array([q.x,q.y,q.z,q.w])
            if not np.isfinite(position).all() or not np.isfinite(quaternion).all() or abs(np.linalg.norm(quaternion)-1)>.001:
                raise ValueError('invalid frozen alignment values')
            matrix=np.eye(4);matrix[:3,3]=position;matrix[:3,:3]=Rotation.from_quat(quaternion).as_matrix()
            with self.data_lock:
                if self.alignment is not None and not np.allclose(self.alignment,matrix,atol=1e-7):raise ValueError('frozen alignment changed')
                self.alignment=matrix
        except ValueError as error:self.failure=str(error)

    def on_pose(self,msg):
        try:
            if msg.header.frame_id!='lio_odom' or msg.child_frame_id!='lio_base_link':raise ValueError('wrong navigation pose frame')
            p,q=msg.pose.pose.position,msg.pose.pose.orientation
            with self.data_lock:
                self.history.add(stamp(msg)/1e9,[p.x,p.y,p.z],[q.x,q.y,q.z,q.w]);self.pose_at=time.monotonic()
        except ValueError as error:self.failure=str(error)

    def on_quality(self,msg):
        for status in msg.status:
            if status.name=='uav001/localization':
                values={v.key:v.value for v in status.values}
                self.quality_ready=status.level==DiagnosticStatus.OK and values.get('ready')=='True';self.quality_at=time.monotonic()

    def on_cloud(self,msg):
        with self.data_lock:self.pending_clouds.append(msg)

    def on_flight_pose(self,msg):
        if msg.header.frame_id=='odom' and msg.child_frame_id=='base_link':self.current=msg.pose.pose;self.current_at=time.monotonic()

    def on_flight_status(self,msg):
        for status in msg.status:
            if status.name=='uav001/flight':self.flight_status={'phase':status.message,**{v.key:v.value for v in status.values}};self.flight_at=time.monotonic()

    def map_cycle(self):
        begun=time.monotonic()
        if self.failure:return
        # DDS may discover sensor publishers before /clock. A zero simulated
        # clock is uninitialized; wait without integrating or declaring ready.
        # Once a map exists, ordinary freshness checks must still fail closed.
        if self.map_stamp is None and not self.was_ready and self.get_clock().now().nanoseconds == 0:
            return
        with self.data_lock:
            if self.alignment is None or not self.quality_ready:return
            for msg in reversed(self.pending_clouds):
                source=stamp(msg)/1e9
                if self.map_stamp is not None and source<=self.map_stamp:continue
                try:pose=self.history.at(source);break
                except ValueError:continue
            else:return
            alignment=self.alignment.copy()
        try:
            points=remove_self_returns(validated_scan(msg),self.extrinsic,self.config['body_halfsize_m'])
            age=self.get_clock().now().nanoseconds/1e9-source
            if age<-.1 or age>self.config['source_timeout_s']:raise ValueError('map source time stale or in future')
            origin,registered=register_scan(points,pose,alignment,self.extrinsic)
            body=np.linalg.inv(alignment)@pose
            with self.map_lock:
                self.grid.integrate(origin,registered)
                self.grid.observe_body(body[:3,3],self.config['body_halfsize_m'])
                occupied=self.grid.occupied_points()
                self.grid.source_stamp=source;self.map_stamp=source;self.map_at=time.monotonic()
            self.publish_occupied(occupied,msg.header.stamp)
            self.record('map',stamp=source,version=self.grid.version,occupied_cells=len(occupied),compute_s=time.monotonic()-begun,
                source_age=self.get_clock().now().nanoseconds/1e9-source)
            if self.run and time.monotonic()-self.saved_at>5 and (self.save_future is None or self.save_future.done()):
                self.saved_at=time.monotonic()
                with self.map_lock:
                    saved={'score':self.grid.score.copy(),'lower':self.grid.lower.copy(),'resolution':self.grid.resolution,
                        'version':self.grid.version,'source_stamp':source,'origin':origin.copy()}
                self.save_future=self.storage.submit(self.archive,saved,registered.copy(),body.copy())
        except RejectedScan:return
        except (ValueError,TypeError) as error:self.failure=str(error)

    def archive(self,saved,endpoints,body):
        np.savez_compressed(self.run/'navigation-grid.npz',**saved)
        raw=self.run/'navigation-observations';raw.mkdir(exist_ok=True)
        if len(list(raw.glob('*.npz')))<30:
            np.savez_compressed(raw/f'{saved["source_stamp"]:.6f}.npz',origin=saved['origin'],endpoints=endpoints,body=body,source_stamp=saved['source_stamp'])

    def archive_plan(self,collision):
        if not self.run:return {}
        with self.archive_lock:
            try:
                directory=self.run/'navigation-plans'/self.session_id;directory.mkdir(parents=True,exist_ok=True)
                path=directory/f'map-v{collision.version}.npz'
                try:
                    with path.open('xb') as handle:
                        np.savez_compressed(handle,free=collision.free,lower=collision.lower,resolution=collision.resolution,
                            map_version=collision.version,source_stamp=collision.source_stamp,envelope=self.envelope,
                            configuration=json.dumps(self.config,sort_keys=True))
                except FileExistsError:pass
                return {'map_artifact':str(path.relative_to(self.run)),'map_sha256':hashlib.sha256(path.read_bytes()).hexdigest()}
            except OSError as error:
                self.failure='plan map archive failed; restart lab';raise ValueError(self.failure) from error

    def report(self,now=None):
        now=time.monotonic() if now is None else now
        source_age=None if self.map_stamp is None else self.get_clock().now().nanoseconds/1e9-self.map_stamp
        healthy=(not self.failure and self.alignment is not None and self.quality_ready and now-self.quality_at<=.75
            and now-self.pose_at<=.5 and self.map_stamp is not None and -.1<=source_age<=self.config['source_timeout_s']
            and now-self.map_at<=self.config['source_timeout_s'])
        if self.was_ready and not healthy and not self.failure:
            self.failure='navigation source/quality expired; restart lab'
            self.record('source_failure',quality_ready=self.quality_ready,quality_age=now-self.quality_at,pose_age=now-self.pose_at,
                map_receipt_age=now-self.map_at,source_age=source_age)
        if healthy:self.was_ready=True
        return {'ready':bool(healthy),'state':'FAILED' if self.failure else 'READY' if healthy else 'INITIALIZING',
            'reason':self.failure,'source_age':source_age,'source_stamp':self.map_stamp,'map_version':self.grid.version,'frame':'odom'}

    def publish_occupied(self,points,source):
        values=np.asarray(points,dtype='<f4');msg=PointCloud2();msg.header.frame_id='odom';msg.header.stamp=source
        msg.height=1;msg.width=len(values);msg.point_step=12;msg.row_step=12*len(values);msg.is_dense=True
        msg.fields=[PointField(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate('xyz')];msg.data=values.tobytes()
        self.occupied_pub.publish(msg)

    def as_path(self,points):
        path=NavPath();path.header.frame_id='odom';path.header.stamp=self.get_clock().now().to_msg()
        for values in points:
            pose=PoseStamped();pose.header=path.header;pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=map(float,values);pose.pose.orientation.w=1.;path.poses.append(pose)
        return path

    def on_reference(self, msg):
        if msg.header.frame_id == 'odom' and np.isfinite([*msg.position, *msg.velocity, *msg.acceleration, msg.yaw, msg.yaw_rate]).all():
            self.reference, self.reference_at = msg, time.monotonic()

    def make_plan(self,goal,start_override=None):
        if not self.report()['ready']:raise ValueError('navigation map/pose not ready: '+self.failure)
        if self.current is None or time.monotonic()-self.current_at>.5:raise ValueError('fresh flight pose required')
        if goal.header.frame_id!='odom':raise ValueError('navigation goal frame must be odom')
        p,q=goal.pose.position,goal.pose.orientation
        if not np.isfinite([p.x,p.y,p.z,q.x,q.y,q.z,q.w]).all() or abs(q.x)>1e-6 or abs(q.y)>1e-6 or abs(q.z*q.z+q.w*q.w-1)>.001:
            raise ValueError('finite position and unit yaw quaternion required')
        if not .5<=p.z<=4.5:raise ValueError('navigation altitude outside 0.5..4.5m')
        start=[self.current.position.x,self.current.position.y,self.current.position.z] if start_override is None else start_override
        with self.map_lock:collision=self.grid.snapshot(self.envelope)
        if not self.planning_lock.acquire(blocking=False):raise ValueError('planning worker busy')
        try:
            pending=self.planning_worker.submit(plan,collision,start,[p.x,p.y,p.z],self.config['max_expansions'],self.config['planning_timeout_s'])
            result=pending.result(timeout=self.config['planning_timeout_s']+3.)
        except (WorkerTimeout,BrokenProcessPool,RuntimeError) as error:
            self.failure='planning worker failed: '+str(error)+'; restart lab';raise ValueError(self.failure) from error
        finally:self.planning_lock.release()
        return collision,result

    def plan_service(self,request,response):
        try:
            collision,result=self.make_plan(request.goal)
            response.success,response.reason,response.map_version=result.success,result.reason,collision.version
            response.path=self.as_path(result.points)
            if result.success:self.path_pub.publish(response.path)
            self.record('plan',success=result.success,reason=result.reason,points=result.points,expanded=result.expanded,map_version=collision.version,**self.archive_plan(collision))
        except ValueError as error:response.success,response.reason=False,str(error)
        return response

    def tick(self):
        report=self.report();msg=DiagnosticArray();msg.header.stamp=self.get_clock().now().to_msg()
        msg.status=[DiagnosticStatus(name='uav001/navigation',hardware_id='bounded-3d',level=DiagnosticStatus.OK if report['ready'] else DiagnosticStatus.ERROR if self.failure else DiagnosticStatus.WARN,
            message=report['state'],values=[KeyValue(key=k,value=str(v)) for k,v in report.items()])]
        self.diagnostic_pub.publish(msg)

    def navigation_goal(self,request):
        with self.motion_lock:
            if self.busy:return GoalResponse.REJECT
            try:
                self.check_motion()
                p,q=request.goal.pose.position,request.goal.pose.orientation
                if request.goal.header.frame_id!='odom' or not np.isfinite([p.x,p.y,p.z,q.x,q.y,q.z,q.w]).all():return GoalResponse.REJECT
                if abs(q.x)>1e-6 or abs(q.y)>1e-6 or abs(q.z*q.z+q.w*q.w-1)>.001 or not .5<=p.z<=4.5:return GoalResponse.REJECT
                self.accepted_epoch=int(self.flight_status.get('operator_generation','-1'));self.busy=True
                return GoalResponse.ACCEPT
            except (ValueError,RuntimeError):return GoalResponse.REJECT

    def check_motion(self,epoch=None):
        if not self.report()['ready']:raise RuntimeError('navigation source failed; restart lab')
        state=self.flight_status
        if time.monotonic()-self.flight_at>1.5:raise RuntimeError('flight status expired')
        if epoch is not None and int(state.get('operator_generation','-1'))!=epoch:raise RuntimeError('interrupted by operator HOLD/LAND; no continuation')
        if state.get('armed')!='True' or state.get('offboard')!='True' or state.get('phase') not in ('HOLDING','MOVING'):
            raise RuntimeError('navigation requires armed Offboard hover')

    def future_result(self,future,timeout=5.):
        deadline=time.monotonic()+timeout
        while rclpy.ok() and not future.done() and time.monotonic()<deadline:time.sleep(.01)
        if not future.done():raise RuntimeError('flight adapter response timeout')
        if future.exception():raise RuntimeError(str(future.exception()))
        return future.result()

    def stop_leg(self,leg):
        if leg is None:return
        self.future_result(leg.cancel_goal_async(),3.)
        # Cancellation acknowledgment precedes actual hold. Wait for the adapter result.
        self.future_result(leg.get_result_async(),3.)

    def submit_leg(self,request):
        pending=self.flight_client.send_goal_async(request)
        try:return self.future_result(pending,self.acceptance_timeout_s)
        except RuntimeError:
            self.failure='flight acceptance uncertain; restart lab'
            def cancel_late(future):
                try:
                    leg=future.result()
                    if leg and leg.accepted:leg.cancel_goal_async()
                except Exception as error:self.record('late_cancel_failure',reason=str(error))
            pending.add_done_callback(cancel_late)
            raise

    def abort_service(self,request,response):
        self.failure='operator navigation abort; restart lab'
        response.success,response.message=True,self.failure
        return response

    def navigate(self,handle):
        if self.continuous_enabled:
            from .continuous_navigation import navigate_continuous
            return navigate_continuous(self, handle)
        outcome=Navigate.Result();leg=None;replans=0
        try:
            epoch=self.accepted_epoch;self.check_motion(epoch)
            if not self.flight_client.wait_for_server(timeout_sec=3):raise RuntimeError('flight adapter unavailable')
            while rclpy.ok():
                if handle.is_cancel_requested:
                    handle.canceled();return Navigate.Result(success=False,reason='canceled; hold commanded')
                self.check_motion(epoch)
                collision,result=self.make_plan(handle.request.goal)
                self.check_motion(epoch)
                if not result.success:raise RuntimeError(result.reason)
                self.path_pub.publish(self.as_path(result.points))
                self.record('navigation_plan',map_version=collision.version,reason=result.reason,points=result.points,replans=replans,**self.archive_plan(collision))
                changed=False
                for target in result.points[1:]:
                    self.check_motion(epoch)
                    if handle.is_cancel_requested:break
                    with self.map_lock:
                        latest=self.grid.snapshot(self.envelope)
                    p=self.current.position
                    if not latest.segment_clear([p.x,p.y,p.z],target):changed=True;break
                    request=ExecuteFlight.Goal();request.operation=1;request.navigation=True;request.navigation_epoch=epoch
                    request.target.header.frame_id='odom';request.target.pose.position.x,request.target.pose.position.y,request.target.pose.position.z=map(float,target)
                    request.target.pose.orientation=handle.request.goal.pose.orientation
                    leg=self.submit_leg(request)
                    if not leg.accepted:leg=None;raise RuntimeError('flight leg rejected')
                    result_future=leg.get_result_async();deadline=time.monotonic()+self.leg_timeout_s;checked=latest.version
                    while not result_future.done():
                        self.check_motion(epoch)
                        if handle.is_cancel_requested:break
                        if time.monotonic()>deadline:raise RuntimeError('flight leg timeout')
                        with self.map_lock:
                            if self.grid.version!=checked:
                                updated=self.grid.snapshot(self.envelope);checked=updated.version
                                p=self.current.position
                                if not updated.segment_clear([p.x,p.y,p.z],target):changed=True;break
                        feedback=Navigate.Feedback();feedback.phase='MOVING';feedback.replans=replans
                        feedback.current_pose.header.frame_id='odom';feedback.current_pose.pose=self.current
                        handle.publish_feedback(feedback);time.sleep(.05)
                    if changed or handle.is_cancel_requested:
                        self.stop_leg(leg);leg=None;break
                    response=self.future_result(result_future).result;leg=None
                    if not response.success:raise RuntimeError('flight leg failed: '+response.reason)
                    self.record('leg_complete',target=target,epoch=epoch)
                if handle.is_cancel_requested:
                    handle.canceled();return Navigate.Result(success=False,reason='canceled; hold commanded')
                if not changed:
                    self.check_motion(epoch);handle.succeed();self.record('navigation_result',success=True,replans=replans)
                    return Navigate.Result(success=True,reason='navigation target reached')
                replans+=1;self.record('replan',count=replans)
                if replans>self.config['max_replans']:raise RuntimeError('replan limit exceeded; hold retained')
            raise RuntimeError('navigation shutting down')
        except (ValueError,RuntimeError) as error:
            if leg:
                try:self.stop_leg(leg)
                except RuntimeError as stop_error:
                    self.failure='flight cancellation uncertain; restart lab';self.record('cancel_failure',reason=str(stop_error))
            outcome.success,outcome.reason=False,str(error);self.record('navigation_result',success=False,reason=str(error),replans=replans)
            handle.abort();return outcome
        finally:
            with self.motion_lock:self.busy=False

    def destroy_node(self):
        self.server.destroy();self.flight_client.destroy();self.trajectory_client.destroy()
        self.planning_worker.shutdown(wait=True,cancel_futures=True)
        self.storage.shutdown(wait=True)
        if self.trace:self.trace.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args);node=NavigationNode();executor=MultiThreadedExecutor(num_threads=4);executor.add_node(node)
    try:executor.spin()
    except KeyboardInterrupt:pass
    finally:
        executor.shutdown();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
