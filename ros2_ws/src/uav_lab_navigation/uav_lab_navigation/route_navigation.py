"""Sequential semantic goals joined before terminal rest by the selected real planner."""
import copy,time,uuid
import numpy as np
from .continuous_navigation import reference,checked_curve,replan_join,NavigationCanceled
from uav_lab_bridge.trajectory_interface import trajectory_goal


def checkpoint_join(curve,elapsed,source_ns,target,lead):
    join=replan_join(curve,elapsed,source_ns,lead)
    if join is None or np.linalg.norm(join[1].position-np.asarray(target))>.2:
        return None
    if np.linalg.norm(join[1].velocity)<1e-6:return None
    return join


def validate_controls(controls):
    if not 1<=len(controls)<=128:raise ValueError('one to128 route controls required')
    positions=[]
    for control in controls:
        p,q=control.pose.position,control.pose.orientation
        if (control.header.frame_id!='odom' or not np.isfinite([p.x,p.y,p.z,q.x,q.y,q.z,q.w]).all()
                or abs(q.x)>1e-6 or abs(q.y)>1e-6 or abs(q.z*q.z+q.w*q.w-1)>.001 or not .5<=p.z<=4.5):
            raise ValueError('finite odom route controls, unit yaw and bounded altitude required')
        positions.append(np.array([p.x,p.y,p.z]))
    if any(np.linalg.norm(a-b)<1e-5 for a,b in zip(positions,positions[1:])):
        raise ValueError('successive route controls must differ')
    return positions


def local_target(collision,initial,target):
    """Use a measured in-window subgoal for a distant fixed semantic control.

    Never replace a blocked in-window control or mark a semantic control as
    reached. The selected planner still admits the complete resulting curve.
    """
    initial,target=np.asarray(initial,dtype=float),np.asarray(target,dtype=float)
    index=collision.index(target)
    if all(0<=v<n for v,n in zip(index,collision.free.shape)):return target,False
    delta=target-initial;distance=float(np.linalg.norm(delta))
    if not np.isfinite(delta).all() or distance<1e-6:raise ValueError('invalid distant route control')
    direction=delta/distance;upper=collision.lower+np.asarray(collision.free.shape)*collision.resolution
    bound=distance
    for axis in range(3):
        if abs(direction[axis])>1e-9:
            edge=upper[axis]-collision.resolution if direction[axis]>0 else collision.lower[axis]+collision.resolution
            bound=min(bound,(edge-initial[axis])/direction[axis])
    for advance in np.arange(bound,.4,-collision.resolution):
        point=initial+advance*direction
        if collision.point_clear(point):return point,True
    raise ValueError('distant route has no observed admissible local target')


def route_curve(node,control,initial,planning_timeout=None):
    with node.map_lock:collision=node.grid.snapshot(node.envelope)
    p=control.pose.position;target,partial=local_target(collision,initial.position,[p.x,p.y,p.z])
    local=copy.deepcopy(control);local.pose.position.x,local.pose.position.y,local.pose.position.z=map(float,target)
    curve,snapshot=checked_curve(node,local,initial,planning_timeout=planning_timeout)
    return curve,snapshot,target,partial


def navigate_route(node,handle):
    from uav_lab_interfaces.action import NavigateRoute
    owners=[];epoch=node.accepted_epoch;index=0;replans=0;handoff=None;local_handoffs=0
    try:
        targets=validate_controls(handle.request.controls);node.check_motion(epoch)
        if not node.trajectory_client.wait_for_server(timeout_sec=3):raise RuntimeError('trajectory adapter unavailable')
        msg,initial=reference(node)
        if np.linalg.norm(initial.velocity)>1e-6 or np.linalg.norm(initial.acceleration)>1e-6:
            raise RuntimeError('new route requires stationary reference')
        curve,collision,leg_target,partial=route_curve(node,handle.request.controls[0],initial)
        request=trajectory_goal(curve,uuid.uuid4().hex,epoch=epoch)

        def submit(request):
            node.check_motion(epoch)
            if handle.is_cancel_requested:raise NavigationCanceled('route canceled; constrained hold')
            pending=node.trajectory_client.send_goal_async(request)
            try:accepted=node.future_result(pending,node.acceptance_timeout_s)
            except RuntimeError:
                node.failure='route acceptance uncertain; restart lab'
                def cancel_late(future):
                    try:
                        late=future.result()
                        if late and late.accepted:late.cancel_goal_async()
                    except Exception as error:node.record('late_route_cancel_failure',reason=str(error))
                pending.add_done_callback(cancel_late);raise
            if not accepted.accepted:raise RuntimeError('continuous route trajectory rejected')
            future=accepted.get_result_async();owners.append((accepted,future))
            return accepted,future

        leg,future=submit(request);checked=collision.version
        distance=sum(float(np.linalg.norm(a-b)) for a,b in zip([initial.position,*targets[:-1]],targets))
        deadline=time.monotonic()+min(3600.,distance*30+len(targets)*20+60)
        while True:
            node.check_motion(epoch)
            if handle.is_cancel_requested:raise NavigationCanceled('route canceled; constrained hold')
            if time.monotonic()>deadline:raise RuntimeError('route deadline exceeded')
            if future.done():
                result=node.future_result(future).result
                if not result.success:raise RuntimeError(result.reason)
                if partial or index!=len(targets)-1:raise RuntimeError('route reached an intermediate stop; continuous handoff failed')
                node.record('route_result',success=True,controls=len(targets),replans=replans)
                handle.succeed();return NavigateRoute.Result(success=True,reason='continuous route final target reached')
            msg,state=reference(node)
            with node.map_lock:updated=node.grid.snapshot(node.envelope) if node.grid.version!=checked else None
            if updated is not None:checked=updated.version
            # Protect both owners throughout a pending future handoff. A map
            # change cannot escape collision checks during scheduling latency.
            if handoff is not None and msg.trajectory_id==handoff[0]:
                if updated is not None and (not handoff[1].collision_free(updated,start_time=min(msg.elapsed,handoff[2]),
                    end_time=handoff[2]) or not curve.collision_free(updated)):
                    raise RuntimeError('pending route handoff unsafe; constrained hold')
            elif msg.trajectory_id==request.trajectory_id:
                handoff=None;elapsed=min(curve.duration,msg.elapsed)
                source_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
                lead=node.replan_lead_s
                advance=checkpoint_join(curve,elapsed,source_ns,leg_target,lead) if partial or index<len(targets)-1 else None
                unsafe=updated is not None and not curve.collision_free(updated,start_time=elapsed)
                if advance is not None or unsafe:
                    snapshot=updated
                    if snapshot is None:
                        with node.map_lock:snapshot=node.grid.snapshot(node.envelope)
                    join=advance if advance is not None else replan_join(curve,elapsed,source_ns,lead)
                    if join is None or not curve.collision_free(snapshot,start_time=elapsed,end_time=join[0]):
                        raise RuntimeError('route cannot reserve an admissible prefix; constrained hold')
                    next_index=index+1 if advance is not None and not partial else index
                    if advance is None:
                        replans+=1
                        if replans>node.config['max_replans']:raise RuntimeError('route obstacle replan limit exceeded')
                    join_elapsed,initial,start_ns=join
                    replacement,collision,next_target,next_partial=route_curve(node,handle.request.controls[next_index],initial,planning_timeout=lead-.3)
                    node.check_motion(epoch)
                    if start_ns-node.get_clock().now().nanoseconds<150_000_000:
                        raise RuntimeError('route planner missed scheduling window; constrained hold')
                    previous_id,previous_curve=request.trajectory_id,curve
                    new_request=trajectory_goal(replacement,uuid.uuid4().hex,start_ns,replaces=previous_id,epoch=epoch)
                    leg,future=submit(new_request)
                    handoff=(previous_id,previous_curve,join_elapsed)
                    curve,request=replacement,new_request;checked=collision.version
                    if advance is not None and not partial:
                        node.record('route_control_scheduled',index=index,target=targets[index].tolist(),
                            reference_error_m=float(np.linalg.norm(initial.position-targets[index])),
                            join_velocity=initial.velocity.tolist(),join_acceleration=initial.acceleration.tolist(),
                            starts_at_ns=start_ns,trajectory_id=request.trajectory_id)
                    elif advance is not None:
                        local_handoffs+=1
                        if local_handoffs>512:raise RuntimeError('bounded local route handoff limit exceeded')
                        node.record('route_local_handoff',current_control=index,target=leg_target.tolist(),
                            starts_at_ns=start_ns,trajectory_id=request.trajectory_id)
                    leg_target,partial=next_target,next_partial
                    index=next_index
            feedback=NavigateRoute.Feedback(phase='MOVING',current_control=index,replans=replans)
            feedback.current_pose.header.frame_id='odom';feedback.current_pose.pose=node.current
            handle.publish_feedback(feedback);time.sleep(.02)
    except (ValueError,RuntimeError) as error:
        for candidate,future in reversed(owners):
            if future.done():continue
            try:node.stop_leg(candidate)
            except RuntimeError as failure:
                node.failure='route cancellation uncertain; restart lab';node.record('route_cancel_failure',reason=str(failure))
        node.record('route_result',success=False,reason=str(error),current_control=index,replans=replans)
        handle.canceled() if isinstance(error,NavigationCanceled) else handle.abort()
        return NavigateRoute.Result(success=False,reason=str(error))
    finally:
        with node.motion_lock:node.busy=False
