"""Observed-space A* routes executed as C2 curves; replacements keep p/v/a.

The legacy navigation mode is retained for V0.4 regression. Workbench mode uses
this pipeline, never the upstream planners' fake-drone executors.
"""
import math
import time
import uuid
import numpy as np
from uav_lab_interfaces.action import Navigate
from uav_lab_bridge.continuous_trajectory import Trajectory, State
from uav_lab_bridge.trajectory_interface import trajectory_goal


class NavigationCanceled(RuntimeError):
    pass


def reference(node):
    msg = node.reference
    if msg is None or time.monotonic() - node.reference_at > .5:
        raise RuntimeError('fresh trajectory reference required')
    state = State(np.array(msg.position), np.array(msg.velocity), np.array(msg.acceleration),
                  yaw=msg.yaw, yaw_rate=msg.yaw_rate)
    return msg, state


def checked_curve(node, goal, initial):
    if getattr(node,'planner_backend','astar')!='astar':
        curve,collision=node.make_curve(goal,initial)
        if not curve.collision_free(collision):raise RuntimeError('NO_SAFE_CONTINUOUS_TRAJECTORY')
        node.record('continuous_plan',trajectory=curve.to_dict(),backend=node.planner_backend,
                    map_version=collision.version,**node.archive_plan(collision))
        sampled=[curve.sample(t).position for t in np.arange(0.,curve.duration,.1)]
        sampled.append(curve.sample(curve.duration).position)
        node.path_pub.publish(node.as_path(sampled))
        return curve,collision
    collision, result = node.make_plan(goal, start_override=initial.position.tolist())
    if not result.success:
        raise RuntimeError(result.reason)
    points = np.asarray(result.points)
    points[0] = initial.position
    goal_yaw = 2 * math.atan2(goal.pose.orientation.z, goal.pose.orientation.w)
    goal_yaw = initial.yaw + math.atan2(math.sin(goal_yaw - initial.yaw), math.cos(goal_yaw - initial.yaw))
    if np.linalg.norm(points[-1] - initial.position) < 1e-5:
        curve = Trajectory.stop(initial)
    else:
        # A rejected curve does not fall back to unchecked linear motion or stops.
        curve = Trajectory.generate(points, initial=initial,
                                    yaws=np.linspace(initial.yaw, goal_yaw, len(points)))
    if not curve.collision_free(collision):
        raise RuntimeError('NO_SAFE_CONTINUOUS_TRAJECTORY')
    node.record('continuous_plan', trajectory=curve.to_dict(), points=points.tolist(),
                map_version=collision.version, **node.archive_plan(collision))
    sampled = [curve.sample(t).position for t in np.arange(0., curve.duration, .1)]
    sampled.append(curve.sample(curve.duration).position)
    node.path_pub.publish(node.as_path(sampled))
    return curve, collision


def navigate_continuous(node, handle):
    legs, leg, replans = [], None, 0
    epoch = node.accepted_epoch
    try:
        node.check_motion(epoch)
        if not node.trajectory_client.wait_for_server(timeout_sec=3):
            raise RuntimeError('trajectory adapter unavailable')
        msg, initial = reference(node)
        if np.linalg.norm(initial.velocity) > 1e-6 or np.linalg.norm(initial.acceleration) > 1e-6:
            raise RuntimeError('new navigation request requires stationary reference')
        curve, collision = checked_curve(node, handle.request.goal, initial)
        request = trajectory_goal(curve, uuid.uuid4().hex, epoch=epoch)

        def submit(request):
            node.check_motion(epoch)
            if handle.is_cancel_requested:
                raise NavigationCanceled('canceled; constrained hold commanded')
            pending = node.trajectory_client.send_goal_async(request)
            try:
                accepted = node.future_result(pending, node.acceptance_timeout_s)
            except RuntimeError:
                node.failure = 'trajectory acceptance uncertain; restart lab'
                def cancel_late(future):
                    try:
                        accepted = future.result()
                        if accepted and accepted.accepted:
                            accepted.cancel_goal_async()
                    except Exception as error:
                        node.record('late_trajectory_cancel_failure', reason=str(error))
                pending.add_done_callback(cancel_late)
                raise
            if not accepted.accepted:
                raise RuntimeError('continuous trajectory rejected')
            legs.append(accepted)
            return accepted

        leg = submit(request)
        result_future = leg.get_result_async()
        deadline = time.monotonic() + curve.duration * 3 + 20
        checked = collision.version
        while not result_future.done():
            node.check_motion(epoch)
            if handle.is_cancel_requested:
                raise NavigationCanceled('canceled; constrained hold commanded')
            if time.monotonic() > deadline:
                raise RuntimeError('continuous trajectory timeout')
            msg, state = reference(node)
            with node.map_lock:
                updated = node.grid.snapshot(node.envelope) if node.grid.version != checked else None
            if updated is not None and msg.trajectory_id == request.trajectory_id:
                checked = updated.version
                elapsed = min(curve.duration, msg.elapsed)
                if not curve.collision_free(updated, start_time=elapsed):
                    replans += 1
                    if replans > node.config['max_replans']:
                        raise RuntimeError('continuous replan limit exceeded')
                    # Keep the active curve only when the entire scheduling prefix
                    # remains admissible. Otherwise stop inside observed free space.
                    join_elapsed = min(curve.duration, elapsed + .6)
                    if join_elapsed > elapsed and curve.collision_free(updated, start_time=elapsed, end_time=join_elapsed):
                        initial = curve.sample(join_elapsed)
                        replacement, collision = checked_curve(node, handle.request.goal, initial)
                        node.check_motion(epoch)
                        stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
                        start_ns = stamp_ns + int(round((join_elapsed - elapsed) * 1e9))
                        request = trajectory_goal(replacement, uuid.uuid4().hex, start_ns,
                                                  replaces=request.trajectory_id, epoch=epoch)
                        leg = submit(request)
                        curve = replacement
                        result_future = leg.get_result_async()
                        checked = collision.version
                        deadline = time.monotonic() + curve.duration * 3 + 20
                    else:
                        stop = Trajectory.stop(state)
                        if not stop.collision_free(updated):
                            node.failure = 'no admissible constrained stop; restart lab'
                            raise RuntimeError(node.failure)
                        node.stop_leg(leg)
                        stop_deadline = time.monotonic() + stop.duration * 3 + 5
                        while True:
                            node.check_motion(epoch)
                            msg, initial = reference(node)
                            if np.linalg.norm(initial.velocity) < 1e-6 and np.linalg.norm(initial.acceleration) < 1e-6:
                                break
                            if time.monotonic() > stop_deadline:
                                raise RuntimeError('constrained stop timeout')
                            time.sleep(.05)
                        curve, collision = checked_curve(node, handle.request.goal, initial)
                        request = trajectory_goal(curve, uuid.uuid4().hex, epoch=epoch)
                        leg = submit(request)
                        result_future = leg.get_result_async()
                        checked = collision.version
                        deadline = time.monotonic() + curve.duration * 3 + 20
                    node.record('continuous_replan', count=replans, trajectory_id=request.trajectory_id)
            feedback = Navigate.Feedback(phase='MOVING', replans=replans)
            feedback.current_pose.header.frame_id = 'odom'
            feedback.current_pose.pose = node.current
            handle.publish_feedback(feedback)
            time.sleep(.05)
        result = node.future_result(result_future).result
        if not result.success:
            raise RuntimeError(result.reason)
        node.check_motion(epoch)
        handle.succeed()
        node.record('navigation_result', success=True, replans=replans, continuous=True)
        return Navigate.Result(success=True, reason='continuous navigation target reached')
    except (ValueError, RuntimeError) as error:
        # Include the pre-replacement owner until its result confirms the handoff.
        for candidate in reversed(legs):
            try:
                node.stop_leg(candidate)
            except RuntimeError as stop_error:
                node.failure = 'trajectory cancellation uncertain; restart lab'
                node.record('cancel_failure', reason=str(stop_error))
        node.record('navigation_result', success=False, reason=str(error), continuous=True)
        handle.canceled() if isinstance(error, NavigationCanceled) else handle.abort()
        return Navigate.Result(success=False, reason=str(error))
    finally:
        with node.motion_lock:
            node.busy = False
