"""Actual ROS Action transport: the new mode sends one C2 curve, not stop legs."""
import time
import threading
import numpy as np
import pytest
rclpy = pytest.importorskip('rclpy')
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from uav_lab_interfaces.action import Navigate, ExecuteTrajectory
from uav_lab_interfaces.msg import TrajectoryReference
from uav_lab_navigation.node import NavigationNode
from uav_lab_navigation.cli import pose
from uav_lab_navigation.planner import Plan
from uav_lab_bridge.trajectory_interface import trajectory_from_goal


@pytest.mark.parametrize(('mode','backend'), [(m,'astar') for m in ('success','source_loss','operator_hold','cancel','reject')]
                         + [('success',b) for b in ('ego','fast_planner','gcopter')])
def test_continuous_navigation_sends_one_collision_checked_curve(monkeypatch, tmp_path, isolated_ros_domain, mode, backend):
    from pathlib import Path
    if backend!='astar' and not (Path('.deps/ego').exists()):pytest.skip('native planner sources absent')
    monkeypatch.delenv('LAB_RUN_DIR', raising=False)
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros'))
    rclpy.init(domain_id=isolated_ros_domain)
    nav = NavigationNode()
    nav.continuous_enabled = True
    nav.planner_backend = backend
    if backend!='astar':
        assert hasattr(nav,'make_curve'),'native planner is not connected to the navigation Action'
        nav.run=tmp_path
    nav.grid.score[:] = -4
    nav.grid.seen[:] = True
    nav.current = pose([0, 0, 2]).pose
    nav.current_at = time.monotonic()
    nav.flight_status = {'phase': 'HOLDING', 'armed': 'True', 'offboard': 'True', 'operator_generation': '0'}
    nav.flight_at = time.monotonic()
    healthy = [True]
    monkeypatch.setattr(nav, 'report', lambda **kw: {'ready': healthy[0],
                        'state': 'READY' if healthy[0] else 'FAILED', 'reason': 'controlled source'})
    monkeypatch.setattr(nav, 'make_plan', lambda goal, start_override=None:
        (nav.grid.snapshot(nav.envelope), Plan(True, 'PLANNED', [[0., 0., 2.], [0., 1., 2.], [1., 1., 2.]])))
    reference = TrajectoryReference(position=[0., 0., 2.], velocity=[0., 0., 0.], acceleration=[0., 0., 0.])
    reference.header.frame_id = 'odom'
    nav.on_reference(reference)
    adapter = rclpy.create_node('continuous_flight_double')
    reference_pub=adapter.create_publisher(TrajectoryReference,'/uav001/trajectory/reference',10)
    def refresh_double():
        reference_pub.publish(reference)
        nav.current_at=nav.flight_at=time.monotonic()
    adapter.create_timer(.05,refresh_double)
    client_node = rclpy.create_node('continuous_navigation_test')
    requests = []
    def execute(handle):
        requests.append(handle.request)
        trajectory = trajectory_from_goal(handle.request)
        midpoint = (trajectory.segments[0].sample(trajectory.segments[0].duration) if backend=='astar'
                    else trajectory.sample(trajectory.duration/2))
        assert np.linalg.norm(midpoint.velocity) > .05
        if mode != 'success':
            if mode == 'source_loss':
                healthy[0] = False
            if mode == 'operator_hold':
                nav.flight_status['operator_generation'] = '1'
            deadline = time.monotonic() + 2
            while not handle.is_cancel_requested and time.monotonic() < deadline:
                time.sleep(.01)
            if handle.is_cancel_requested:
                handle.canceled()
                return ExecuteTrajectory.Result(success=False, reason='cancel confirmed')
        handle.succeed()
        return ExecuteTrajectory.Result(success=True, reason='transport confirmed')
    server = ActionServer(adapter, ExecuteTrajectory, '/uav001/execute_trajectory',
                          execute_callback=execute, callback_group=ReentrantCallbackGroup(),
                          goal_callback=lambda r: GoalResponse.REJECT if mode == 'reject' else GoalResponse.ACCEPT,
                          cancel_callback=lambda h: CancelResponse.ACCEPT)
    executor = MultiThreadedExecutor(num_threads=4)
    for node in (nav, adapter, client_node):
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    client = ActionClient(client_node, Navigate, '/uav001/navigation/navigate')
    def wait(future):
        deadline = time.monotonic() + 5
        while not future.done() and time.monotonic() < deadline:
            time.sleep(.01)
        assert future.done()
        return future.result()
    try:
        assert client.wait_for_server(timeout_sec=5)
        handle = wait(client.send_goal_async(Navigate.Goal(goal=pose([1, 1, 2]))))
        assert handle.accepted
        if mode == 'cancel':
            wait(handle.cancel_goal_async())
        result = wait(handle.get_result_async())
        if mode == 'success':
            assert result.result.success
        else:
            assert not result.result.success
        assert len(requests) <= 1
        if requests:
            assert requests[0].navigation and requests[0].navigation_epoch == 0
    finally:
        executor.shutdown(timeout_sec=3)
        thread.join(3)
        client.destroy()
        server.destroy()
        for node in (client_node, adapter, nav):
            node.destroy_node()
        rclpy.shutdown()


def test_pending_goto_handoff_rechecks_old_prefix_and_cancels_both_owners(monkeypatch):
    """A new obstacle appears after acceptance, while the old reference owns flight."""
    from concurrent.futures import Future
    from types import SimpleNamespace
    from uav_lab_bridge.continuous_trajectory import State, Trajectory
    from uav_lab_navigation.occupancy import CollisionMap
    import uav_lab_navigation.continuous_navigation as module

    curve=Trajectory.generate([[0.,0.,2.],[3.,0.,2.]],initial=State(np.array([0.,0.,2.])))
    join=curve.sample(3.6)
    replacement=Trajectory.generate([join.position,join.position+[1.,1.,0.]],initial=join)
    lower=np.array([-.1,-.1,1.9]);free=np.ones((500,300,20),dtype=bool)
    original=CollisionMap(.01,lower,free.copy(),1)
    far=free.copy();far[tuple(np.floor((curve.sample(curve.duration).position-lower)/.01).astype(int))]=False
    changed=CollisionMap(.01,lower,far.copy(),2)
    near=far.copy();near[tuple(np.floor((curve.sample(3.3).position-lower)/.01).astype(int))]=False
    pending_map=CollisionMap(.01,lower,near,3)
    assert curve.collision_free(original)
    assert not curve.collision_free(changed,start_time=3.)
    assert curve.collision_free(changed,start_time=3.,end_time=3.6)
    assert replacement.collision_free(pending_map)
    assert not curve.collision_free(pending_map,start_time=3.1,end_time=3.6)

    ref=TrajectoryReference(position=[0.,0.,2.],velocity=[0.,0.,0.],acceleration=[0.,0.,0.])
    ref.header.stamp.sec=10
    results=[];owners=[];stopped=[];records=[];plans=iter([(curve,original),(replacement,changed)])
    node=SimpleNamespace(accepted_epoch=0,acceptance_timeout_s=1.,replan_lead_s=.6,
        config={'max_replans':3},reference=ref,reference_at=time.monotonic(),
        map_lock=threading.Lock(),motion_lock=threading.Lock(),envelope=np.zeros(3),busy=True,
        current=pose([0,0,2]).pose,check_motion=lambda epoch:None,
        future_result=lambda future,*args:future.result(),stop_leg=lambda owner:stopped.append(owner),
        record=lambda name,**kwargs:records.append((name,kwargs)),
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=10_000_000_000)))
    node.grid=SimpleNamespace(version=2,snapshot=lambda envelope:changed if node.grid.version==2 else pending_map)
    def submit(request):
        result=Future();results.append(result)
        owner=SimpleNamespace(accepted=True,get_result_async=lambda:result);owners.append(owner)
        if len(owners)==1:
            ref.trajectory_id=request.trajectory_id;ref.elapsed=3.
            state=curve.sample(3.)
            ref.position=state.position.tolist();ref.velocity=state.velocity.tolist();ref.acceleration=state.acceleration.tolist()
        else:
            assert request.replaces_id==ref.trajectory_id
            node.grid.version=3;ref.elapsed=3.1
        accepted=Future();accepted.set_result(owner);return accepted
    node.trajectory_client=SimpleNamespace(wait_for_server=lambda **kwargs:True,send_goal_async=submit)
    monkeypatch.setattr(module,'checked_curve',lambda *args,**kwargs:next(plans))
    monkeypatch.setattr(module.time,'sleep',lambda duration:None)
    outcome=[]
    def feedback(msg):
        if len(results)==2:
            # Allow exactly one further map update before the result completes.
            if outcome:results[-1].set_result(SimpleNamespace(result=ExecuteTrajectory.Result(success=True)))
            else:outcome.append('waiting')
    handle=SimpleNamespace(request=Navigate.Goal(goal=pose([3,0,2])),is_cancel_requested=False,
        publish_feedback=feedback,succeed=lambda:outcome.append('success'),abort=lambda:outcome.append('aborted'))
    result=module.navigate_continuous(node,handle)
    assert not result.success, 'old executing prefix became blocked before replacement activation'
    assert 'handoff unsafe' in result.reason
    assert len(stopped)==2 and all(owner in stopped for owner in owners)
    assert outcome[-1]=='aborted' and not node.busy
