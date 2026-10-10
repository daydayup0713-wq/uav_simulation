"""Actual ROS action transport against a controlled flight adapter, no fake rclpy."""
import threading
import time
import numpy as np
import pytest
rclpy=pytest.importorskip('rclpy')
from rclpy.action import ActionServer,ActionClient,GoalResponse,CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from action_msgs.msg import GoalStatus
from uav_lab_interfaces.action import Navigate,ExecuteFlight
from uav_lab_navigation.node import NavigationNode
from uav_lab_navigation.planner import Plan
from uav_lab_navigation.cli import pose


def wait(future,timeout=8):
    end=time.monotonic()+timeout
    while not future.done() and time.monotonic()<end:time.sleep(.01)
    assert future.done(),'ROS action timeout';return future.result()


@pytest.fixture
def rig(monkeypatch,isolated_ros_domain):
    monkeypatch.delenv('LAB_RUN_DIR',raising=False);rclpy.init(domain_id=isolated_ros_domain)
    nav=NavigationNode();adapter=rclpy.create_node('flight_double');client=rclpy.create_node('nav_test')
    nav.grid.score[:]=-4;nav.grid.seen[:]=True
    nav.current=pose([0,0,2]).pose;nav.current_at=time.monotonic()
    nav.flight_status={'phase':'HOLDING','armed':'True','offboard':'True','operator_generation':'0'};nav.flight_at=time.monotonic()
    monkeypatch.setattr(nav,'report',lambda **kw:{'ready':True,'state':'READY','reason':'','source_stamp':0.,'frame':'odom'})
    def planning(goal):
        collision=nav.grid.snapshot(nav.envelope)
        return collision,Plan(True,'PLANNED',[[0.,0.,2.],[0.,1.,2.],[1.,1.,2.]],2)
    monkeypatch.setattr(nav,'make_plan',planning)
    control={'mode':'success','delay':.05,'goals':[],'cancelled':0}
    def goal(request):
        if control['mode']=='delay_ack':time.sleep(.35)
        return GoalResponse.REJECT if control['mode']=='reject' else GoalResponse.ACCEPT
    def execute(handle):
        control['goals'].append(handle.request);end=time.monotonic()+control['delay']
        while time.monotonic()<end:
            if handle.is_cancel_requested:
                control['cancelled']+=1;handle.canceled();return ExecuteFlight.Result(success=False,reason='canceled; hold commanded')
            time.sleep(.01)
        nav.current=handle.request.target.pose
        handle.succeed();return ExecuteFlight.Result(success=True,reason='target reached')
    server=ActionServer(adapter,ExecuteFlight,'/uav001/execute_flight',goal_callback=goal,cancel_callback=lambda h:CancelResponse.ACCEPT,
        execute_callback=execute,callback_group=ReentrantCallbackGroup())
    executor=MultiThreadedExecutor(num_threads=6)
    for node in (nav,adapter,client):executor.add_node(node)
    worker=threading.Thread(target=executor.spin,daemon=True);worker.start()
    action=ActionClient(client,Navigate,'/uav001/navigation/navigate')
    try:
        assert action.wait_for_server(timeout_sec=8)
        yield nav,control,action
    finally:
        executor.shutdown(timeout_sec=3);worker.join(3);action.destroy();server.destroy()
        for node in (client,adapter,nav):node.destroy_node()
        rclpy.shutdown()


def submit(action):return wait(action.send_goal_async(Navigate.Goal(goal=pose([1,1,2]))))


def test_multileg_motion_is_guarded_and_confirmed(rig):
    nav,control,action=rig;handle=submit(action);assert handle.accepted
    result=wait(handle.get_result_async())
    assert result.status==GoalStatus.STATUS_SUCCEEDED and result.result.success
    assert len(control['goals'])==2 and all(g.navigation and g.navigation_epoch==0 for g in control['goals'])


def test_duplicate_and_cancel_do_not_continue(rig):
    nav,control,action=rig;control['delay']=1.
    handle=submit(action);assert handle.accepted
    duplicate=submit(action);assert not duplicate.accepted
    wait(handle.cancel_goal_async());result=wait(handle.get_result_async())
    assert result.status==GoalStatus.STATUS_CANCELED and not result.result.success
    assert len(control['goals'])<=1


@pytest.mark.parametrize('interrupt',['HOLDING','LANDING'])
def test_operator_hold_or_land_prevents_next_leg(rig,interrupt):
    nav,control,action=rig;control['delay']=.4;handle=submit(action)
    end=time.monotonic()+3
    while not control['goals'] and time.monotonic()<end:time.sleep(.01)
    nav.flight_status.update(phase=interrupt,operator_generation='1')
    result=wait(handle.get_result_async())
    assert not result.result.success and len(control['goals'])==1
    assert 'operator' in result.result.reason


def test_flight_rejection_is_failure(rig):
    nav,control,action=rig;control['mode']='reject';handle=submit(action)
    result=wait(handle.get_result_async());assert not result.result.success and 'rejected' in result.result.reason


def test_changed_obstacle_cancels_and_replans_with_bound(rig):
    nav,control,action=rig;control['delay']=.4;handle=submit(action)
    end=time.monotonic()+3
    while not control['goals'] and time.monotonic()<end:time.sleep(.01)
    with nav.map_lock:
        nav.grid.score[:]=4;nav.grid.version+=1
    result=wait(handle.get_result_async())
    assert not result.result.success and control['cancelled']>=1
    assert len(control['goals'])<=4 and 'replan' in result.result.reason.lower()


def test_unreachable_goal_does_not_send_motion(rig,monkeypatch):
    nav,control,action=rig
    monkeypatch.setattr(nav,'make_plan',lambda goal:(nav.grid.snapshot(nav.envelope),Plan(False,'NO_PATH')))
    handle=submit(action);result=wait(handle.get_result_async())
    assert not result.result.success and result.result.reason=='NO_PATH' and not control['goals']


def test_flight_timeout_cancels_and_has_no_next_leg(rig):
    nav,control,action=rig;control['delay']=1.;nav.leg_timeout_s=.15
    handle=submit(action);result=wait(handle.get_result_async())
    assert not result.result.success and 'timeout' in result.result.reason
    assert control['cancelled']==1 and len(control['goals'])==1


def test_source_failure_aborts_without_recovery_motion(rig,monkeypatch):
    nav,control,action=rig;control['delay']=1.;handle=submit(action)
    end=time.monotonic()+3
    while not control['goals'] and time.monotonic()<end:time.sleep(.01)
    monkeypatch.setattr(nav,'report',lambda **kw:{'ready':False,'state':'FAILED','reason':'stale source'})
    result=wait(handle.get_result_async())
    assert not result.result.success and len(control['goals'])==1
    assert control['cancelled']==1


def test_hold_between_acceptance_and_execution_does_not_reauthorize(rig):
    nav,control,action=rig
    def accepted(handle):
        nav.flight_status['operator_generation']='1';handle.execute()
    nav.server.register_handle_accepted_callback(accepted)
    handle=submit(action);result=wait(handle.get_result_async())
    assert not result.result.success and not control['goals']
    assert 'operator' in result.result.reason


def test_late_flight_acceptance_is_canceled_and_failure_latched(rig):
    nav,control,action=rig;control['mode']='delay_ack';control['delay']=1.;nav.acceptance_timeout_s=.1
    handle=submit(action);result=wait(handle.get_result_async())
    assert not result.result.success and nav.failure
    time.sleep(.6)
    assert len(control['goals'])<=1 and control['cancelled']==len(control['goals'])


def test_unconfirmed_stop_latches_navigation_failure(rig,monkeypatch):
    nav,control,action=rig;control['delay']=1.;handle=submit(action)
    end=time.monotonic()+3
    while not control['goals'] and time.monotonic()<end:time.sleep(.01)
    nav.flight_status['operator_generation']='1'
    def uncertain(leg):raise RuntimeError('cancellation confirmation timeout')
    monkeypatch.setattr(nav,'stop_leg',uncertain)
    result=wait(handle.get_result_async());assert not result.result.success and nav.failure


def test_cli_timeout_before_ack_cancels_late_navigation(rig):
    from uav_lab_navigation.cli import Navigator
    nav,control,action=rig;original=nav.navigation_goal
    def delayed(request):time.sleep(.3);return original(request)
    nav.server.register_goal_callback(delayed)
    client=rclpy.create_node('delayed_ack_cli');operator=Navigator(client,.1)
    try:
        with pytest.raises(RuntimeError):operator.goto([1,1,2])
        time.sleep(.5)
        assert len(control['goals'])<=1
        assert control['cancelled']==len(control['goals'])
    finally:client.destroy_node()


def test_web_goto_uses_an_executable_cli_and_preserves_goal_yaw(rig,monkeypatch,tmp_path,isolated_ros_domain):
    """Execute the real command parser and real DDS action, no Popen/CLI fake."""
    import math
    from pathlib import Path
    from workbench_server import Workbench
    nav,control,_=rig
    run=tmp_path/'web-command-run'
    app=Workbench(Path(__file__).resolve().parents[2],watch=False)
    active=(run,{'supervisor_pid':1,'environment':{'ROS_DOMAIN_ID':isolated_ros_domain}})
    monkeypatch.setattr(app,'active_run',lambda:active)
    app.ingest({'kind':'telemetry','run_id':run.name,'diagnostics':{'uav001/flight':{
        'age_s':0.,'message':'HOLDING','values':{'armed':'True','landed':'False','offboard':'True'}}}},time.monotonic())
    def refresh():
        nav.flight_at=time.monotonic();nav.current_at=time.monotonic()
    timer=nav.create_timer(.05,refresh)
    try:
        job=app.request('goto',{'target':[1.,1.,2.],'yaw':.3})
        deadline=time.monotonic()+20
        while app.jobs[job['id']]['state']=='RUNNING' and time.monotonic()<deadline:time.sleep(.05)
        result=app.jobs[job['id']]
        assert result['state']=='SUCCEEDED',result
        assert result['result']['success'] is True
        assert len(control['goals'])==2
        orientation=control['goals'][-1].target.pose.orientation
        assert orientation.z==pytest.approx(math.sin(.15))
        assert orientation.w==pytest.approx(math.cos(.15))
    finally:nav.destroy_timer(timer);app.close()
