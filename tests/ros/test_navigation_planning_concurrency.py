"""Real ROS heartbeat remains live during a bounded, blocked 3D search."""
import threading
import time
import pytest
rclpy=pytest.importorskip('rclpy')
from uav_lab_navigation.node import NavigationNode
from uav_lab_navigation.cli import pose
from diagnostic_msgs.msg import DiagnosticArray


def test_search_budget_does_not_stall_ros_heartbeat(monkeypatch,isolated_ros_domain):
    monkeypatch.delenv('LAB_RUN_DIR',raising=False);rclpy.init(domain_id=isolated_ros_domain)
    node=NavigationNode();client=rclpy.create_node('planner_heartbeat_observer');arrivals=[];outcomes=[];errors=[]
    client.create_subscription(DiagnosticArray,'/uav001/navigation/diagnostics',lambda msg:arrivals.append(time.monotonic()),10)
    node.grid.score[:]=-4;node.grid.seen[:]=True;node.grid.score[35,:,:]=4
    node.current=pose([0,0,2]).pose;node.current_at=time.monotonic();node.config['planning_timeout_s']=1.
    monkeypatch.setattr(node,'report',lambda **kw:{'ready':True,'state':'READY','reason':''})
    def search():
        try:outcomes.append(node.make_plan(pose([2,0,2]))[1])
        except Exception as error:errors.append(error)
    worker=threading.Thread(target=search);worker.start()
    try:
        deadline=time.monotonic()+5
        while worker.is_alive() and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.01);rclpy.spin_once(client,timeout_sec=.01)
        worker.join(1)
        assert not errors and outcomes and outcomes[0].reason=='SEARCH_BUDGET_EXCEEDED'
        assert len(arrivals)>=8
        assert max(b-a for a,b in zip(arrivals,arrivals[1:]))<.4
    finally:worker.join(5);client.destroy_node();node.destroy_node();rclpy.shutdown()
