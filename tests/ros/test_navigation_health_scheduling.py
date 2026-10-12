"""Real ROS health publication is independent of busy default input callbacks."""
import threading
import time
import pytest
rclpy=pytest.importorskip('rclpy')
from rclpy.executors import MultiThreadedExecutor
from rclpy.clock import Clock,ClockType
from diagnostic_msgs.msg import DiagnosticArray
from uav_lab_navigation.node import NavigationNode


def test_busy_input_callbacks_do_not_delay_health_timer(monkeypatch,isolated_ros_domain):
    monkeypatch.delenv('LAB_RUN_DIR',raising=False)
    rclpy.init(domain_id=isolated_ros_domain)
    node=NavigationNode();observer=rclpy.create_node('health_scheduler_observer');arrivals=[];busy=[]
    monkeypatch.setattr(node,'report',lambda **kw:{'ready':True,'state':'READY','reason':'','source_stamp':10.,'frame':'odom'})
    observer.create_subscription(DiagnosticArray,'/uav001/navigation/diagnostics',lambda msg:arrivals.append(time.monotonic()),10)
    def input_work():
        busy.append(time.monotonic());time.sleep(.3)
    timer=node.create_timer(.05,input_work,clock=Clock(clock_type=ClockType.STEADY_TIME))
    executor=MultiThreadedExecutor(num_threads=3)
    executor.add_node(node);executor.add_node(observer)
    worker=threading.Thread(target=executor.spin,daemon=True);worker.start()
    try:
        deadline=time.monotonic()+1.3
        while time.monotonic()<deadline:time.sleep(.02)
        assert len(busy)>=3
        assert len(arrivals)>=8
        assert max(b-a for a,b in zip(arrivals,arrivals[1:]))<.25
    finally:
        executor.shutdown(timeout_sec=3);worker.join(3);node.destroy_timer(timer)
        observer.destroy_node();node.destroy_node();rclpy.shutdown()


def test_map_commit_during_health_read_does_not_mix_initial_metadata():
    from types import SimpleNamespace
    class ConcurrentCommit:
        reads=0
        @property
        def map_stamp(self):
            self.reads+=1
            return None if self.reads==1 else 10.
    value=ConcurrentCommit();value.failure='';value.alignment=True;value.quality_ready=True
    value.quality_at=value.pose_at=value.map_at=time.monotonic();value.was_ready=False
    value.config={'source_timeout_s':1.};value.grid=SimpleNamespace(version=1)
    value.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=10200000000))
    report=NavigationNode.report(value)
    assert not report['ready'] and report['source_stamp'] is None
