import time
import numpy as np
import pytest
rclpy=pytest.importorskip('rclpy')
from uav_lab_navigation.node import NavigationNode
from uav_lab_navigation.cli import pose
from concurrent.futures.process import BrokenProcessPool


def test_broken_worker_returns_explicit_latched_failure(monkeypatch,isolated_ros_domain):
    monkeypatch.delenv('LAB_RUN_DIR',raising=False);rclpy.init(domain_id=isolated_ros_domain);node=NavigationNode();original=node.planning_worker
    class Broken:
        def submit(self,*args):raise BrokenProcessPool('worker died')
    try:
        node.planning_worker=Broken();node.current=pose([0,0,2]).pose;node.current_at=time.monotonic()
        monkeypatch.setattr(node,'report',lambda:{'ready':True})
        with pytest.raises(ValueError,match='worker'):node.make_plan(pose([1,1,2]))
        assert node.failure and 'worker' in node.failure
    finally:node.planning_worker=original;node.destroy_node();rclpy.shutdown()


def test_plan_archive_is_immutable_and_versioned(monkeypatch,tmp_path,isolated_ros_domain):
    monkeypatch.setenv('LAB_RUN_DIR',str(tmp_path));rclpy.init(domain_id=isolated_ros_domain);node=NavigationNode()
    try:
        node.grid.score[:]=-4;node.grid.seen[:]=True;node.grid.version=12;node.grid.source_stamp=7.
        collision=node.grid.snapshot(node.envelope);proof=node.archive_plan(collision)
        path=tmp_path/proof['map_artifact'];data=np.load(path)
        assert data['map_version']==12 and data['source_stamp']==7.
        assert np.array_equal(data['free'],collision.free)
        before=path.read_bytes();node.grid.score[:]=4;node.grid.version=13
        node.archive_plan(node.grid.snapshot(node.envelope))
        assert path.read_bytes()==before
        import hashlib
        assert hashlib.sha256(before).hexdigest()==proof['map_sha256']
    finally:node.destroy_node();rclpy.shutdown()


def test_concurrent_same_version_archive_never_hashes_partial_file(monkeypatch,tmp_path,isolated_ros_domain):
    import threading
    monkeypatch.setenv('LAB_RUN_DIR',str(tmp_path));rclpy.init(domain_id=isolated_ros_domain);node=NavigationNode()
    node.grid.version=20;node.grid.source_stamp=10.;collision=node.grid.snapshot(node.envelope)
    entered=threading.Event();original=np.savez_compressed;first=[]
    def delayed(*args,**kw):entered.set();time.sleep(.15);return original(*args,**kw)
    monkeypatch.setattr(np,'savez_compressed',delayed)
    worker=threading.Thread(target=lambda:first.append(node.archive_plan(collision)));worker.start()
    try:
        assert entered.wait(2);second=node.archive_plan(collision);worker.join(2)
        assert second['map_sha256']==first[0]['map_sha256']
    finally:worker.join(2);node.destroy_node();rclpy.shutdown()
