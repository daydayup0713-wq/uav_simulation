"""Run in the private RTK environment; checks generated message fields, not mocks."""
import json
from pathlib import Path
import pytest
import rclpy
from sensor_msgs.msg import NavSatFix
GnssPVTSolnMsg=pytest.importorskip('gnss_comm.msg').GnssPVTSolnMsg
from uav_lab_experiments.algorithm_sensor_node import AlgorithmSensorAdapter


def test_admitted_position_velocity_and_gps_epoch_use_real_pvt_schema(tmp_path):
    calibration=Path('configs/sensors-livox-rtk.json').resolve()
    rclpy.init(args=['--ros-args','-p','backend:=fast_livo2_rtk','-p','calibration:='+str(calibration)])
    node=AlgorithmSensorAdapter();received=[]
    node.gnss_pub.publish=lambda msg:received.append(msg)
    origin=json.loads(calibration.read_text())['gnss']['origin']
    try:
        msg=NavSatFix();msg.header.frame_id='gnss_link';msg.header.stamp.sec=10
        msg.status.status=2;msg.latitude,msg.longitude,msg.altitude=origin
        msg.position_covariance=[.0004,0.,0.,0.,.0004,0.,0.,0.,.0004]
        msg.position_covariance_type=2
        node.fix(msg);assert not received
        msg.header.stamp.sec=11;msg.altitude+=.1;node.fix(msg)
        pvt=received[0];assert isinstance(pvt,GnssPVTSolnMsg)
        assert pvt.vel_d==pytest.approx(-.1) and pvt.vel_acc>0
        assert pvt.time.week*604800+pvt.time.tow+315964800-18-1609459200==pytest.approx(11.)
        msg.header.stamp.sec=12;msg.altitude+=30;node.fix(msg)
        assert len(received)==1 and node.gnss_counts['rejected']==1
    finally:
        node.destroy_node();rclpy.shutdown()
