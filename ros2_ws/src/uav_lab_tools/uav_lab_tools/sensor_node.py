"""Only sensor extrinsics and orientation-free algorithm IMU, never PX4 input."""
import json
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu
from tf2_ros import StaticTransformBroadcaster
from .sensor_contract import remove_orientation

class Sensors(Node):
    def __init__(self):
        super().__init__('sensor_frames',namespace='uav001')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
        calibration=json.loads(open(self.declare_parameter('calibration_file','').value).read())
        self.static=StaticTransformBroadcaster(self)
        transforms=[]
        for spec in calibration['transforms']:
            tf=TransformStamped();tf.header.frame_id=spec['parent'];tf.child_frame_id=spec['child']
            tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z=map(float,spec['xyz'])
            tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w=spec['xyzw']
            transforms.append(tf)
        self.static.sendTransform(transforms)
        self.publisher=self.create_publisher(Imu,'imu/data',qos_profile_sensor_data)
        self.create_subscription(Imu,'sim/imu',self.imu,qos_profile_sensor_data)

    def imu(self,msg):
        self.publisher.publish(remove_orientation(msg))

def main():
    rclpy.init();node=None
    try:
        node=Sensors();rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
