"""Public simulation sensor topics; excludes control inputs and raw truth TF."""
SENSOR_TOPICS = {
    '/uav001/lidar/points': ('sensor_msgs/msg/PointCloud2', 'lidar', 'frame'),
    '/uav001/imu/data': ('sensor_msgs/msg/Imu', 'imu', 'frame'),
    '/uav001/camera/image_raw': ('sensor_msgs/msg/Image', 'camera', 'optical_frame'),
    '/uav001/camera/camera_info': ('sensor_msgs/msg/CameraInfo', 'camera', 'optical_frame'),
    '/uav001/ground_truth/odometry': ('nav_msgs/msg/Odometry', 'truth', 'frame'),
}
RECORD_TOPICS = tuple(SENSOR_TOPICS)+('/clock','/tf','/tf_static','/uav001/odometry','/uav001/path','/uav001/diagnostics')

def sensor_topics(calibration):
    return {**SENSOR_TOPICS, **({'/uav001/gnss/fix':('sensor_msgs/msg/NavSatFix','gnss','frame')} if 'gnss' in calibration else {})}

def record_topics(calibration):
    return RECORD_TOPICS + (('/uav001/gnss/fix','/uav001/sensors/timed_diagnostics') if 'gnss' in calibration else ())

def stamp_ns(stamp):
    return int(stamp.sec)*10**9+int(stamp.nanosec)

def remove_orientation(msg):
    # Imu orientation_covariance[0] == -1 means no orientation estimate.
    msg.orientation.x=msg.orientation.y=msg.orientation.z=0.
    msg.orientation.w=1.
    msg.orientation_covariance[0]=-1.
    return msg
