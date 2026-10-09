"""Private algorithm command construction shared by replay and live checks."""
from pathlib import Path


def core_command(backend,binary,configuration):
    if backend not in ('fast_lio2','fast_livo2','fast_livo2_rtk'):raise ValueError('unknown backend')
    private='/uav001/backends/'+backend
    configuration=Path(configuration)
    remaps=['/Odometry:='+private+'/raw_odometry','/aft_mapped_to_init:='+private+'/raw_odometry',
            '/cloud_registered:='+private+'/registered_points','/cloud_registered_body:='+private+'/body_points',
            '/path:='+private+'/path','/tf:='+private+'/tf','/tf_static:='+private+'/tf_static',
            '/ublox_driver/receiver_pvt:='+private+'/gnss_pvt','/mavros/vision_pose/pose:='+private+'/vision_pose',
            '/odometry/fast_livo2:='+private+'/keyframe_odometry','/synced_cloud:='+private+'/keyframe_points',
            '/gps/odometry:='+private+'/gnss_odometry','/gps/odometry_opt:='+private+'/gnss_odometry_opt',
            '/Laser_map:='+private+'/map','/cloud_effected:='+private+'/effective_points',
            '/cloud_visual_sub_map_before:='+private+'/visual_submap','/LIVO2/imu_propagate:='+private+'/imu_propagation',
            '/planes:='+private+'/planes','/voxels:='+private+'/voxels','/planner_normal:='+private+'/planner_normal',
            '/dyn_obj:='+private+'/dynamic_points','/dyn_obj_removed:='+private+'/static_points','/dyn_obj_dbg_hist:='+private+'/dynamic_history']
    return [str(binary),'--ros-args','--params-file',str(configuration/'parameters.yaml'),'--params-file',str(configuration/'camera.yaml'),
            *[v for remap in remaps for v in ('-r',remap)]]
