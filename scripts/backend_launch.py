"""Private algorithm command construction shared by replay and live checks."""
from pathlib import Path


def core_command(backend,binary,configuration):
    if backend=='glim':
        private='/uav001/backends/glim'
        remaps={'~/odom':private+'/raw_odometry','~/odom_corrected':private+'/global_odometry',
                '~/aligned_points':private+'/registered_points','~/aligned_points_corrected':private+'/global_registered_points',
                '~/map':private+'/global_map','/tf':private+'/tf','/tf_static':private+'/tf_static'}
        return [str(binary),'--ros-args','--params-file',str(Path(configuration)/'parameters.yaml'),
                *[part for a,b in remaps.items() for part in ('-r',a+':='+b)]]
    if backend in ('orb_slam3','vins_fusion'):
        configuration=Path(configuration)
        if backend=='vins_fusion':return [str(binary),str(configuration/'algorithm.yaml')]
        root=Path(__file__).resolve().parents[1]
        return [str(binary),str(root/'.deps/orb_slam3/Vocabulary/ORBvoc.txt'),str(configuration/'algorithm.yaml'),str(configuration.parent)]
    if backend=='lio_sam':
        private='/uav001/backends/lio_sam'
        remaps={'__ns':private,'lio_sam/mapping/odometry_incremental':private+'/raw_odometry',
                'lio_sam/mapping/odometry':private+'/global_odometry',
                'lio_sam/mapping/cloud_registered':private+'/global_registered_points',
                'lio_sam/mapping/map_global':private+'/global_map',
                '/lio_sam/mapping/loop_closure_constraints':private+'/loop_constraints',
                '/tf':private+'/tf','/tf_static':private+'/tf_static'}
        return [str(binary),'--ros-args','--params-file',str(Path(configuration)/'parameters.yaml'),
                *[part for a,b in remaps.items() for part in ('-r',a+':='+b)]]
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


def core_commands(backend,binary,configuration):
    if backend=='vins_fusion':
        private='/uav001/backends/vins_fusion'
        loop_binary=Path(binary).parents[1]/'loop_fusion/loop_fusion_node'
        remaps={'__ns':private,'/vins_estimator/odometry':private+'/raw_odometry',
                '/vins_estimator/keyframe_pose':private+'/keyframe_pose',
                '/vins_estimator/keyframe_point':private+'/keyframe_point',
                '/vins_estimator/extrinsic':private+'/extrinsic',
                '/vins_estimator/margin_cloud':private+'/margin_cloud',
                'odometry_rect':private+'/global_odometry',
                '/tf':private+'/tf','/tf_static':private+'/tf_static'}
        return [core_command(backend,binary,configuration),
                [str(loop_binary),str(Path(configuration)/'algorithm.yaml'),'--ros-args',
                 '-p','use_sim_time:=true',*[part for a,b in remaps.items() for part in ('-r',a+':='+b)]]]
    if backend=='lio_sam':
        directory=Path(binary).parent
        return [core_command(backend,directory/('lio_sam_'+name),configuration)
                for name in ('imageProjection','featureExtraction','imuPreintegration','mapOptimization')]
    return [core_command(backend,binary,configuration)]
