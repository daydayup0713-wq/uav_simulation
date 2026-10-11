"""Generate ROS parameters only from sensor calibration and pinned defaults."""
from pathlib import Path
import math,yaml,numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1]

def public_ntu_parameters(backend,run_id):
    """Pinned upstream rig and measured end-of-scan timestamp convention."""
    livo=yaml.safe_load((ROOT/'.deps/fast_livo2/config/NTU_VIRAL.yaml').read_text())['/**']['ros__parameters']
    camera=yaml.safe_load((ROOT/'.deps/fast_livo2/config/camera_NTU_VIRAL.yaml').read_text())['/**']['ros__parameters']
    calibration={'imu':{'xyz':[0.,0.,0.],'rpy':[0.,0.,0.]},
                 'lidar':{'kind':'mechanical','measurement_time':'per_beam'},'reference':'NTU_VIRAL upstream parameters',
                 'pose_time_offset_s':{'fast_lio2':-.1},'scan_end_offset_s':0.}
    if backend in ('orb_slam3','vins_fusion'):
        rcl=np.array(livo['extrin_calib']['Rcl']).reshape(3,3)
        transform=np.eye(4);transform[:3,:3]=rcl.T
        transform[:3,3]=np.array(livo['extrin_calib']['extrinsic_T'])-rcl.T@np.array(livo['extrin_calib']['Pcl'])
        params={'metric_scale_source':'inertial','imu_T_camera':transform.ravel().tolist(),
                'imu_hz':385,'gyro_noise':.01,'accel_noise':.1,'camera_hz':10}
        return params,camera,calibration
    if backend=='fast_livo2':
        livo['common'].update(lid_topic='/uav001/lidar/points',imu_topic='/uav001/imu/data',img_topic='/uav001/camera/image_raw')
        livo['evo'].update(seq_name=run_id,pose_output_en=True)
        return livo,camera,calibration
    if backend!='fast_lio2':raise ValueError('NTU clip has no compatible RTK input')
    params=yaml.safe_load((ROOT/'.deps/fast_lio2/config/ouster64.yaml').read_text())['/**']['ros__parameters']
    params['common'].update(lid_topic='/uav001/lidar/points',imu_topic='/uav001/imu/data',time_offset_lidar_to_imu=-.1)
    params['preprocess'].update(scan_line=16,blind=1.)
    params['mapping'].update(extrinsic_T=livo['extrin_calib']['extrinsic_T'],extrinsic_R=livo['extrin_calib']['extrinsic_R'])
    params['pcd_save']['pcd_save_en']=False
    return params,camera,calibration

def simulation_parameters(backend,calibration,run_id,camera_info=None,output=None):
    lidar,imu,camera=[calibration[k] for k in ('lidar','imu','camera')]
    if lidar.get('measurement_time')!='per_beam':raise ValueError('timed measured scan required')
    if backend=='fast_livo2_rtk' and 'gnss' not in calibration:raise ValueError('GNSS calibration required')
    rbl=Rotation.from_euler('xyz',lidar['rpy']).as_matrix();rbi=Rotation.from_euler('xyz',imu['rpy']).as_matrix()
    rbc=Rotation.from_euler('xyz',camera['rpy']).as_matrix()@Rotation.from_euler('xyz',camera['optical_rpy']).as_matrix()
    tl,ti,tc=[np.array(x['xyz'],dtype=float) for x in (lidar,imu,camera)]
    ext_t=(rbi.T@(tl-ti)).tolist();ext_r=(rbi.T@rbl).ravel().tolist()
    fx=camera['width']/(2*math.tan(camera['horizontal_fov_rad']/2));cx=camera['width']/2;cy=camera['height']/2
    if camera_info is not None:fx,fy,cx,cy=camera_info.k[0],camera_info.k[4],camera_info.k[2],camera_info.k[5]
    else:fy=fx
    cam={'cam_model':'Pinhole','cam_width':camera['width'],'cam_height':camera['height'],'scale':1.,
         'cam_fx':float(fx),'cam_fy':float(fy),'cam_cx':float(cx),'cam_cy':float(cy),
         'cam_d0':0.,'cam_d1':0.,'cam_d2':0.,'cam_d3':0.}
    prefix='/uav001/backends/'+backend
    if backend in ('orb_slam3','vins_fusion'):
        transform=np.eye(4);transform[:3,:3]=rbi.T@rbc;transform[:3,3]=rbi.T@(tc-ti)
        return {'metric_scale_source':'inertial','imu_T_camera':transform.ravel().tolist(),
                'imu_hz':imu['hz'],'gyro_noise':imu['gyro_stddev'],'accel_noise':imu['accel_stddev'],
                'camera_hz':camera['hz'],'output_directory':str(output)},cam
    if backend=='lio_sam':
        if lidar['kind']!='mechanical':raise ValueError('mechanical measured ring scans required')
        if not imu.get('attitude_stddev_rad'):raise ValueError('attitude IMU required')
        params=yaml.safe_load((ROOT/'.deps/lio_sam/config/params.yaml').read_text())['/**']['ros__parameters']
        params.update(pointCloudTopic=prefix+'/points',imuTopic='/uav001/imu/data',
                      odomTopic=prefix+'/odometry/imu',gpsTopic=prefix+'/disabled_gps',
                      lidarFrame='lidar_link',baselinkFrame='lidar_link',
                      odometryFrame='lio_sam_odom',mapFrame='lio_sam_map',savePCD=False,
                      sensor='ouster',N_SCAN=lidar['vertical_samples'],Horizon_SCAN=lidar['horizontal_samples'],
                      lidarMinRange=float(lidar['min_m']),lidarMaxRange=float(lidar['max_m']),
                      extrinsicTrans=ext_t,extrinsicRot=ext_r,extrinsicRPY=ext_r.copy(),
                      imuAccNoise=imu['accel_stddev'],imuGyrNoise=imu['gyro_stddev'],imuGravity=9.81,
                      odometrySurfLeafSize=.2,mappingCornerLeafSize=.1,mappingSurfLeafSize=.2,
                      numberOfCores=2,mappingProcessInterval=.05,surroundingKeyframeSearchRadius=20.,
                      globalMapVisualizationSearchRadius=30.,globalMapVisualizationLeafSize=.2)
        return params,cam
    if backend=='fast_lio2':
        params=yaml.safe_load((ROOT/'.deps/fast_lio2/config/mid360.yaml').read_text())['/**']['ros__parameters']
        # FAST-LIO's sliding-cube trigger is 1.5*det_range from either face.
        # A 30m cube with 20m detection range deletes useful map slabs even
        # while stationary; keep a 10m margin beyond that trigger.
        params.update(point_filter_num=1,filter_size_surf=.2,filter_size_map=.2,cube_side_length=80.)
        params['common'].update(lid_topic=prefix+'/livox',imu_topic='/uav001/imu/data',time_sync_en=False,time_offset_lidar_to_imu=0.)
        params['preprocess'].update(lidar_type=1,scan_line=4,blind=.15,timestamp_unit=3,scan_rate=10)
        if lidar['kind']=='mechanical':
            params['common']['lid_topic']=prefix+'/points'
            params['preprocess'].update(lidar_type=2,scan_line=lidar['vertical_samples'],timestamp_unit=0)
        params['mapping'].update(extrinsic_T=ext_t,extrinsic_R=ext_r,extrinsic_est_en=False,fov_degree=360.,det_range=20.)
        params['publish'].update(path_en=True);params['pcd_save']['pcd_save_en']=False
        return params,cam
    source=ROOT/'.deps/fast_livo2/config/avia.yaml' if backend=='fast_livo2' else ROOT/'.deps/fast_livo2_rtk_ros2/src/fast_livo/config/HH-LVGO.yaml'
    params=next(iter(yaml.safe_load(source.read_text()).values()))['ros__parameters']
    params['common'].update(lid_topic=prefix+'/livox',imu_topic='/uav001/imu/data',img_topic='/uav001/camera/image_raw')
    if backend=='fast_livo2_rtk':params['common']['sensor_qos']='sensor_data'
    params['extrin_calib'].update(extrinsic_T=ext_t,extrinsic_R=ext_r,Rcl=(rbc.T@rbl).ravel().tolist(),Pcl=(rbc.T@(tl-tc)).tolist())
    params['time_offset'].update(imu_time_offset=0.,img_time_offset=0.)
    params['preprocess'].update(scan_line=4,blind=.15,filter_size_surf=.2,point_filter_num=1)
    params['local_map'].update(map_sliding_en=True,half_map_size=15,sliding_thresh=3.)
    params['pcd_save']['pcd_save_en']=False
    params['evo'].update(seq_name=run_id,pose_output_en=True)
    if backend=='fast_livo2_rtk':
        params['laserMapping']={k.removeprefix('cam_'):v for k,v in cam.items()}
        params['laserMapping']['outputfilepath']=str(output)
        params['gps']['extrinsic_T']=(rbi.T@(np.array(calibration['gnss']['xyz'])-ti)).tolist()
        params['gps']['gps_time_offset']=0.
        params['gps']['gps_topic']=prefix+'/gnss_pvt'
        params['platform']={'gnss_unix_offset':1609459200.}
    return params,cam

def write_config(backend,calibration,output,run_id,camera_info=None):
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    if backend=='glim':
        from uav_lab_localization.configuration import prepare_slam
        config=prepare_slam(ROOT,output/'glim',calibration,
                            use_timed_scans=calibration['lidar'].get('measurement_time')=='per_beam')
        file=config/'config_ros.json'
        import json
        parameters=json.loads(file.read_text())
        parameters['glim_ros'].update(imu_topic='/uav001/imu/data',points_topic='/uav001/lidar/points',
            imu_frame_id='imu_link',base_frame_id='imu_link',lidar_frame_id='lidar_link',
            odom_frame_id='glim_odom',map_frame_id='glim_map')
        file.write_text(json.dumps(parameters,indent=2)+'\n')
        (output/'parameters.yaml').write_text(yaml.safe_dump({'/**':{'ros__parameters':{
            'use_sim_time':True,'config_path':str(config),'dump_path':str(output.resolve()/'glim-map')}}}))
        (output/'camera.yaml').write_text(yaml.safe_dump({'/**':{'ros__parameters':{}}}))
        return output/'parameters.yaml'
    params,camera=simulation_parameters(backend,calibration,run_id,camera_info,output/'rtk-output')
    (output/'parameters.yaml').write_text(yaml.safe_dump({'/**':{'ros__parameters':{'use_sim_time':True,**params}}},sort_keys=False))
    (output/'camera.yaml').write_text(yaml.safe_dump({'/**':{'ros__parameters':camera}},sort_keys=False))
    if backend in ('orb_slam3','vins_fusion'):write_visual_settings(backend,params,camera,output)
    return output/'parameters.yaml'


def opencv_matrix(name,value,dtype='d'):
    value=np.asarray(value)
    return f'{name}: !!opencv-matrix\n  rows: {value.shape[0]}\n  cols: {value.shape[1]}\n  dt: {dtype}\n  data: ['+', '.join(str(float(v)) for v in value.ravel())+']\n'


def write_visual_settings(backend,params,camera,output):
    output=Path(output).resolve()
    if backend=='orb_slam3':
        settings={'File.version':'1.0','Camera.type':'PinHole',
            **{'Camera1.'+k:camera['cam_'+k] for k in ('fx','fy','cx','cy')},
            'Camera1.k1':camera['cam_d0'],'Camera1.k2':camera['cam_d1'],
            'Camera1.p1':camera['cam_d2'],'Camera1.p2':camera['cam_d3'],
            'Camera.width':camera['cam_width'],'Camera.height':camera['cam_height'],
            'Camera.fps':params['camera_hz'],'Camera.RGB':1,
            'IMU.NoiseGyro':params['gyro_noise']/math.sqrt(params['imu_hz']),
            'IMU.NoiseAcc':params['accel_noise']/math.sqrt(params['imu_hz']),
            'IMU.GyroWalk':1e-5,'IMU.AccWalk':1e-4,'IMU.Frequency':float(params['imu_hz']),
            'ORBextractor.nFeatures':1200,'ORBextractor.scaleFactor':1.2,'ORBextractor.nLevels':8,
            'ORBextractor.iniThFAST':20,'ORBextractor.minThFAST':7,
            'Viewer.KeyFrameSize':.05,'Viewer.KeyFrameLineWidth':1.,'Viewer.GraphLineWidth':.9,
            'Viewer.PointSize':2.,'Viewer.CameraSize':.08,'Viewer.CameraLineWidth':3.,
            'Viewer.ViewpointX':0.,'Viewer.ViewpointY':-.7,'Viewer.ViewpointZ':-3.5,'Viewer.ViewpointF':500.}
        text='%YAML:1.0\n'+yaml.safe_dump(settings,sort_keys=False)
        text+=opencv_matrix('IMU.T_b_c1',np.array(params['imu_T_camera']).reshape(4,4),'f')
    else:
        settings={'imu':1,'num_of_cam':1,'imu_topic':'/uav001/imu/data','image0_topic':'/uav001/camera/image_raw',
            'image1_topic':'','output_path':str(output/'vins-output'),'cam0_calib':'camera-intrinsics.yaml',
            'image_width':camera['cam_width'],'image_height':camera['cam_height'],'estimate_extrinsic':0,
            'multiple_thread':0,'max_cnt':200,'min_dist':20,'freq':15,'F_threshold':1.,'show_track':0,'flow_back':1,
            'max_solver_time':.04,'max_num_iterations':8,'keyframe_parallax':10.,
            'acc_n':params['accel_noise'],'gyr_n':params['gyro_noise'],'acc_w':1e-4,'gyr_w':1e-5,'g_norm':9.81,
            'estimate_td':0,'td':0.,'load_previous_pose_graph':0,'pose_graph_save_path':str(output/'pose_graph')+'/', 'save_image':0}
        settings.update(world_frame_id='vins_fusion_odom',body_frame_id='imu_link',camera_frame_id='camera_optical')
        (output/'vins-output').mkdir(exist_ok=True)
        (output/'pose_graph').mkdir(exist_ok=True)
        text='%YAML:1.0\n'+yaml.safe_dump(settings,sort_keys=False)
        text+=opencv_matrix('body_T_cam0',np.array(params['imu_T_camera']).reshape(4,4))
        intrinsic={'model_type':'PINHOLE','camera_name':'platform_camera','image_width':camera['cam_width'],
            'image_height':camera['cam_height'],'distortion_parameters':{key:camera['cam_d'+str(i)] for i,key in enumerate(('k1','k2','p1','p2'))},
            'projection_parameters':{k:camera['cam_'+k] for k in ('fx','fy','cx','cy')}}
        (output/'camera-intrinsics.yaml').write_text('%YAML:1.0\n'+yaml.safe_dump(intrinsic,sort_keys=False))
    (output/'algorithm.yaml').write_text(text)
