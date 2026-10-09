"""Generate ROS parameters only from sensor calibration and pinned defaults."""
from pathlib import Path
import math,yaml,numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1]

def public_ntu_parameters(backend,run_id):
    """Pinned upstream rig and measured end-of-scan timestamp convention."""
    livo=yaml.safe_load((ROOT/'.deps/fast_livo2/config/NTU_VIRAL.yaml').read_text())['/**']['ros__parameters']
    camera=yaml.safe_load((ROOT/'.deps/fast_livo2/config/camera_NTU_VIRAL.yaml').read_text())['/**']['ros__parameters']
    calibration={'imu':{'xyz':[0.,0.,0.],'rpy':[0.,0.,0.]},'reference':'NTU_VIRAL upstream parameters',
                 'pose_time_offset_s':{'fast_lio2':-.1},'scan_end_offset_s':0.}
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
    params,camera=simulation_parameters(backend,calibration,run_id,camera_info,output/'rtk-output')
    (output/'parameters.yaml').write_text(yaml.safe_dump({'/**':{'ros__parameters':{'use_sim_time':True,**params}}},sort_keys=False))
    (output/'camera.yaml').write_text(yaml.safe_dump({'/**':{'ros__parameters':camera}},sort_keys=False))
    return output/'parameters.yaml'
