"""Generate a private sensor payload and bridge from the archived contract."""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

def quaternion_from_rpy(rpy):
    r,p,y = (v/2 for v in rpy)
    cr,sr,cp,sp,cy,sy = math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    return [sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy]

def select_profile(root, name):
    if name not in ('flight', 'sensors', 'navigation'):
        raise ValueError('unknown lab profile: '+name)
    return Path(root)/'simulation/worlds'/({'flight':'lab.sdf','sensors':'room.sdf','navigation':'navigation.sdf'}[name])

def element(parent, tag, value=None, **attributes):
    child = ET.SubElement(parent, tag, attributes)
    if value is not None:
        child.text = ' '.join(str(v) for v in value) if isinstance(value,(list,tuple)) else str(value)
    return child

def write_xml(tree, path):
    ET.indent(tree)
    ET.ElementTree(tree).write(path, encoding='utf-8', xml_declaration=True)

def prepare_sensors(root, run_dir, profile='sensors', sensor_profile=None, scene=None):
    root, run_dir = Path(root), Path(run_dir)
    if profile not in ('sensors','navigation'): raise ValueError('unknown sensor profile')
    if sensor_profile is not None and sensor_profile not in ('livox','livox-rtk','mechanical'):
        raise ValueError('unknown timed sensor profile')
    filename = 'sensors-'+sensor_profile+'.json' if sensor_profile else 'navigation-sensors.json' if profile=='navigation' else 'sensors.json'
    if profile=='navigation' and scene=='corridor' and sensor_profile is None:
        filename='navigation-corridor-sensors.json'
    c = json.loads((root/'configs'/filename).read_text())
    directory = run_dir/'configuration'
    directory.mkdir(exist_ok=True, parents=True)
    model_dir = directory/'models/x500_sensors'
    model_dir.mkdir(parents=True)
    sdf = ET.Element('sdf', version='1.9')
    model = element(sdf,'model',name='x500_sensors',canonical_link='base_link')
    element(element(model,'include',merge='true'),'uri','model://x500')
    link = element(model,'link',name='lab_payload')
    element(link,'pose',[0]*6,relative_to='base_link')
    inertial = element(link,'inertial')
    element(inertial,'mass',c['payload_mass_kg'])
    inertia = element(inertial,'inertia')
    for key in ('ixx','iyy','izz','ixy','ixz','iyz'):
        element(inertia,key,1e-5 if key in ('ixx','iyy','izz') else 0)
    joint = element(model,'joint',name='lab_payload_joint',type='fixed')
    element(joint,'parent','base_link'); element(joint,'child','lab_payload')
    transforms = []
    for name in ('lidar','imu','camera'):
        item = c[name]
        transforms.append({'parent':'base_link','child':item['frame'],'xyz':item['xyz'],
                           'xyzw':quaternion_from_rpy(item['rpy'])})
        if name=='lidar' and sensor_profile:
            continue
        sensor = element(link,'sensor',name='lab_'+name,type='gpu_lidar' if name=='lidar' else name)
        element(sensor,'pose',item['xyz']+item['rpy'])
        element(sensor,'update_rate',item['hz']); element(sensor,'always_on','true')
        element(sensor,'gz_frame_id',item.get('optical_frame',item['frame']))
        element(sensor,'topic',{'lidar':'/uav001/lidar','imu':'/uav001/sim/imu','camera':'/uav001/camera/image_raw'}[name])
        if name == 'lidar':
            lidar = element(sensor,'lidar'); scan = element(lidar,'scan')
            for axis in ('horizontal','vertical'):
                a = element(scan,axis)
                element(a,'samples',item[axis+'_samples']); element(a,'resolution',1)
                element(a,'min_angle',-item[axis+'_fov_rad']/2); element(a,'max_angle',item[axis+'_fov_rad']/2)
            rang = element(lidar,'range')
            element(rang,'min',item['min_m']); element(rang,'max',item['max_m']); element(rang,'resolution',.01)
            noise = element(lidar,'noise'); element(noise,'type','gaussian'); element(noise,'mean',0); element(noise,'stddev',item['noise_stddev_m'])
        elif name == 'camera':
            camera = element(sensor,'camera')
            element(camera,'horizontal_fov',item['horizontal_fov_rad'])
            image = element(camera,'image'); element(image,'width',item['width']); element(image,'height',item['height']); element(image,'format','R8G8B8')
            clip = element(camera,'clip'); element(clip,'near',item['near_m']); element(clip,'far',item['far_m'])
            element(camera,'camera_info_topic','/uav001/camera/camera_info')
        else:
            imu = element(sensor,'imu')
            for quantity, stddev in (('angular_velocity',item['gyro_stddev']),('linear_acceleration',item['accel_stddev'])):
                v = element(imu,quantity)
                for axis in 'xyz':
                    noise = element(element(v,axis),'noise',type='gaussian')
                    element(noise,'mean',0); element(noise,'stddev',stddev)
    transforms.append({'parent':c['camera']['frame'],'child':c['camera']['optical_frame'], 'xyz':[0,0,0], 'xyzw':quaternion_from_rpy(c['camera']['optical_rpy'])})
    truth = element(model,'plugin',filename='gz-sim-odometry-publisher-system',name='gz::sim::systems::OdometryPublisher')
    for key,value in {'dimensions':3,'odom_frame':c['truth']['frame'],'robot_base_frame':c['truth']['child_frame'],
                      'odom_publish_frequency':200 if sensor_profile else c['truth']['hz'],
                      'odom_topic':'/_lab/sensor_physics/odometry' if sensor_profile else '/uav001/ground_truth/odometry',
                      'tf_topic':'/uav001/sim/truth_tf','xyz_offset':c['model_to_base_link']}.items():
        element(truth,key,value)
    model_path = model_dir/'model.sdf'
    write_xml(sdf,model_path)
    (model_dir/'model.config').write_text('<model><name>x500_sensors</name><version>0.2.0</version><sdf version="1.9">model.sdf</sdf></model>\n')
    scenario = None
    if sensor_profile or scene:
        from experiment_scenarios import generate_scene
        scenario = generate_scene(root, directory/'scene', scene or 'circle-eight')
        c['scene'] = scenario['route']['scene']
        c['geometry_file'] = 'scene/sensor-geometry.json'
        c['route_file'] = 'scene/route.json'
        if 'gnss' in c:
            c['gnss']['events'] = scenario['route']['gnss_events']
            item=c['gnss']; transforms.append({'parent':'base_link','child':item['frame'],'xyz':item['xyz'],
                                                'xyzw':quaternion_from_rpy(item['rpy'])})
    world = ET.parse(scenario['world'] if scenario else select_profile(root,profile)).getroot()
    world.find('world/include/uri').text = 'model://x500_sensors'
    world_path = directory/'room.sdf'; write_xml(world,world_path)
    bridges = [('/uav001/lidar/points','sensor_msgs/msg/PointCloud2','gz.msgs.PointCloudPacked'),
               ('/uav001/sim/imu','sensor_msgs/msg/Imu','gz.msgs.IMU'),
               ('/uav001/camera/image_raw','sensor_msgs/msg/Image','gz.msgs.Image'),
               ('/uav001/camera/camera_info','sensor_msgs/msg/CameraInfo','gz.msgs.CameraInfo'),
               ('/uav001/ground_truth/odometry','nav_msgs/msg/Odometry','gz.msgs.Odometry')]
    if sensor_profile:
        bridges = [v for v in bridges if v[0] not in ('/uav001/lidar/points','/uav001/ground_truth/odometry')]
        bridges.append(('/_lab/sensor_physics/odometry','nav_msgs/msg/Odometry','gz.msgs.Odometry'))
    # JSON is a YAML subset understood by ros_gz_bridge.
    bridge_path = directory/'sensor-bridge.yaml'
    bridge_path.write_text(json.dumps([{'ros_topic_name':topic,'gz_topic_name':topic,'ros_type_name':ros,'gz_type_name':gz,
                                       'direction':'GZ_TO_ROS','qos_profile':'SENSOR_DATA'} for topic,ros,gz in bridges],indent=2)+'\n')
    c['transforms'] = transforms
    calibration_path = directory/'calibration.json'
    calibration_path.write_text(json.dumps(c,indent=2)+'\n')
    return {'model':model_path,'world':world_path,'bridge':bridge_path,'calibration':c,'calibration_path':calibration_path,
            'resource_path':model_dir.parent}
