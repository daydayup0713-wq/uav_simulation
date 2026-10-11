import json,pytest


def test_template_blocks_hardware_entry_until_measured_inputs_exist():
    import importlib.util
    assert importlib.util.find_spec('check_hardware')
    import check_hardware
    report=check_hardware.check({'simulation':False})
    assert not report['ready'] and not report['hardware_flight_qualified']
    assert report['missing'] and report['ros_publications']==0


def test_hardware_readiness_checks_real_calibration_clock_and_firmware():
    import check_hardware
    data={'simulation':False,'firmware_commit':'a'*40,'xrce_client_major':2,'xrce_agent_version':'2.4.3',
       'transport':'serial:///dev/ttyACM0','computer_arch':'aarch64','mass_kg':2.,'body_size_m':[.8,.8,.6],
       'inertia_kg_m2':[.02,.02,.03],'calibration_id':'measured-v1','calibration_source':'bench measurement',
       'sensor_extrinsics':{'lidar':{'parent':'base_link','child':'lidar_link','xyz':[0.,0.,.2],'rpy':[0.,0.,0.]},
                            'imu':{'parent':'base_link','child':'imu_link','xyz':[0.,0.,0.],'rpy':[0.,0.,0.]}},
       'clock_contract':'source timestamps converted to common monotonic boot epoch; measured offset and uncertainty',
       'clock_uncertainty_ms':2.,'sensor_latency_ms':{'lidar':25.,'imu':3.},'geofence_enu_m':{'lower':[-5.,-5.,.2],'upper':[5.,5.,4.]}}
    report=check_hardware.check(data)
    assert report['ready'] and not report['hardware_flight_qualified'] and report['ros_publications']==0
    data['inertia_kg_m2']=[0.,0.,0.]
    assert not check_hardware.check(data)['ready']


@pytest.mark.parametrize('data',[None,[],{'sensor_extrinsics':None},
    {'sensor_extrinsics':{'imu':None}},{'sensor_latency_ms':None},{'geofence_enu_m':None}])
def test_malformed_hardware_input_is_rejected_without_ros_or_traceback(data):
    import check_hardware
    report=check_hardware.check(data)
    assert not report['ready'] and report['ros_publications']==0 and report['missing']
