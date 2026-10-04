"""Real CLI signal cleanup after rosbag opens its database, in an isolated domain."""
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time
import pytest
pytest.importorskip('rosbag2_py')
from sensor_model import prepare_sensors
from conftest import ROOT

@pytest.mark.parametrize('interrupt',[signal.SIGINT,signal.SIGTERM])
def test_interrupted_record_closes_owned_rosbag(tmp_path,isolated_ros_domain,interrupt):
    runtime=tmp_path/'.runtime';run=runtime/'signal-test';run.mkdir(parents=True)
    calibration=prepare_sensors(ROOT,run)['calibration']
    shutil.copy2(ROOT/'configs/recording-qos.yaml',run/'configuration/recording-qos.yaml')
    (runtime/'current-run').write_text(str(run)+'\n');(run/'ready').write_text('ready\n')
    (run/'manifest.json').write_text(json.dumps({'run_id':run.name,'profile':'sensors','environment':{'ROS_DOMAIN_ID':str(isolated_ros_domain)},'calibration':calibration,'dependencies':json.loads((ROOT/'dependencies/lock.json').read_text()),'platform_commit':'test-fixture','platform_dirty':False}))
    env={**os.environ,'ROS_DOMAIN_ID':str(isolated_ros_domain),'LAB_ROOT':str(tmp_path),'ROS_LOG_DIR':str(tmp_path/'ros')}
    env.pop('LAB_RUN_DIR',None)
    recorder=None
    with (tmp_path/'operator.log').open('w') as log:
        operator=subprocess.Popen(['/usr/bin/python3','-c','from uav_lab_tools.data_cli import main; raise SystemExit(main())','record','--duration','30'],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            end=time.monotonic()+10
            while time.monotonic()<end:
                bags=list((tmp_path/'recordings').glob('*/bag/*.db3'))
                if bags:break
                assert operator.poll() is None,(tmp_path/'operator.log').read_text()
                time.sleep(.05)
            assert bags,'rosbag did not open a real database'
            children=Path(f'/proc/{operator.pid}/task/{operator.pid}/children').read_text().split()
            assert children
            recorder=int(children[0])
            os.kill(operator.pid,interrupt);operator.wait(timeout=20)
            assert operator.returncode==1,(tmp_path/'operator.log').read_text()
            dataset=bags[0].parent.parent
            metadata=json.loads((dataset/'dataset.json').read_text())
            assert not metadata['complete'] and 'interrupted' in metadata['reason']
            assert (dataset/'bag/metadata.yaml').exists(),'rosbag must finalize metadata after interruption'
            with pytest.raises(ProcessLookupError):os.killpg(recorder,0)
        finally:
            for pid in (recorder,operator.pid):
                if pid:
                    try:os.killpg(pid,signal.SIGKILL)
                    except ProcessLookupError:pass
            operator.wait(timeout=3)
