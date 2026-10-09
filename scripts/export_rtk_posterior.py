#!/usr/bin/python3
"""Export a source-associated global body trajectory and separate RTK correction."""
import argparse,json
from pathlib import Path
import numpy as np
from uav_lab_experiments.rtk_posterior import normalize_posterior
from uav_lab_tools.datasets import file_hash


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('experiment',type=Path)
    args=parser.parse_args();directory=args.experiment.resolve();output=directory/'posterior'
    report=json.loads((directory/'report.json').read_text())
    if report['backend']!='fast_livo2_rtk' or not report.get('rtk_batch',{}).get('completed'):
        raise ValueError('completed RTK posterior experiment required')
    if output.exists():raise ValueError('posterior artifact exists; evidence is immutable')
    calibration=json.loads((directory/'calibration.json').read_text())
    raw=directory/'configuration/rtk-output/TUM/opt_trajectory_after.txt'
    cloud=directory/'configuration/rtk-output/global_pcd/after_optimization.pcd'
    if not cloud.is_file() or cloud.stat().st_size<100:raise ValueError('posterior point map missing')
    result=normalize_posterior(np.loadtxt(directory/'estimate.tum'),np.loadtxt(raw),calibration)
    output.mkdir();trajectory=output/'global_body.tum';np.savetxt(trajectory,result['body_trajectory'],fmt='%.9f')
    manifest={'schema':1,'backend':'fast_livo2_rtk','map_frame':'rtk_map','local_frame':'fast_livo2_rtk_odom',
              'body_frame':'base_link','world_convention':'ENU relative to first admitted GNSS fix',
              'source_time_s':result['source_time_s'],'map_to_local':result['map_to_local'].tolist(),
              'outside_local_support':result['outside_local_support'],
              'scope':'posterior map/body trajectory and latest rigid map-to-local correction; local control trajectory unchanged; optimized map must not be approximated by rigidly warping all old points',
              'control_application':False,'artifacts':[
                  {'kind':kind,'path':str(path),'sha256':file_hash(path)} for kind,path in
                  [('source_report',directory/'report.json'),('local_body_trajectory',directory/'estimate.tum'),
                   ('source_antenna_trajectory',raw),('global_body_trajectory',trajectory),('optimized_map',cloud),('calibration',directory/'calibration.json')]]}
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps({k:v for k,v in manifest.items() if k!='artifacts'},indent=2))


if __name__=='__main__':
    try:main()
    except (OSError,ValueError,KeyError) as error:raise SystemExit(str(error))
