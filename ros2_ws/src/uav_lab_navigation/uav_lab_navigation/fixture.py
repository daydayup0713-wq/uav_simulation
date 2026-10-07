"""Ideal sensor fixture generator. Scene geometry is never a mapper/planner input."""
import hashlib
import json
from pathlib import Path
import numpy as np


def write_fixture(directory):
    directory=Path(directory); directory.mkdir(parents=True,exist_ok=False)
    azimuth,elevation=np.meshgrid(np.linspace(-np.pi,np.pi,360,endpoint=False),np.linspace(-np.pi/4,np.pi/4,32))
    directions=np.column_stack((np.cos(elevation.ravel())*np.cos(azimuth.ravel()),
        np.cos(elevation.ravel())*np.sin(azimuth.ravel()),np.sin(elevation.ravel())))
    origins=np.array([[x,y,z] for x,y in ((0,0),(0,3),(0,-3),(3,3),(3,-3),(3,0)) for z in (1.,2.,3.)])
    # Room and one physical divider, used only to synthesize lidar observations.
    boxes=[([-5.9,-5.9,-.1],[-5.7,5.9,4.9]),([5.7,-5.9,-.1],[5.9,5.9,4.9]),
        ([-5.9,-5.9,-.1],[5.9,-5.7,4.9]),([-5.9,5.7,-.1],[5.9,5.9,4.9]),
        ([-5.9,-5.9,-.1],[5.9,5.9,.1]),([-5.9,-5.9,4.7],[5.9,5.9,4.9]),
        ([1.3,-1,0],[1.7,1,4])]
    scans=[]
    for origin in origins:
        distance=np.full(len(directions),20.)
        for lower,upper in boxes:
            with np.errstate(divide='ignore',invalid='ignore'):
                a=(np.asarray(lower)-origin)/directions; b=(np.asarray(upper)-origin)/directions
            enter=np.minimum(a,b).max(axis=1); leave=np.maximum(a,b).min(axis=1)
            hit=(leave>=np.maximum(enter,0))&(enter>0)
            distance[hit]=np.minimum(distance[hit],enter[hit])
        scans.append(origin+distance[:,None]*directions)
    target=directory/'observations.npz'
    np.savez_compressed(target,origins=origins,endpoints=np.asarray(scans))
    metadata={'schema_version':1,'frame':'odom','source':'ideal analytical rays, no rendered precision claim',
        'sha256':hashlib.sha256(target.read_bytes()).hexdigest(),'scans':len(scans),'rays_per_scan':len(directions)}
    (directory/'manifest.json').write_text(json.dumps(metadata,indent=2)+'\n')
    return metadata
