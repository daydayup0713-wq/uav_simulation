"""Deterministic experimental worlds; simulator geometry stays out of estimators."""
import json
from pathlib import Path
import struct
import xml.etree.ElementTree as ET
import zlib
import numpy as np

SCENES = ('circle-eight', 'helix', 'multi-room', 'corridor', 'dense', 'outdoor-rtk')


def densify(vertices, count):
    vertices = np.asarray(vertices, dtype=float)
    distance = np.r_[0., np.cumsum(np.linalg.norm(np.diff(vertices, axis=0), axis=1))]
    return np.column_stack([np.interp(np.linspace(0, distance[-1], count), distance, vertices[:, i]) for i in range(3)])


def definition(name):
    if name not in SCENES:
        raise ValueError('unknown experiment scene')
    boxes = []
    if name == 'circle-eight':
        a, b = np.linspace(0, 2*np.pi, 17), np.linspace(0, 2*np.pi, 32)
        points = np.vstack([np.column_stack([3-3*np.cos(a), 3*np.sin(a), np.full(len(a), 2.)]),
                            np.column_stack([4*np.sin(b[1:]), 2*np.sin(2*b[1:]), np.full(len(b)-1, 2.)])])
    elif name == 'helix':
        a = np.linspace(0, 4*np.pi, 33)
        points = np.column_stack([2-2*np.cos(a), 2*np.sin(a), 2+2*np.sin(a/4)])
    elif name == 'multi-room':
        points = densify([[0,0,2],[1.5,2,2],[2,0,2],[4.5,0,2],[4.5,2,2],[5,0,2],
                          [7.5,0,2],[7.5,-2,2],[7,0,2],[4.5,0,2],[4.5,-2,2],[2,0,2],[0,0,2]], 40)
        for x in (3., 6.):
            boxes.extend([[[x-.1,-4,0],[x+.1,-1.4,5]], [[x-.1,1.4,0],[x+.1,4,5]]])
    elif name == 'corridor':
        a = np.linspace(0, np.pi, 40)
        points = np.column_stack([60*np.sin(a), .6*np.sin(6*a), np.full(len(a), 2.)])
        for x in range(3, 62, 3):
            boxes.extend([[[x-.3,-3,0],[x+.3,-2.2,4]], [[x-.3,2.2,0],[x+.3,3,4]]])
    elif name == 'dense':
        points = densify([[0,0,2],[7,0,2],[7,1,2],[0,1,2],[0,-1,2],[7,-1,2],[0,0,2]], 36)
        for x in (1.5, 3., 4.5, 6.):
            for y in (-3., -2.1, 2.1, 3.):
                boxes.append([[x-.25,y-.25,0],[x+.25,y+.25,4]])
    else:
        a = np.linspace(0, 2*np.pi, 42)
        points = np.column_stack([60-60*np.cos(a), 20*np.sin(a), 2+.8*np.sin(a)**2])
        for x in range(10, 120, 10):
            boxes.append([[x-1.,-1.,0.],[x+1.,1.,4.]])
    lower = points.min(axis=0)-[2.,2.,2.]
    upper = points.max(axis=0)+[2.,2.,2.]
    lower[2], upper[2] = -.2, 5.
    boxes.insert(0, [[float(lower[0]-2), float(lower[1]-2), -.2], [float(upper[0]+2), float(upper[1]+2), 0.]])
    if name != 'outdoor-rtk':
        boxes.extend([[[lower[0]-.2,lower[1],0],[lower[0],upper[1],5]],
                      [[upper[0],lower[1],0],[upper[0]+.2,upper[1],5]],
                      [[lower[0],lower[1]-.2,0],[upper[0],lower[1],5]],
                      [[lower[0],upper[1],0],[upper[0],upper[1]+.2,5]]])
    else:
        # Landmarks are useful visual/LiDAR features; this remains an open world.
        boxes.extend([[[x,-28,0],[x+1,-27,5]] for x in range(0,121,10)])
        # A launch-area facade provides vertical constraints before takeoff;
        # the long route remains in x >= 0 and stays clear of this structure.
        boxes.append([[-3.,-4.,0.],[-2.8,4.,5.]])
    return points, np.asarray(boxes, dtype=float), lower, upper


def texture_png(path, seed=7103):
    """Fixed irregular high-contrast albedo, generated without external assets."""
    rng = np.random.default_rng(seed)
    tiles = rng.integers(20, 235, (16, 16, 3), dtype=np.uint8)
    pixels = np.repeat(np.repeat(tiles, 16, axis=0), 16, axis=1)
    pixels[::16], pixels[:,::16] = 245, 245
    raw = b''.join(b'\0'+row.tobytes() for row in pixels)
    def chunk(kind, data):
        return struct.pack('>I',len(data))+kind+data+struct.pack('>I',zlib.crc32(kind+data)&0xffffffff)
    path.write_bytes(b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('>IIBBBBB',256,256,8,2,0,0,0))+
                     chunk(b'IDAT',zlib.compress(raw))+chunk(b'IEND',b''))


def generate_scene(root, directory, name):
    directory = Path(directory); directory.mkdir(parents=True, exist_ok=True)
    points, boxes, lower, upper = definition(name)
    tree = ET.parse(Path(root)/'simulation/worlds/navigation.sdf')
    world = tree.getroot().find('world')
    world.find('physics/max_step_size').text = '0.002'
    for model in list(world.findall('model')):
        world.remove(model)
    texture = directory/'feature-albedo.png'; texture_png(texture)
    for index, (low, high) in enumerate(boxes):
        object_texture = texture
        if index and name!='corridor':
            object_texture = directory/f'feature-albedo-{index:03d}.png'
            texture_png(object_texture,7103+index)
        model = ET.SubElement(world,'model',name=f'feature_{index:03d}')
        ET.SubElement(model,'static').text='true'
        ET.SubElement(model,'pose').text=' '.join(map(str,[*((low+high)/2),0,0,0]))
        link = ET.SubElement(model,'link',name='link')
        for tag in ('collision','visual'):
            child=ET.SubElement(link,tag,name=tag)
            ET.SubElement(ET.SubElement(ET.SubElement(child,'geometry'),'box'),'size').text=' '.join(map(str,high-low))
            if tag=='visual':
                material=ET.SubElement(child,'material')
                ET.SubElement(material,'ambient').text='0.8 0.8 0.8 1'
                ET.SubElement(material,'diffuse').text='0.8 0.8 0.8 1'
                metal=ET.SubElement(ET.SubElement(material,'pbr'),'metal')
                ET.SubElement(metal,'albedo_map').text=str(object_texture.resolve())
                ET.SubElement(metal,'metalness').text='0'
                ET.SubElement(metal,'roughness').text='0.9'
    ET.indent(tree)
    path = directory/'experiment.sdf'; tree.write(path,encoding='utf-8',xml_declaration=True)
    geometry = directory/'sensor-geometry.json'
    geometry.write_text(json.dumps({'schema_version':1,'scene':name,'boxes':boxes.tolist(),
                                    'input_scope':'simulator sensor generator and independent evaluation only'},indent=2)+'\n')
    route = {'schema_version':1,'scene':name,'frame':'odom','points':points.tolist(), 'truth_input_allowed':False,
             'controls':points[1:].tolist(),
             'limits':{'speed':.5,'acceleration':.5,'jerk':1.},'dwell':{},
             'flight_bounds':{'lower':lower.tolist(),'upper':upper.tolist()},
             'gnss_events':[{'start_s':50.,'end_s':80.,'mode':'lost'},
                            {'start_s':80.,'end_s':90.,'mode':'float'},
                            {'start_s':100.,'end_s':102.,'mode':'outlier','offset_m':[30.,0.,0.]}] if name=='outdoor-rtk' else []}
    (directory/'route.json').write_text(json.dumps(route,indent=2)+'\n')
    return {'world':path,'geometry':geometry,'texture':texture,'route':route}
