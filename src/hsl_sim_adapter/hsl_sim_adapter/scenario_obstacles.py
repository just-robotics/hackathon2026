"""Validation and SDF for configurable Gazebo-only boxes."""
from math import isfinite
from pathlib import Path
import re
import yaml


def load_obstacles(path):
    if not path:
        return {'enabled': False, 'boxes': []}
    cfg = yaml.safe_load(Path(path).read_text())
    if not isinstance(cfg,dict) or set(cfg)!={'enabled','boxes'} or type(cfg['enabled']) is not bool:
        raise ValueError('obstacles must contain enabled: bool and boxes: list')
    if not isinstance(cfg['boxes'],list):
        raise ValueError('boxes must be a list')
    names=set()
    for box in cfg['boxes']:
        if not isinstance(box,dict) or not {'name','size','pose'} <= set(box) or set(box)-{'name','size','pose','movable','mass','friction'}:
            raise ValueError('box requires name, size, pose and optional movable/mass/friction')
        if type(box.get('movable',False)) is not bool:
            raise ValueError('movable must be bool')
        for key,default in (('mass',.10),('friction',.30)):
            value=box.get(key,default)
            if isinstance(value,bool) or not isinstance(value,(int,float)) or not isfinite(value) or value<=0:
                raise ValueError(f'{key} must be positive finite')
        name=box['name']
        if not isinstance(name,str) or not re.fullmatch(r'unknown_box_[A-Za-z0-9_]+',name) or name in names:
            raise ValueError('unique names beginning unknown_box_ are required')
        names.add(name)
        for field in ('size','pose'):
            values=box[field]
            if not isinstance(values,list) or len(values)!=3 or any(
                isinstance(v,bool) or not isinstance(v,(int,float)) or not isfinite(v) for v in values):
                raise ValueError('size and pose require three finite numbers')
        if min(box['size'])<=0:
            raise ValueError('box sizes must be positive')
    return cfg


def box_sdf(box):
    sx,sy,sz=box['size']
    mass=box.get('mass',.10)
    friction=box.get('friction',.30)
    static='false' if box.get('movable',False) else 'true'
    inertial=f'<inertial><mass>{mass}</mass><inertia><ixx>{mass*(sy*sy+sz*sz)/12}</ixx><iyy>{mass*(sx*sx+sz*sz)/12}</iyy><izz>{mass*(sx*sx+sy*sy)/12}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>'
    return f'''<sdf version="1.6"><model name="{box['name']}"><static>{static}</static>
<link name="box">{inertial}<collision name="collision"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry><surface><friction><ode><mu>{friction}</mu><mu2>{friction}</mu2></ode></friction></surface></collision>
<visual name="visual"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
<material><ambient>0.75 0.45 0.15 1</ambient><diffuse>0.75 0.45 0.15 1</diffuse></material></visual>
</link></model></sdf>'''
