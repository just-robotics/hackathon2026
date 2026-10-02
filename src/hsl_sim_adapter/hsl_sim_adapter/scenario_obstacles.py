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
        if not isinstance(box,dict) or set(box)!={'name','size','pose'}:
            raise ValueError('box must contain name, size, pose')
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
    return f'''<sdf version="1.6"><model name="{box['name']}"><static>true</static>
<link name="box"><collision name="collision"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry></collision>
<visual name="visual"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
<material><ambient>0.75 0.45 0.15 1</ambient><diffuse>0.75 0.45 0.15 1</diffuse></material></visual>
</link></model></sdf>'''
