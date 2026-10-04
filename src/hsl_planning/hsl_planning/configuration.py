"""Shared external global/local planner configuration, validated before start."""
from math import isfinite
from pathlib import Path
import yaml

GLOBAL_DEFAULTS = {
    'resolution': .10, 'robot_radius': .21, 'local_safety_margin': 0.,
    'pose_timeout': 1.2,
    'scan_timeout': 1.8, 'intent_timeout': 1.,
    # Восстановление при застревании: простой stuck_hold_s с нулевым смещением
    # у маршрута; временная метка препятствия на phantom_distance впереди живёт
    # phantom_ttl_s (препятствие, которого нет в карте, например коробка).
    'stuck_hold_s': 4., 'phantom_distance': .45, 'phantom_ttl_s': 30.,
}
LOCAL_DEFAULTS = {
    'MPPI.time_steps': 60,
    'MPPI.PathAngleCritic.max_angle_to_furthest': .35,
    'MPPI.ObstaclesCritic.repulsion_weight': 1.,
    'MPPI.ObstaclesCritic.critical_weight': 10.,
    'MPPI.ObstaclesCritic.collision_margin_distance': .02,
    'MPPI.ObstaclesCritic.inflation_radius': .45,
    'MPPI.ObstaclesCritic.cost_scaling_factor': 8.,
    'costmap.inflation_layer.inflation_radius': .45,
    'costmap.inflation_layer.cost_scaling_factor': 8.,
}


def load_planning(path=None):
    data=yaml.safe_load(Path(path).read_text()) if path else {}
    if not isinstance(data,dict) or set(data)-{'global','local'}:
        raise ValueError('planning.yaml accepts global/local sections')
    result={}
    for section, defaults in [('global',GLOBAL_DEFAULTS),('local',LOCAL_DEFAULTS)]:
        values=data.get(section,{})
        if not isinstance(values,dict) or set(values)-set(defaults):
            raise ValueError(f'unknown {section} planning parameters')
        values=dict(defaults,**values)
        for key,v in values.items():
            if type(v) not in (float,int) or not isfinite(v) or v<0:
                raise ValueError(f'{section}.{key} must be finite and nonnegative')
            if key=='MPPI.time_steps':
                if type(v) is not int or not 20<=v<=120:
                    raise ValueError('MPPI.time_steps must be an integer in [20,120]')
                values[key]=v
            else:
                values[key]=float(v)
        result[section]=values
    g,l=result['global'],result['local']
    if not .178 <= g['robot_radius'] <= .5:
        raise ValueError('robot_radius must cover the Kobuki body (>=0.178 m)')
    if not .025 <= g['resolution'] <= .2:
        raise ValueError('global resolution must be in [0.025,0.2] m')
    if any(g[k]<=0 for k in ('pose_timeout','scan_timeout','intent_timeout','stuck_hold_s')):
        raise ValueError('planning lifetimes/timeouts must be positive')
    if not 0 < l['MPPI.PathAngleCritic.max_angle_to_furthest'] <= 3.141592653589793:
        raise ValueError('path angle threshold must be in (0,pi] rad')
    if l['MPPI.ObstaclesCritic.inflation_radius'] != l['costmap.inflation_layer.inflation_radius'] or l['MPPI.ObstaclesCritic.cost_scaling_factor'] != l['costmap.inflation_layer.cost_scaling_factor']:
        raise ValueError('critic and costmap inflation parameters must agree')
    if l['costmap.inflation_layer.inflation_radius'] < g['robot_radius']:
        raise ValueError('inflation must cover the robot radius')
    l['costmap.robot_radius']=g['robot_radius']
    return result
