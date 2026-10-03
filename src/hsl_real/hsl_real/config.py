"""Configuration shared by the host CLI and ROS launch; no ROS imports."""
from pathlib import Path
from math import isfinite
import yaml


def load_config(path):
    path = Path(path).resolve()
    config = yaml.safe_load(path.read_text())
    required = {'mission_file', 'kobuki_port', 'livox_config', 'localization', 'odom_topic',
                'lidar_topic', 'lidar_mount', 'map_file', 'arena_bounds', 'ros_domain_id', 'rviz'}
    optional = {'localization_file', 'lidar_filter_file', 'planning_file', 'fastlio_file'}
    if not isinstance(config, dict) or not required <= set(config) <= required | optional:
        raise ValueError('real.yaml fields must be ' + str(sorted(required)))
    if config['localization'] not in ('odometry', 'external_tf', 'amcl', 'fastlio'):
        raise ValueError('localization must be odometry, external_tf, amcl or fastlio')
    if type(config['ros_domain_id']) is not int or not 0 <= config['ros_domain_id'] <= 232:
        raise ValueError('ros_domain_id must be in [0,232]')
    if type(config['rviz']) is not bool:
        raise ValueError('rviz must be a YAML boolean')
    def numbers(value, count):
        if not isinstance(value, list) or len(value) != count or any(
            type(v) not in (int, float) or not isfinite(v) for v in value):
            raise ValueError(f'expected {count} finite numbers: {value}')
        return [float(v) for v in value]
    config['lidar_mount'] = numbers(config['lidar_mount'], 6)
    b = config['arena_bounds'] = numbers(config['arena_bounds'], 4)
    if b[0] >= b[2] or b[1] >= b[3]:
        raise ValueError('invalid arena_bounds')
    for key in ('kobuki_port', 'odom_topic', 'lidar_topic'):
        if not isinstance(config[key], str) or not config[key].startswith('/'):
            raise ValueError(key + ' must be an absolute device/topic name')
    if config['localization'] == 'amcl' and (not config['map_file'] or not config.get('localization_file')):
        raise ValueError('amcl requires map_file and localization_file')
    if config['localization'] == 'fastlio' and (not config['map_file'] or not config.get('fastlio_file') or not config.get('localization_file')):
        raise ValueError('fastlio requires map_file, fastlio_file and localization_file')
    for key in ('mission_file', 'livox_config', 'map_file') + tuple(key for key in sorted(optional) if key in config):
        name = config[key]
        if key == 'map_file' and name == '':
            continue
        if not isinstance(name, str) or not name:
            raise ValueError(key + ' must name a file')
        target = (path.parent / name).resolve()
        if not target.is_relative_to(path.parent) or not target.is_file():
            raise ValueError(key + ' must exist inside the configuration directory')
        config[key] = str(target)
    mission = yaml.safe_load(Path(config['mission_file']).read_text())
    for side in ('robot', 'opponent'):
        mission[side]['start'] = numbers(mission[side]['start'], 3)
        size = numbers([mission[side]['start_area_half_size']], 1)[0]
        if size <= 0:
            raise ValueError('start area size must be positive')
    if mission['robot']['role'] not in ('explorer', 'guardian'):
        raise ValueError('invalid role')
    if type(mission['motion']['allow_reverse']) is not bool:
        raise ValueError('allow_reverse must be a boolean')
    for key in ('max_speed', 'max_angular_speed'):
        speed = numbers([mission['motion'][key]], 1)[0]
        if speed <= 0:
            raise ValueError('speed must be positive')
        mission['motion'][key] = speed
    duration = numbers([mission['match']['active_seconds']], 1)[0]
    if duration <= 0 or type(mission['match']['seed']) is not int or mission['match']['seed'] < 0:
        raise ValueError('invalid match duration or seed')
    height = numbers([mission['perception']['opponent_max_height']], 1)[0]
    if not .08 <= height <= .60:
        raise ValueError('opponent_max_height outside [.08,.60]')
    return config, mission


def start_polygon(side):
    x, y, _ = side['start']
    h = float(side['start_area_half_size'])
    return [x-h, y-h, x+h, y-h, x+h, y+h, x-h, y+h]


def load_recording(path):
    path=Path(path).resolve()
    cfg=yaml.safe_load(path.read_text())
    fields={'bag_topics','keyboard_timeout_s'}
    if not isinstance(cfg,dict) or set(cfg)!=fields:
        raise ValueError('recording.yaml fields must be '+str(sorted(fields)))
    topics=cfg['bag_topics']
    if not isinstance(topics,list) or not topics or len(topics)!=len(set(topics)) or any(not isinstance(t,str) or not t.startswith('/') for t in topics):
        raise ValueError('bag_topics must be unique absolute names')
    for key in ('keyboard_timeout_s',):
        value=cfg[key]
        if type(value) not in (int,float) or not isfinite(value) or value<=0:
            raise ValueError(key+' must be positive finite')
        cfg[key]=float(value)
    return cfg
