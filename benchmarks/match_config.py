"""External mission configuration: map-frame poses, simulator conversion and ROS args."""
import json
import math
from pathlib import Path
import re
import yaml

DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / 'config/match.yaml'


def load_config(path=DEFAULT_CONFIG):
    config = yaml.safe_load(Path(path).read_text())
    if not isinstance(config, dict):
        raise ValueError('match config must be a mapping')
    config.setdefault('perception', {'opponent_max_height': 0.46})
    required = {'perception', 'robot', 'opponent', 'motion', 'match', 'simulation'}
    if set(config) != required:
        raise ValueError(f'config sections must be {sorted(required)}')
    keys = {'perception': {'opponent_max_height'}, 'robot': {'role', 'start', 'start_area_half_size'},
            'opponent': {'start', 'start_area_half_size'},
            'motion': {'allow_reverse', 'max_speed', 'max_angular_speed'},
            'match': {'seed', 'active_seconds'},
            'simulation': {'world', 'map_origin_world', 'arena_bounds', 'headless', 'rviz'}}
    for section, expected in keys.items():
        if not isinstance(config[section], dict) or set(config[section]) != expected:
            raise ValueError(f'{section} fields must be {sorted(expected)}')
    def numbers(values, length):
        if not isinstance(values, list) or len(values) != length or any(
                isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
                for v in values):
            raise ValueError(f'expected {length} finite numbers: {values!r}')
    for side in ('robot', 'opponent'):
        numbers(config[side]['start'], 3)
        numbers([config[side]['start_area_half_size']], 1)
        if config[side]['start_area_half_size'] <= 0:
            raise ValueError('start area size must be positive')
    if config['robot']['role'] not in ('explorer', 'guardian'):
        raise ValueError('robot role must be explorer or guardian')
    sim, motion = config['simulation'], config['motion']
    numbers(sim['map_origin_world'], 2)
    numbers(sim['arena_bounds'], 4)
    if not (sim['arena_bounds'][0] < sim['arena_bounds'][2] and
            sim['arena_bounds'][1] < sim['arena_bounds'][3]):
        raise ValueError('invalid arena bounds')
    if not isinstance(sim['world'], str) or not re.fullmatch(r'[A-Za-z0-9_-]+', sim['world']):
        raise ValueError('invalid world name')
    if type(sim['headless']) is not bool or type(motion['allow_reverse']) is not bool:
        raise ValueError('headless and allow_reverse must be YAML booleans')
    if sim['rviz'] != 'auto' and type(sim['rviz']) is not bool:
        raise ValueError('rviz must be auto, true or false')
    for value in (motion['max_speed'], motion['max_angular_speed'], config['match']['active_seconds']):
        numbers([value], 1)
        if value <= 0:
            raise ValueError('speed and duration must be positive')
    height = config['perception']['opponent_max_height']
    numbers([height], 1)
    if not 0.08 <= height <= 0.60:
        raise ValueError('opponent_max_height must be within detector height band [0.08, 0.60]')
    if type(config['match']['seed']) is not int or config['match']['seed'] < 0:
        raise ValueError('seed must be a nonnegative integer')
    return config


def configuration_environment(config):
    def polygon(side):
        x, y, _ = config[side]['start']
        h = config[side]['start_area_half_size']
        return json.dumps([x-h, y-h, x+h, y-h, x+h, y+h, x-h, y+h], separators=(',', ':'))
    origin = config['simulation']['map_origin_world']
    first, second = config['robot']['start'], config['opponent']['start']
    role = config['robot']['role']
    return {key: str(value).lower() if type(value) is bool else str(value)
            for key, value in {
        'MAP': config['simulation']['world'], 'MAP_ORIGIN_X': origin[0], 'MAP_ORIGIN_Y': origin[1],
        'SPAWN_X': first[0]+origin[0], 'DUEL_SPAWN_Y': first[1]+origin[1], 'DUEL_SPAWN_YAW': first[2],
        'OPPONENT_X': second[0]+origin[0], 'OPPONENT_Y': second[1]+origin[1], 'OPPONENT_YAW': second[2],
        'DUEL_FIRST_START': polygon('robot'), 'DUEL_SECOND_START': polygon('opponent'),
        'DUEL_ARENA_BOUNDS': json.dumps(config['simulation']['arena_bounds'], separators=(',', ':')),
        'HSL_OPPONENT_MAX_HEIGHT': config['perception']['opponent_max_height'],
        'HSL_ROLE': role, 'HSL_OPPONENT_ROLE': 'guardian' if role == 'explorer' else 'explorer',
        'HSL_ALLOW_REVERSE': config['motion']['allow_reverse'], 'HSL_MAX_SPEED': config['motion']['max_speed'],
        'HSL_MAX_ANGULAR_SPEED': config['motion']['max_angular_speed'],
        'DUEL_SEED': config['match']['seed'], 'DUEL_OPPONENT_SEED': config['match']['seed']+1000003,
        'DUEL_MAX_ACTIVE_S': config['match']['active_seconds'],
        'GAZEBO_HEADLESS': config['simulation']['headless'], 'HSL_RVIZ_ENABLED': config['simulation']['rviz'],
    }.items()}
