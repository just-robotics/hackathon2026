from pathlib import Path
import sys
import yaml
import pytest
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/hsl_planning'))
from hsl_planning.configuration import load_planning


def test_external_config_shares_hard_radius_and_inflation():
    cfg=load_planning(ROOT/'config/planning.yaml')
    assert cfg['global']['robot_radius']==cfg['local']['costmap.robot_radius']
    assert cfg['local']['MPPI.ObstaclesCritic.inflation_radius']==cfg['local']['costmap.inflation_layer.inflation_radius']


@pytest.mark.parametrize('change',[
    {'global':{'robot_radius':.17}}, {'global':{'resolution':0}},
    {'global':{'pose_timeout':0}}, {'global':{'robot_radius':True}},
    {'local':{'costmap.inflation_layer.inflation_radius':.6}},
    {'local':{'MPPI.ObstaclesCritic.repulsion_weight':float('nan')}},
    {'global':{'misspelled_radius':.2}},
    {'local':{'MPPI.time_steps':30.5}},
    {'local':{'MPPI.time_steps':10}},
    {'local':{'MPPI.time_steps':True}},
])
def test_bad_planner_config_is_rejected(tmp_path,change):
    p=tmp_path/'planning.yaml';p.write_text(yaml.safe_dump(change))
    with pytest.raises(ValueError):load_planning(p)


def test_smaller_clearance_can_be_requested_without_rebuild(tmp_path):
    p=tmp_path/'planning.yaml';p.write_text('global:\n  robot_radius: 0.21\n')
    cfg=load_planning(p)
    assert cfg['global']['robot_radius']==cfg['local']['costmap.robot_radius']==.21


def test_horizon_can_be_changed_without_losing_ros_integer_type(tmp_path):
    p=tmp_path/'planning.yaml';p.write_text('local:\n  MPPI.time_steps: 30\n')
    cfg=load_planning(p)
    assert type(cfg['local']['MPPI.time_steps']) is int
    assert cfg['local']['MPPI.time_steps']==30


def test_native_yaml_fallback_matches_external_soft_clearance_defaults():
    native=yaml.safe_load((ROOT/'src/hsl_nav2_control/config/native_mppi.yaml').read_text())['/**']['ros__parameters']
    effective=load_planning()
    for key,value in effective['local'].items():
        parts=key.split('.')
        current=native
        for part in parts:
            current=current[part]
        assert current==value, key


def test_stuck_recovery_defaults_are_present_and_positive():
    cfg=load_planning(ROOT/'config/planning.yaml')['global']
    assert cfg['stuck_hold_s']>0 and cfg['phantom_distance']>0 and cfg['phantom_ttl_s']>0
