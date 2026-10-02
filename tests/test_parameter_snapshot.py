from pathlib import Path
import sys

import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'benchmarks'))
from ros_parameter_snapshot import nested_parameters


def test_ros_parameter_types_and_nested_plugin_names_are_preserved():
    names = ['use_sim_time', 'MPPI.vx_min', 'MPPI.GoalAngleCritic.enabled', 'start', 'role']
    values = [True, -0.5, False, [0.0, 1.0], 'explorer']
    assert nested_parameters(names, values) == {
        'use_sim_time': True, 'MPPI': {'vx_min': -0.5, 'GoalAngleCritic': {'enabled': False}},
        'start': [0.0, 1.0], 'role': 'explorer'}


def test_incomplete_snapshot_is_rejected():
    with pytest.raises(RuntimeError, match='Incomplete'):
        nested_parameters(['role'], [])
