#!/usr/bin/env python3
"""Audit installed launch wiring, without a simulator or publishing ROS commands."""
import importlib.util
import json
from pathlib import Path
from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
from launch import LaunchContext

path = Path(get_package_share_directory('hsl_planning')) / 'launch/autonomous.launch.py'
spec = importlib.util.spec_from_file_location('autonomous', path)
launch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch)
original = launch.Node
observed = []

def capture(**kwargs):
    observed.append(kwargs)
    return original(**kwargs)

launch.Node = capture
results = []
for reverse in (True, False):
    for role in ('explorer', 'guardian'):
        observed.clear()
        context = LaunchContext()
        context.launch_configurations.update(robot_namespace='audit', role=role, random_seed='29',
            arena_bounds='[-3,-3,3,3]', allow_reverse=str(reverse).lower(),
            max_speed='0.7', max_angular_speed='1.2')
        launch.nodes(context)
        planner = next(n for n in observed if n['executable'] == 'trajectory_planner')['parameters'][0]
        native = next(n for n in observed if n['executable'] == 'native_mppi')['parameters'][1]
        assert planner['max_speed'] == 0.7 and planner['role'] == role
        assert native['MPPI.vx_max'] == 0.7
        effective_reverse = reverse and role == 'explorer'
        assert native['MPPI.vx_min'] == (-0.7 if effective_reverse else 0.0)
        assert native['MPPI.wz_max'] == 1.2
        assert native['MPPI.PathAngleCritic.forward_preference'] == (not effective_reverse)
        assert native['MPPI.GoalAngleCritic.enabled'] is False
        assert native['MPPI.GoalCritic.cost_weight'] == (15.0 if role == 'guardian' else 5.0)
        results.append(dict(role=role, allow_reverse=reverse, native_parameters=native))
assert 'swarm_controller' not in get_packages_with_prefixes()
assert not Path('/autoware/src/hsl_planning/hsl_planning/mppi.py').exists()
assert not Path('/autoware/src/hsl_debug_control/hsl_debug_control/node.py').exists()
assert (Path(get_package_share_directory('hsl_debug_control'))/'launch/motion.launch.py').exists()
print(json.dumps(dict(installed_launch_only=True, cases=results, passed=True), indent=2))
