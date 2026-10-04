#!/usr/bin/env python3
"""Construct real launch actions in every localization mode without starting ROS."""
import argparse
import copy
import importlib.util
import json
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext


def main():
    workspace = Path(__file__).resolve().parents[1]
    source_launch = workspace / 'src/hsl_real/launch/robot.launch.py'
    source_perception = workspace / 'src/jr_perception'
    source_config = workspace / 'config/real.yaml'
    parser = argparse.ArgumentParser()
    parser.add_argument('--launch', type=Path,
                        default=source_launch if source_launch.is_file() else
                        Path(get_package_share_directory('hsl_real')) / 'launch/robot.launch.py')
    parser.add_argument('--perception-share', type=Path,
                        default=source_perception if source_perception.is_dir() else
                        Path(get_package_share_directory('jr_perception')))
    parser.add_argument('--config', type=Path,
                        default=source_config if source_config.is_file() else Path('/config/real.yaml'))
    args = parser.parse_args()

    spec = importlib.util.spec_from_file_location('real_robot_launch', args.launch)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    original_get_share = module.get_package_share_directory
    module.get_package_share_directory = lambda package: (
        str(args.perception_share) if package == 'jr_perception'
        else str(workspace / 'src' / package)
        if (workspace / 'src' / package).is_dir()
        else original_get_share(package))
    original_load_config = module.load_config
    cfg, mission = original_load_config(str(args.config))
    # Hardware actions require installed robot description packages. The audit
    # checks only stack wiring, so leave those actions out without starting any.
    module.hardware_nodes = lambda *_args, **_kwargs: []
    observed = []
    original_node = module.Node

    def capture(**kwargs):
        observed.append(kwargs)
        return original_node(**kwargs)

    module.Node = capture
    report = []
    for mode in ('fastlio', 'amcl', 'odometry', 'external_tf'):
        mode_cfg = dict(cfg, localization=mode)
        module.load_config = lambda _path: (mode_cfg, copy.deepcopy(mission))
        observed.clear()
        context = LaunchContext()
        context.launch_configurations.update(
            config_file=str(args.config), drivers_enabled='false', session_dir='')
        module.nodes(context)  # Only constructs actions; does not start processes.
        detectors = [node for node in observed if node['package'] == 'jr_perception'
                     and node['executable'] in ('robot_detector.py', 'opponent_detector_cpp')]
        assert len(detectors) == 1, (mode, detectors)
        detector_node = detectors[0]
        assert detector_node['executable'] == 'robot_detector.py', (mode, detector_node)
        params = {}
        for layer in detector_node['parameters']:
            if isinstance(layer, dict):
                params.update(layer)
            else:
                profile = yaml.safe_load(Path(layer).read_text())
                params.update(profile['/**']['ros__parameters'])
        assert params['use_sim_time'] is False
        assert params['cloud_topic'] == '/sensing/lidar/dbscan/pointcloud'
        expected_pose = '/localization/kinematic_state' if mode == 'fastlio' else '/navigation/self'
        assert params['pose_topic'] == expected_pose, (mode, params['pose_topic'])
        assert params['map_topic'] == '/map'
        assert params['world_frame'] == 'map' and params['base_frame'] == 'base_footprint'
        assert params['tracker']['reacquire_time'] == 3.0
        assert params['tracker']['switch_after'] == 1.0
        native = next(node for node in observed if node['package'] == 'hsl_nav2_control')
        assert native['parameters'][-1]['costmap.plugins'] == ['static_layer', 'inflation_layer']
        report.append({'mode': mode, 'detector': detector_node['executable'],
                       'cloud': params['cloud_topic'], 'pose': params['pose_topic'],
                       'detector_count': len(detectors)})
    print(json.dumps({'launch_constructed': True, 'nodes_started': False,
                      'real_detector': report}, indent=2))


if __name__ == '__main__':
    main()
