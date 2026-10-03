#!/usr/bin/env python3
"""Inspect installed real launch and detector dependencies. Does not launch ROS nodes."""
import importlib.util
import json
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
from launch import LaunchContext

path=Path(get_package_share_directory('hsl_real'))/'launch/robot.launch.py'
spec=importlib.util.spec_from_file_location('real_robot_launch',path)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
observed=[];original=module.Node

def capture(**kwargs):
    observed.append(kwargs)
    return original(**kwargs)

module.Node=capture
context=LaunchContext();context.launch_configurations.update(
    config_file='/config/real.yaml',drivers_enabled='false',session_dir='')
module.nodes(context)  # Constructs launch actions only; never executes them.
detectors=[n for n in observed if n['package']=='jr_perception' and n['executable']=='robot_detector.py']
assert len(detectors)==1 and detectors[0]['executable']=='robot_detector.py'
layers=detectors[0]['parameters'];detector={}
for layer in layers:
    detector.update(layer if isinstance(layer,dict) else yaml.safe_load(Path(layer).read_text())['/**']['ros__parameters'])
assert detector['cloud_topic']=='/perception/obstacle_cloud'
prefilters=[n for n in observed if n['executable']=='cloud_prefilter.py']
assert len(prefilters)==1 and prefilters[0]['parameters'][-1]['cloud_topic']=='/livox/lidar'
assert any(n['package']=='linefit_ground_segmentation_ros' for n in observed)
assert detector['pose_topic']=='navigation/self'
assert detector['world_frame']=='map' and detector['base_frame']=='base_footprint'
assert detector['background_topic']=='/map' and not detector['world']
assert detector['use_sim_time'] is False
native=next(n for n in observed if n['package']=='hsl_nav2_control')
assert native['parameters'][-1]['costmap.plugins']==['static_layer','inflation_layer']
packages=get_packages_with_prefixes()
assert 'hsl_perception' not in packages and 'hsl_perception_cpp' not in packages
from jr_perception.robot_detector import RobotDetector
from hsl_sim_adapter.pointcloud import cloud_xyz
print(json.dumps(dict(installed_real_launch=True, nodes_started=False,
    detector_input=detector['cloud_topic'], detector_pose=detector['pose_topic'],
    robot_output='opponent/odom', objects_output='opponent/markers',
    costmap_plugins=native['parameters'][-1]['costmap.plugins'],passed=True),indent=2))
