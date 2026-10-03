"""Own simulator observations and independent shape-based LiDAR opponent detector."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    values = {name: LaunchConfiguration(name).perform(context) for name in
              ('robot_namespace', 'own_spawn_x', 'own_spawn_y', 'own_odom_topic',
               'own_truth_topic', 'lidar_topic')}
    params = dict(use_sim_time=True, own_spawn_x=float(values['own_spawn_x']),
                  own_spawn_y=float(values['own_spawn_y']),
                  **{key: values[key] for key in ('own_odom_topic', 'own_truth_topic', 'lidar_topic')})
    processes = [Node(package='hsl_sim_adapter', executable='sim_observations',
                      namespace=values['robot_namespace'], parameters=[params], output='screen'),
                 Node(package='jr_perception', executable='robot_detector.py',
                      namespace=values['robot_namespace'], parameters=[str(Path(get_package_share_directory('jr_perception'))/'config/simulation.yaml'), {
                          'use_sim_time': True, 'world':'', 'background_topic':'/map',
                          'world_frame':'map', 'pose_topic':'navigation/self',
                          'base_frame':((values['robot_namespace'].strip('/')+'/') if values['robot_namespace'].strip('/') else '')+'base_footprint',
                          'cloud_topic': values['lidar_topic']}], output='screen')]
    handlers = [RegisterEventHandler(OnProcessExit(target_action=process,
                on_exit=[EmitEvent(event=Shutdown(reason='Observation process exited'))]))
                for process in processes]
    return processes + handlers


def generate_launch_description():
    defaults = dict(robot_namespace='', own_spawn_x='-0.468', own_spawn_y='-0.582',
                    own_odom_topic='/odom', own_truth_topic='/localization/pose',
                    lidar_topic='/livox/lidar')
    return LaunchDescription([DeclareLaunchArgument(key, default_value=value)
                              for key, value in defaults.items()] + [OpaqueFunction(function=nodes)])
