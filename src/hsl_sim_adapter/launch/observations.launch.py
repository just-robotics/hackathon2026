"""Own simulator observations and independent shape-based LiDAR opponent detector."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    values = {name: LaunchConfiguration(name).perform(context) for name in
              ('robot_namespace', 'opponent_max_height', 'own_spawn_x', 'own_spawn_y', 'own_odom_topic',
               'own_truth_topic', 'lidar_topic')}
    params = dict(use_sim_time=True, own_spawn_x=float(values['own_spawn_x']),
                  own_spawn_y=float(values['own_spawn_y']),
                  **{key: values[key] for key in ('own_odom_topic', 'own_truth_topic', 'lidar_topic')})
    processes = [Node(package='hsl_sim_adapter', executable='sim_observations',
                      namespace=values['robot_namespace'], parameters=[params], output='screen'),
                 Node(package='hsl_perception_cpp', executable='opponent_detector',
                      namespace=values['robot_namespace'], parameters=[{'use_sim_time': True, 'opponent_max_height': float(values['opponent_max_height']), 'robot.max_gap_share': 0.12, 'robot.line_ratio': 0.35, 'strong_arc_min_span_deg': 90.0, 'allow_merged_strong': False, 'strong_min_inlier_fraction': 0.95, 'strong_rectangle_ratio': 0.70}], output='screen')]
    handlers = [RegisterEventHandler(OnProcessExit(target_action=process,
                on_exit=[EmitEvent(event=Shutdown(reason='Observation process exited'))]))
                for process in processes]
    return processes + handlers


def generate_launch_description():
    defaults = dict(opponent_max_height='0.46', robot_namespace='', own_spawn_x='-0.468', own_spawn_y='-0.582',
                    own_odom_topic='/odom', own_truth_topic='/localization/pose',
                    lidar_topic='/livox/lidar')
    return LaunchDescription([DeclareLaunchArgument(key, default_value=value)
                              for key, value in defaults.items()] + [OpaqueFunction(function=nodes)])
