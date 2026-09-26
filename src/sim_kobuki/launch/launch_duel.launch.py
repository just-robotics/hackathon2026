"""Two namespaced Kobuki models in the existing world for navigation tests."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("sim_kobuki")
    gz_share = get_package_share_directory("ros_gz_sim")
    world = LaunchConfiguration("world")
    headless = LaunchConfiguration("headless")
    model = os.path.join(share, "description", "kobuki.urdf.xacro")
    own_description = ParameterValue(Command(["xacro ", model]), value_type=str)
    opponent_description = ParameterValue(
        Command(["xacro ", model, " topic_prefix:=opponent/ frame_prefix:=opponent/"]),
        value_type=str,
    )

    def gazebo(args, condition):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gz_share, "launch", "gz_sim.launch.py")),
            launch_arguments={"gz_args": [args, world], "on_exit_shutdown": "true"}.items(),
            condition=condition,
        )

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value=os.path.join(share, "worlds", "empty.sdf")),
        DeclareLaunchArgument("headless", default_value="false", choices=["true", "false"]),
        gazebo("-r -s -v 4 ", IfCondition(headless)),
        gazebo("-r -v 4 ", UnlessCondition(headless)),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             name="robot_state_publisher_self", parameters=[{
                 "robot_description": own_description, "use_sim_time": True}], output="screen"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             namespace="opponent", name="robot_state_publisher_opponent",
             remappings=[("/opponent/tf", "/tf"), ("/opponent/tf_static", "/tf_static")],
             parameters=[{"robot_description": opponent_description,
                          "frame_prefix": "opponent/", "use_sim_time": True}], output="screen"),
        Node(package="ros_gz_sim", executable="create",
             arguments=["-topic", "robot_description", "-name", "kobuki", "-z", "0.05"],
             output="screen"),
        Node(package="ros_gz_sim", executable="create",
             arguments=["-topic", "/opponent/robot_description", "-name", "opponent",
                        "-x", "4.0", "-y", "-2.0", "-z", "0.05", "-Y", "3.14159"],
             output="screen"),
        Node(package="ros_gz_bridge", executable="parameter_bridge",
             parameters=[{"config_file": os.path.join(share, "config", "gz_bridge_duel.yaml"),
                          "use_sim_time": True}], output="screen"),
        Node(package="tf2_ros", executable="static_transform_publisher",
             arguments=["0", "0", "0", "0", "0", "0", "map", "odom"], output="screen"),
        Node(package="tf2_ros", executable="static_transform_publisher",
             arguments=["4", "-2", "0", "3.14159", "0", "0", "map", "opponent/odom"],
             output="screen"),
    ])
