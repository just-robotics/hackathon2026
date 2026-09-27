"""Two Kobuki models in Gazebo Classic with separate ROS topics and TF frames."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("sim_kobuki")
    gazebo_share = get_package_share_directory("gazebo_ros")
    model = os.path.join(share, "description", "kobuki.urdf.xacro")
    world = PathJoinSubstitution([share, "worlds", [LaunchConfiguration("map"), ".world"]])
    lidar_args = [" lidar_horizontal_samples:=", LaunchConfiguration("lidar_horizontal_samples"),
                  " lidar_vertical_samples:=", LaunchConfiguration("lidar_vertical_samples")]
    own_description = ParameterValue(Command(["xacro ", model, *lidar_args]), value_type=str)
    opponent_description = ParameterValue(
        Command(["xacro ", model,
                 " ros_namespace:=/opponent frame_prefix:=opponent/", *lidar_args]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument("map", default_value="maze"),
        DeclareLaunchArgument("headless", default_value="true"),
        DeclareLaunchArgument("lidar_horizontal_samples", default_value="360"),
        DeclareLaunchArgument("lidar_vertical_samples", default_value="16"),
        DeclareLaunchArgument("spawn_x", default_value="-0.34"),
        DeclareLaunchArgument("spawn_y", default_value="-0.18"),
        DeclareLaunchArgument("spawn_z", default_value="0.23"),
        DeclareLaunchArgument("opponent_x", default_value="2.5"),
        DeclareLaunchArgument("opponent_y", default_value="2.5"),
        DeclareLaunchArgument("opponent_z", default_value="0.23"),
        DeclareLaunchArgument("opponent_yaw", default_value="3.14159"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             name="robot_state_publisher", parameters=[{
                 "robot_description": own_description, "use_sim_time": True}], output="screen"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             namespace="opponent", name="robot_state_publisher",
             remappings=[("/opponent/tf", "/tf"), ("/opponent/tf_static", "/tf_static")],
             parameters=[{"robot_description": opponent_description,
                          "frame_prefix": "opponent/", "use_sim_time": True}], output="screen"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, "launch", "gzserver.launch.py")),
            launch_arguments={"world": world, "verbose": "true", "init": "true",
                              "factory": "true", "force_system": "false"}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, "launch", "gzclient.launch.py")),
            condition=UnlessCondition(LaunchConfiguration("headless"))),
        TimerAction(period=8.0, actions=[
            Node(package="gazebo_ros", executable="spawn_entity.py",
                 arguments=["-topic", "robot_description", "-entity", "kobuki",
                            "-x", LaunchConfiguration("spawn_x"),
                            "-y", LaunchConfiguration("spawn_y"),
                            "-z", LaunchConfiguration("spawn_z")], output="screen"),
            Node(package="gazebo_ros", executable="spawn_entity.py",
                 arguments=["-topic", "/opponent/robot_description", "-entity", "opponent",
                            "-x", LaunchConfiguration("opponent_x"),
                            "-y", LaunchConfiguration("opponent_y"),
                            "-z", LaunchConfiguration("opponent_z"),
                            "-Y", LaunchConfiguration("opponent_yaw")], output="screen"),
        ]),
        OpaqueFunction(function=world_odom_transforms),
    ])


def world_odom_transforms(context):
    """Gazebo Classic odometry is world-based; map is shifted to own spawn."""
    own_x = float(LaunchConfiguration("spawn_x").perform(context))
    own_y = float(LaunchConfiguration("spawn_y").perform(context))
    return [Node(package="tf2_ros", executable="static_transform_publisher",
                 arguments=["--x", str(-own_x), "--y", str(-own_y),
                            "--z", "0", "--yaw", "0", "--pitch", "0", "--roll", "0",
                            "--frame-id", "map", "--child-frame-id", child],
                 output="screen") for child in ("odom", "opponent/odom")]
