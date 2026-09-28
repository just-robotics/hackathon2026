import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("sim_kobuki")
    gazebo_ros_share = get_package_share_directory("gazebo_ros")

    # мир выбирается по имени карты (MAP в .env): worlds/<map>.world
    world = PathJoinSubstitution(
        [package_share, "worlds", [LaunchConfiguration("map"), ".world"]]
    )
    headless = LaunchConfiguration("headless")
    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                os.path.join(package_share, "description", "kobuki.urdf.xacro"),
                " lidar:=",
                LaunchConfiguration("lidar"),
            ]
        ),
        value_type=str,
    )

    # Gazebo Classic разнесен на два процесса: gzserver считает физику,
    # gzclient рисует окно. В headless-режиме поднимаем только сервер.
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, "launch", "gzserver.launch.py")
        ),
        launch_arguments={
            "world": world,
            "verbose": "true",
            # плагины ROS-моста грузит сам gzserver
            "init": "true",
            "factory": "true",
            "force_system": "false",
        }.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, "launch", "gzclient.launch.py")
        ),
        condition=UnlessCondition(headless),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                default_value="maze",
                description="World name from worlds/, without the .world suffix.",
            ),
            DeclareLaunchArgument(
                "spawn_x",
                default_value="-0.34",
                description="Robot spawn X, world frame.",
            ),
            DeclareLaunchArgument(
                "spawn_y",
                default_value="-0.18",
                description="Robot spawn Y, world frame.",
            ),
            DeclareLaunchArgument(
                "spawn_z",
                default_value="0.23",
                description="Robot spawn Z, world frame.",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                choices=["true", "false"],
                description="Run only the Gazebo server when true.",
            ),
            DeclareLaunchArgument(
                "lidar",
                default_value="false",
                choices=["true", "false"],
                description=(
                    "Attach the Livox lidar sensor. Off by default: ray "
                    "casting runs on the CPU in Gazebo Classic and costs RTF."
                ),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[
                    {
                        "robot_description": robot_description,
                        "use_sim_time": True,
                    }
                ],
                output="screen",
            ),
            gzserver,
            gzclient,
            # Спавн отложен: spawn_entity.py обращается к сервису gzserver и
            # без задержки успевает стартовать раньше него -- сервис ещё не
            # поднят, и процесс падает с "Spawn service failed".
            TimerAction(
                period=8.0,
                actions=[
                    Node(
                        package="gazebo_ros",
                        executable="spawn_entity.py",
                        arguments=[
                            "-topic",
                            "robot_description",
                            "-entity",
                            "kobuki",
                            "-x",
                            LaunchConfiguration("spawn_x"),
                            "-y",
                            LaunchConfiguration("spawn_y"),
                            "-z",
                            LaunchConfiguration("spawn_z"),
                        ],
                        output="screen",
                    ),
                ],
            ),
            # ros_gz_bridge не нужен: в Classic плагины gazebo_ros публикуют
            # в ROS напрямую, а /clock отдает gazebo_ros_init внутри gzserver.
        ]
    )
