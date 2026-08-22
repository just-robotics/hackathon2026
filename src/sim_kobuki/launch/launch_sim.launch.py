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
    package_share = get_package_share_directory("sim_kobuki")
    ros_gz_share = get_package_share_directory("ros_gz_sim")

    world = LaunchConfiguration("world")
    headless = LaunchConfiguration("headless")
    robot_description = ParameterValue(
        Command(["xacro ", os.path.join(package_share, "description", "kobuki.urdf.xacro")]),
        value_type=str,
    )

    common_gz_arguments = {
        "on_exit_shutdown": "true",
    }

    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            **common_gz_arguments,
            "gz_args": ["-r -s -v 4 ", world],
        }.items(),
        condition=IfCondition(headless),
    )

    gazebo_with_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            **common_gz_arguments,
            "gz_args": ["-r -v 4 ", world],
        }.items(),
        condition=UnlessCondition(headless),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=os.path.join(package_share, "worlds", "empty.sdf"),
                description="Absolute path to an SDF world.",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                choices=["true", "false"],
                description="Run only the Gazebo server when true.",
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
            gazebo_headless,
            gazebo_with_gui,
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=[
                    "-topic",
                    "robot_description",
                    "-name",
                    "kobuki",
                    "-z",
                    "0.05",
                ],
                output="screen",
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                parameters=[
                    {
                        "config_file": os.path.join(
                            package_share, "config", "gz_bridge.yaml"
                        ),
                        "use_sim_time": True,
                    }
                ],
                output="screen",
            ),
        ]
    )
