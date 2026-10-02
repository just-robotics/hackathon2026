"""Open RViz on a laptop and show the robot topics. The display layout is robot.rviz."""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

here = Path(__file__).resolve().parent


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable("ROS_DOMAIN_ID", "26"),
        SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp"),
        SetEnvironmentVariable("CYCLONEDDS_URI", f"file://{here / 'cyclonedds.xml'}"),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", str(here / "robot.rviz")],
            output="screen",
        ),
    ])
