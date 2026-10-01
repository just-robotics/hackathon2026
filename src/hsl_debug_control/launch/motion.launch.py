"""Run the sole safety gate for direct MPPI commands."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_namespace", default_value=""),
        Node(package="hsl_debug_control", executable="motion_gate",
             namespace=LaunchConfiguration("robot_namespace"), name="hsl_motion_gate",
             parameters=[{"use_sim_time": True, "require_match_active": True}],
             output="screen"),
    ])
