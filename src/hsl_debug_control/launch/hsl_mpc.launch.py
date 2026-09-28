"""Drive the local path with main's MPC, protected by a navigation gate."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def config(name):
    return os.path.join(get_package_share_directory("swarm_controller"), "config", name)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_namespace", default_value=""),
        OpaqueFunction(function=control_nodes),
    ])


def control_nodes(context):
    namespace = LaunchConfiguration("robot_namespace").perform(context).strip("/")
    prefix = f"/{namespace}" if namespace else ""
    return [
        Node(package="swarm_controller", executable="swarm_cc_mpc_node",
             namespace=namespace, name="hsl_cc_mpc", output="screen",
             parameters=[config("cc_mpc.param.yaml"), {
                 "start": True,
                 "use_sim_time": True,
                 "v_ref": 0.3,
                 "v_cmd_max": 0.3,
                 "odom_topic": prefix + "/navigation/self",
                 "cmd_vel_topic": prefix + "/navigation/long_cmd",
             }]),
        Node(package="swarm_controller", executable="swarm_lat_mpc_node",
             namespace=namespace, name="hsl_lat_mpc", output="screen",
             parameters=[config("lat_mpc.param.yaml"), {
                 "a_lat_max": 0.12,
                 "v_min": 0.2,
                 "curve_lookahead": 1.0,
                 "long_cmd_topic": prefix + "/navigation/long_cmd",
                 "use_sim_time": True,
                 "cmd_vel_topic": prefix + "/navigation/mpc_cmd_vel",
                 "pacemaker_path_topic": prefix + "/navigation/local_path",
                 "pose_topic": prefix + "/navigation/self",
                 "odom_topic": prefix + "/navigation/self",
             }]),
        Node(package="hsl_debug_control", executable="mpc_gate",
             namespace=namespace, name="hsl_mpc_gate", output="screen",
             parameters=[{"use_sim_time": True}]),
    ]
