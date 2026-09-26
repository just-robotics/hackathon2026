"""Drive the local path with main's MPC, protected by a navigation gate."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def config(name):
    return os.path.join(get_package_share_directory("swarm_controller"), "config", name)


def generate_launch_description():
    return LaunchDescription([
        Node(package="swarm_controller", executable="swarm_cc_mpc_node",
             name="hsl_cc_mpc", output="screen",
             parameters=[config("cc_mpc.param.yaml"), {
                 "start": True,
                 "use_sim_time": True,
                 "odom_topic": "/navigation/self",
                 "cmd_vel_topic": "/navigation/long_cmd",
             }]),
        Node(package="swarm_controller", executable="swarm_lat_mpc_node",
             name="hsl_lat_mpc", output="screen",
             parameters=[config("lat_mpc.param.yaml"), {
                 "long_cmd_topic": "/navigation/long_cmd",
                 "use_sim_time": True,
                 "cmd_vel_topic": "/navigation/mpc_cmd_vel",
                 "pacemaker_path_topic": "/navigation/local_path",
                 "pose_topic": "/navigation/self",
                 "odom_topic": "/navigation/self",
             }]),
        Node(package="hsl_debug_control", executable="mpc_gate",
             name="hsl_mpc_gate", output="screen",
             parameters=[{"use_sim_time": True}]),
    ])
