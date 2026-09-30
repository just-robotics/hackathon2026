"""Role-aware global planner with selectable local control backend."""
import json
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    values = {name: LaunchConfiguration(name).perform(context) for name in (
        "robot_namespace", "role", "random_seed", "arena_bounds", "control_mode",
        "mpc_path_source", "local_backend")}
    backend = values["local_backend"]
    if backend not in ("python", "nav2_cpp"):
        raise ValueError("local_backend must be python or nav2_cpp")
    if backend == "nav2_cpp" and values["control_mode"] != "mppi":
        raise ValueError("nav2_cpp requires control_mode=mppi")
    params = {key: values[key] for key in (
        "role", "control_mode", "mpc_path_source", "local_backend")}
    params.update(use_sim_time=True, random_seed=int(values["random_seed"]),
                  arena_bounds=[float(v) for v in json.loads(values["arena_bounds"])])
    planner = Node(package="hsl_planning", executable="trajectory_planner",
                   namespace=values["robot_namespace"], parameters=[params], output="screen")
    result = [planner]
    if backend == "nav2_cpp":
        config = os.path.join(get_package_share_directory("hsl_nav2_control"),
                              "config", "native_mppi.yaml")
        result.append(Node(package="hsl_nav2_control", executable="native_mppi",
                           namespace=values["robot_namespace"], parameters=[config, {
                               "role": values["role"],
                               "random_seed": int(values["random_seed"])}], output="screen"))
    for process in list(result):
        result.append(RegisterEventHandler(OnProcessExit(
            target_action=process, on_exit=[EmitEvent(event=Shutdown(
                reason="Autonomous planning process exited"))])))
    return result


def generate_launch_description():
    defaults = {"robot_namespace": "", "role": "explorer", "random_seed": "0",
                "arena_bounds": "[-2.66,-0.4,3.34,4.6]", "control_mode": "mppi",
                "mpc_path_source": "local", "local_backend": "python"}
    return LaunchDescription([DeclareLaunchArgument(name, default_value=value)
                              for name, value in defaults.items()] + [OpaqueFunction(function=nodes)])
