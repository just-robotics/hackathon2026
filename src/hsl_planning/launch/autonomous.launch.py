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
from hsl_planning.backend import resolve_backend


def nodes(context):
    values = {name: LaunchConfiguration(name).perform(context) for name in (
        "robot_namespace", "role", "random_seed", "arena_bounds", "control_mode",
        "mpc_path_source", "local_backend")}
    backend = resolve_backend(values["control_mode"], values["local_backend"])
    values["local_backend"] = backend
    params = {key: values[key] for key in (
        "role", "control_mode", "mpc_path_source", "local_backend")}
    params.update(use_sim_time=True, require_match_active=True, random_seed=int(values["random_seed"]),
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
                               # Capture requires the guardian to face its prey;
                               # explorer evasion can follow a route in reverse.
                               "MPPI.PathAngleCritic.forward_preference":
                                   values["role"] == "guardian",
                               "random_seed": int(values["random_seed"])}], output="screen"))
    for process in list(result):
        result.append(RegisterEventHandler(OnProcessExit(
            target_action=process, on_exit=[EmitEvent(event=Shutdown(
                reason="Autonomous planning process exited"))])))
    return result


def generate_launch_description():
    defaults = {"robot_namespace": "", "role": "explorer", "random_seed": "0",
                "arena_bounds": "[-2.66,-0.4,3.34,4.6]", "control_mode": "mppi",
                "mpc_path_source": "local", "local_backend": "auto"}
    return LaunchDescription([DeclareLaunchArgument(name, default_value=value)
                              for name, value in defaults.items()] + [OpaqueFunction(function=nodes)])
