"""Role-aware global planner and native Nav2 MPPI."""
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
        "robot_namespace", "role", "random_seed", "arena_bounds", "allow_reverse", "max_speed", "max_angular_speed")}
    params = {"role": values["role"]}
    allow_reverse = values["allow_reverse"].lower() == "true"
    speed = float(values["max_speed"])
    angular = float(values["max_angular_speed"])
    params.update(max_speed=speed, use_sim_time=True, require_match_active=True, random_seed=int(values["random_seed"]),
                  arena_bounds=[float(v) for v in json.loads(values["arena_bounds"])])
    planner = Node(package="hsl_planning", executable="trajectory_planner",
                   namespace=values["robot_namespace"], parameters=[params], output="screen")
    result = [planner]
    config = os.path.join(get_package_share_directory("hsl_nav2_control"),
                          "config", "native_mppi.yaml")
    result.append(Node(package="hsl_nav2_control", executable="native_mppi",
                       namespace=values["robot_namespace"], parameters=[config, {
                           "role": values["role"],
                           # Both travel directions are equal. Native MPPI
                           # enables endpoint yaw only for guardian capture.
                           "MPPI.vx_max": speed, "MPPI.vx_min": -speed if allow_reverse else 0.0,
                           "MPPI.wz_max": angular,
                           "MPPI.PathAngleCritic.forward_preference": not allow_reverse,
                           "MPPI.PreferForwardCritic.enabled": False,
                           "MPPI.GoalAngleCritic.enabled": False,
                           # A moving capture goal needs stronger positional
                           # tracking near the endpoint; keep explorer tuning.
                           "MPPI.GoalCritic.cost_weight":
                               15.0 if values["role"] == "guardian" else 5.0,
                           "random_seed": int(values["random_seed"])}], output="screen"))
    for process in list(result):
        result.append(RegisterEventHandler(OnProcessExit(
            target_action=process, on_exit=[EmitEvent(event=Shutdown(
                reason="Autonomous planning process exited"))])))
    return result


def generate_launch_description():
    defaults = {"robot_namespace": "", "role": "explorer", "random_seed": "0",
                "arena_bounds": "[-2.66,-0.4,3.34,4.6]",
                "allow_reverse": "true",
                "max_speed": "0.5", "max_angular_speed": "1.5"}
    return LaunchDescription([DeclareLaunchArgument(name, default_value=value)
                              for name, value in defaults.items()] + [OpaqueFunction(function=nodes)])
