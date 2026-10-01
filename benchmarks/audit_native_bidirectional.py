#!/usr/bin/env python3
"""Exercise the real native MPPI on synthetic free space in an isolated ROS domain.

This checks commands and capture-mode switching, not physical duel performance.
Run in the built image with ROS_DOMAIN_ID=73; no Gazebo or final cmd_vel publisher.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import statistics
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--role", choices=("explorer", "guardian"), default="explorer")
    role = parser.parse_args().role
    if os.environ.get("ROS_DOMAIN_ID") != "73":
        raise SystemExit("This synthetic audit requires isolated ROS_DOMAIN_ID=73")
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
    from hsl_interfaces.msg import PlanningIntent
    from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
    from std_msgs.msg import Bool, Header, String
    from tf2_ros import StaticTransformBroadcaster

    rclpy.init()
    node = rclpy.create_node("bidirectional_audit")
    ns = "/bidirectional_audit"
    retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pubs = {
        "map": node.create_publisher(OccupancyGrid, "/map", retained),
        "active": node.create_publisher(Bool, "/match/active", retained),
        "self": node.create_publisher(Odometry, ns + "/navigation/self", 10),
        "intent": node.create_publisher(PlanningIntent, ns + "/navigation/intent", 10),
        "path": node.create_publisher(RosPath, ns + "/navigation/nav2_reference", 10),
        "status": node.create_publisher(String, ns + "/navigation/global_status", 10),
        "scan": node.create_publisher(PointCloud2, ns + "/navigation/scan", 10),
    }
    tf = StaticTransformBroadcaster(node)
    transform = TransformStamped()
    transform.header.frame_id = "map"
    transform.header.stamp = node.get_clock().now().to_msg()
    transform.child_frame_id = "bidirectional_audit/base_footprint"
    transform.transform.rotation.w = 1.0
    tf.sendTransform(transform)
    grid = OccupancyGrid()
    grid.header = Header(frame_id="map")
    grid.info.resolution = 0.05
    grid.info.width = grid.info.height = 160
    grid.info.origin.position.x = grid.info.origin.position.y = -4.0
    grid.info.origin.orientation.w = 1.0
    grid.data = [0] * (160 * 160)
    pubs["map"].publish(grid)
    ready = {"value": False}
    commands, diagnostics = [], []
    subscriptions = [
        node.create_subscription(Bool, ns + "/navigation/native_ready",
                                 lambda msg: ready.update(value=msg.data), retained),
        node.create_subscription(Twist, ns + "/navigation/mppi_cmd_vel",
                                 lambda msg: commands.append((msg.linear.x, msg.angular.z)), 10),
        node.create_subscription(String, ns + "/navigation/mppi_diagnostics",
                                 lambda msg: diagnostics.append(json.loads(msg.data)), 10),
    ]
    state = {"behavior": 6, "x": -1.0, "y": 0.0, "yaw": 0.0}

    def publish():
        stamp = node.get_clock().now().to_msg()
        own = Odometry()
        own.header = Header(stamp=stamp, frame_id="map")
        own.child_frame_id = transform.child_frame_id
        own.pose.pose.orientation.w = 1.0
        pubs["self"].publish(own)
        intent = PlanningIntent()
        intent.header = own.header
        intent.behavior = state["behavior"]
        intent.has_target = True
        intent.max_speed = 1.0
        intent.target.position.x, intent.target.position.y = state["x"], state["y"]
        intent.target.orientation.z = math.sin(state["yaw"] / 2)
        intent.target.orientation.w = math.cos(state["yaw"] / 2)
        pubs["intent"].publish(intent)
        path = RosPath(header=own.header)
        for i in range(21):
            point = PoseStamped(header=own.header)
            point.pose.position.x = state["x"] * i / 20
            point.pose.position.y = state["y"] * i / 20
            point.pose.orientation.w = 1.0
            if i == 20:
                point.pose.orientation = intent.target.orientation
            path.poses.append(point)
        pubs["path"].publish(path)
        pubs["status"].publish(String(data="OK"))
        pubs["scan"].publish(create_cloud_xyz32(own.header, []))
        pubs["active"].publish(Bool(data=True))

    timer = node.create_timer(0.05, publish)
    config = str(Path(get_package_share_directory("hsl_nav2_control")) /
                 "config/native_mppi.yaml")
    log = open("/tmp/bidirectional-audit-native.log", "w")
    process = subprocess.Popen([
        "ros2", "run", "hsl_nav2_control", "native_mppi", "--ros-args",
        "-r", "__ns:=" + ns, "--params-file", config,
        "-p", "use_sim_time:=false", "-p", "role:=" + role,
        "-p", "MPPI.vx_min:=" + ("-0.5" if role == "explorer" else "0.0"),
        "-p", "MPPI.PathAngleCritic.forward_preference:=" + ("false" if role == "explorer" else "true"),
        "-p", "random_seed:=19", "-p", "MPPI.GoalCritic.cost_weight:=" + ("15.0" if role == "guardian" else "5.0")],
        stdout=log, stderr=subprocess.STDOUT)
    results = []
    try:
        deadline = time.monotonic() + 40
        while not ready["value"] and time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError("Native MPPI exited before ready; inspect native log")
            rclpy.spin_once(node, timeout_sec=0.1)
        if not ready["value"]:
            raise TimeoutError("Native MPPI not ready on synthetic free map")
        stages = [
                ("reverse", 6, -1.0, 0.0, 0.0),
                ("forward", 6, 1.0, 0.0, 0.0),
                ("capture_face_left", 7, 0.0, 0.08, math.pi / 2),
                ("resume_reverse", 6, -1.0, 0.0, 0.0)]
        for name, behavior, x, y, yaw in stages:
            if role == "explorer" and behavior == 7:
                continue
            state.update(behavior=behavior, x=x, y=y, yaw=yaw)
            # Exclude the transport/mode transition; retain the steady commands.
            warmup = time.monotonic() + 1.0
            while time.monotonic() < warmup:
                rclpy.spin_once(node, timeout_sec=0.05)
            commands.clear(); diagnostics.clear()
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            ok = [d for d in diagnostics if d.get("result") == "ok"]
            median_v = statistics.median(v for v, _ in commands) if commands else 0.0
            median_w = statistics.median(w for _, w in commands) if commands else 0.0
            capture = role == "guardian" and behavior == 7
            heading_ok = bool(ok) and all(d.get("capture_heading_required") == capture for d in ok)
            direction_ok = (median_w > 0.05 if capture else
                            (median_v < -0.05 if role == "explorer" else
                             all(v >= -1e-6 for v, _ in commands) and any(abs(w) > 0.05 for _, w in commands))
                            if x < 0 else median_v > 0.05)
            results.append(dict(stage=name, median_v_mps=median_v, median_w_radps=median_w,
                                command_count=len(commands), ok_count=len(ok),
                                heading_mode_ok=heading_ok,
                                passed=heading_ok and direction_ok and len(commands) >= 10))
        print(json.dumps({"synthetic_free_space_only": True, "domain_id": 73, "role": role,
                          "stages": results, "passed": all(r["passed"] for r in results)}, indent=2))
        return 0 if all(r["passed"] for r in results) else 1
    finally:
        process.send_signal(signal.SIGINT) if process.poll() is None else None
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            process.kill(); process.wait()
        log.close()
        node.destroy_timer(timer)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
