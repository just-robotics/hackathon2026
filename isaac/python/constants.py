"""Shared paths and Kobuki / Livox parameters for the Isaac Sim scene."""

import os

ISAAC_ROOT = os.environ.get("HACKATHON_ISAAC_ROOT", "/workspace/isaac")
URDF_PATH = os.path.join(ISAAC_ROOT, "robots", "turtlebot2", "urdf", "turtlebot2.urdf")
USD_CACHE_DIR = os.environ.get(
    "HACKATHON_USD_CACHE", "/isaac-sim/.cache/hackathon/turtlebot2"
)
USD_CACHE_PATH = os.path.join(USD_CACHE_DIR, "turtlebot2.usd")

SIMPLE_ROOM_REL = "/Isaac/Environments/Simple_Room/simple_room.usd"

ROBOT_PRIM = "/World/turtlebot2"
GRAPH_DRIVE = "/Graphs/ROS_Drive"
GRAPH_LIDAR = "/Graphs/ROS_Lidar"
GRAPH_TF = "/Graphs/ROS_TF"
GRAPH_CLOCK = "/Graphs/ROS_Clock"

WHEEL_RADIUS = 0.035
WHEEL_DISTANCE = 0.230
MAX_LINEAR_SPEED = 0.70
MAX_ANGULAR_SPEED = 3.14
WHEEL_JOINTS = ["wheel_left_joint", "wheel_right_joint"]

CMD_VEL_TOPIC = "cmd_vel"
LIDAR_TOPIC = "livox/lidar"
ODOM_TOPIC = "odom"
LIDAR_FRAME = "livox_frame"
ODOM_FRAME = "odom"
BASE_FRAME = "base_link"

LIDAR_SCAN_HZ = 10.0
LIDAR_NEAR_M = 0.10
LIDAR_FAR_M = 40.0
LIDAR_CHANNELS = 32
# Livox Mid-360 vertical FOV in the sensor frame. The mount is inverted, so in
# world coordinates this becomes -52..+7 deg (floor plus a little above horizon).
LIDAR_ELEVATION_MIN_DEG = -7.0
LIDAR_ELEVATION_MAX_DEG = 52.0
# Example_Rotary fires at 36 kHz; a third of that keeps the viewport usable.
LIDAR_FIRING_HZ = 12000.0

# Spawn height above the detected room floor, so PhysX can settle the wheels.
ROBOT_SPAWN_HEIGHT = 0.05
ROBOT_SPAWN = (0.0, 0.0, ROBOT_SPAWN_HEIGHT)
IMPORT_REV = "float-base-1"
