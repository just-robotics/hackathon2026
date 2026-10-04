"""Детектор соперника (robot_detector.py) на бэге с едущего робота + RViz.

Локализация -- FAST-LIO2: livox_custom.py -> fastlio_mapping ->
fastlio_bridge.py. Облако детектору -- после robot_body_filter (корпус и
пол) и dbscan_filter (шум вблизи робота), во фрейме лидара. Фон -- карта
полигона из map_server.

    ros2 launch jr_launch jr_detector_fastlio_bag.launch.py bag:=<запись или запись/bag>

Бэг проигрывает сам launch, один раз, когда ноды уже запущены. Повтор не
годится: FAST-LIO2 продолжил бы с конца прошлого прохода -- для повтора
launch перезапускается. Из бэга берутся только /livox/lidar, /livox/imu и
/tf_static. /tf не проигрывается: в autonomous-записях в нём map -> odom от
AMCL робота, который спорил бы с FAST-LIO2, поэтому мост публикует сразу
map -> base_footprint.

Стартовая поза робота на карте -- x, y, yaw. Если они не заданы, её считает
initial_pose.py по первым сканам бэга и карте.
"""

import subprocess
from pathlib import Path

import launch.logging
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def bag_directory(value: str) -> Path:
    """Папка с metadata.yaml: можно указать и саму запись, и её bag/"""
    path = Path(value).expanduser().resolve()
    if (path / "bag" / "metadata.yaml").is_file():
        path = path / "bag"
    if not (path / "metadata.yaml").is_file():
        raise RuntimeError(f"Нет бэга в {value}: нужна папка записи или её bag/ с metadata.yaml")
    return path


def start_pose(context, bag: Path, map_file: str) -> tuple:
    """(x, y, yaw) из аргументов или от initial_pose.py"""
    given = [LaunchConfiguration(name).perform(context) for name in ("x", "y", "yaw")]
    if all(given):
        return tuple(float(value) for value in given)
    if any(given):
        raise RuntimeError("x, y и yaw задаются вместе")
    launch.logging.get_logger("launch").info(
        "Стартовая поза не задана: считаю её по первым сканам бэга (около 10 с)"
    )
    result = subprocess.run(
        ["ros2", "run", "jr_perception", "initial_pose.py", str(bag), map_file],
        capture_output=True,
        text=True,
        check=True,
    )
    x, y, yaw = (float(value) for value in result.stdout.strip().splitlines()[-1].split())
    return x, y, yaw


def nodes(context):
    share = Path(get_package_share_directory("jr_launch"))
    bag = bag_directory(LaunchConfiguration("bag").perform(context))
    map_file = LaunchConfiguration("map").perform(context) or str(
        share / "config/maps/maze_bag_v1.yaml"
    )
    x, y, yaw = start_pose(context, bag, map_file)
    sim = {"use_sim_time": True}

    actions = [
        LogInfo(msg=f"Бэг {bag}, старт робота на карте: x {x:.3f}, y {y:.3f}, yaw {yaw:.4f}"),
        Node(
            package="jr_perception",
            executable="livox_custom.py",
            name="livox_custom",
            output="screen",
            parameters=[sim],
        ),
        Node(
            package="fast_lio",
            executable="fastlio_mapping",
            name="laser_mapping",
            output="screen",
            parameters=[str(share / "config/perception/fastlio.param.yaml"), sim],
        ),
        Node(
            package="jr_perception",
            executable="fastlio_bridge.py",
            name="fastlio_bridge",
            output="screen",
            # без /tf бэга одометрии колёс нет: TF сразу map -> base_footprint
            parameters=[{"x": x, "y": y, "yaw": yaw, "odom_frame": "", "tf_tolerance": 0.0, **sim}],
        ),
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[{"yaml_filename": map_file, **sim}],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="map_lifecycle_manager",
            output="screen",
            parameters=[{"autostart": True, "node_names": ["map_server"], **sim}],
        ),
        # Облако остаётся во фрейме лидара: детектор считает self_range и лучи
        # от начала координат облака, в base_footprint датчик оказался бы на полу
        Node(
            package="robot_body_filter",
            executable="robot_body_filter_node",
            name="crop_box_filter_node",
            output="screen",
            parameters=[
                str(share / "config/sensing/crop.param.yaml"),
                {"keep_input_frame": True, **sim},
            ],
            remappings=[
                ("~/input/pointcloud", "/livox/lidar"),
                ("~/output/pointcloud", "/sensing/lidar/cropped/pointcloud"),
            ],
        ),
        Node(
            package="dbscan_filter",
            executable="dbscan_filter_node",
            name="dbscan_filter_node",
            output="screen",
            parameters=[
                str(Path(get_package_share_directory("dbscan_filter")) / "config/dbscan.param.yaml"),
                sim,
            ],
            remappings=[
                ("~/input/pointcloud", "/sensing/lidar/cropped/pointcloud"),
                ("~/output/pointcloud", "/sensing/lidar/dbscan/pointcloud"),
            ],
        ),
        Node(
            package="jr_perception",
            executable="robot_detector.py",
            name="robot_detector",
            output="screen",
            parameters=[str(share / "config/perception/robot_detector_fastlio.param.yaml"), sim],
        ),
    ]
    if LaunchConfiguration("rviz").perform(context).lower() == "true":
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", str(share / "config/perception/detector_fastlio.rviz")],
                parameters=[sim],
            )
        )

    play = ExecuteProcess(
        cmd=[
            "ros2", "bag", "play", str(bag), "--clock",
            "--rate", LaunchConfiguration("rate").perform(context),
            "--topics", "/livox/lidar", "/livox/imu", "/tf_static",
        ],
        output="screen",
    )
    actions += [
        TimerAction(period=float(LaunchConfiguration("delay").perform(context)), actions=[play]),
        RegisterEventHandler(
            OnProcessExit(
                target_action=play,
                on_exit=[LogInfo(msg="Бэг закончился. Повтор -- перезапуск launch (FAST-LIO2 начнёт с нуля)")],
            )
        ),
    ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("bag", description="папка записи или её bag/ с metadata.yaml"),
        DeclareLaunchArgument("map", default_value="", description="карта Nav2 .yaml; по умолчанию -- полигон"),
        DeclareLaunchArgument("x", default_value="", description="старт base_footprint на карте, м; пусто -- initial_pose.py"),
        DeclareLaunchArgument("y", default_value=""),
        DeclareLaunchArgument("yaw", default_value="", description="рад"),
        DeclareLaunchArgument("rate", default_value="1.0", description="скорость проигрывания бэга"),
        DeclareLaunchArgument("delay", default_value="3.0", description="пауза перед бэгом, с: ноды успевают подняться"),
        DeclareLaunchArgument("rviz", default_value="true"),
        OpaqueFunction(function=nodes),
    ])
