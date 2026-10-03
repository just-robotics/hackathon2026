#!/usr/bin/env python3

"""Два робота в одном мире: защитник и атакующий.

Отличие от launch_sim.launch.py -- робот здесь не один, поэтому всё, что у
него своё (описание, robot_state_publisher, спавн, дерево TF), собирается
для каждого по списку ROBOTS. Симулятор при этом один: общий gzserver,
общий /clock и общая физика.

Роботы разведены пространствами имён: топики уходят в /defender/... и
/attacker/..., фреймы получают префикс defender/ и attacker/. Соперника
каждый видит через общий корень TF world, к которому подшиты оба дерева
одометрии.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# Стартовые позиции -- центры площадок totami: защитник на синей у восточной
# стены, атакующий на красной у западной. Роботы развёрнуты навстречу друг
# другу, поэтому защитник смотрит в -X (yaw = pi), атакующий в +X.
# Точки спавна по мирам. totami -- открытый квадрат, роботы стоят на своих
# площадках навстречу друг другу. maze_duel -- лабиринт, там защитник стоит
# не на цели, а в середине: на синей зоне он перекрывал бы её сразу, и у
# атакующего не оставалось бы ни одного выигрышного хода.
SPAWNS = {
    "totami": (
        {"name": "defender", "x": 3.5, "y": 0.0, "yaw": 3.14159},
        {"name": "attacker", "x": -3.5, "y": 0.0, "yaw": 0.0},
    ),
    # Лабиринт: три сценария отличаются только позицией стража, атакующий
    # всегда стартует в одной точке. Чем дальше страж, тем больше у
    # атакующего свободы на первых метрах.
    "maze_duel": (
        {"name": "defender", "x": 2.5, "y": 2.5, "yaw": 3.14159},
        {"name": "attacker", "x": -0.34, "y": 0.4, "yaw": 1.5708},
    ),
    "maze_duel_m2": (
        {"name": "defender", "x": 1.5, "y": 1.5, "yaw": 3.14159},
        {"name": "attacker", "x": -0.34, "y": 0.4, "yaw": 1.5708},
    ),
    "maze_duel_m3": (
        {"name": "defender", "x": -2.5, "y": 1.5, "yaw": 0.0},
        {"name": "attacker", "x": -0.34, "y": 0.4, "yaw": 1.5708},
    ),
}

# Сценарии 2 и 3 идут по той же карте, отличаются только расстановкой.
WORLD_ALIASES = {
    "maze_duel_m2": "maze_duel",
    "maze_duel_m3": "maze_duel",
}

DEFAULT_MAP = "totami"

SPAWN_Z = 0.23

# spawn_entity.py обращается к сервису gzserver и без задержки успевает
# стартовать раньше него: сервис ещё не поднят, и процесс падает с
# "Spawn service failed".
SPAWN_DELAY = 8.0


def robot_actions(robot, package_share, lidar):
    """Собрать действия запуска для одного робота

    :robot элемент ROBOTS
    :package_share каталог share пакета sim_kobuki
    :lidar подстановка с флагом лидара

    :return список действий launch
    """
    name = robot["name"]

    description = ParameterValue(
        Command(
            [
                "xacro ",
                os.path.join(package_share, "description", "kobuki.urdf.xacro"),
                " lidar:=",
                lidar,
                " ns:=",
                name,
            ]
        ),
        value_type=str,
    )

    return [
        # frame_prefix заставляет robot_state_publisher публиковать TF с
        # префиксом робота: defender/base_link вместо base_link. Плагины
        # Gazebo получают тот же префикс через ns в xacro.
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=name,
            parameters=[
                {
                    "robot_description": description,
                    "frame_prefix": f"{name}/",
                    "use_sim_time": True,
                }
            ],
            output="screen",
        ),
        # Общий корень дерева TF, через который роботы видят друг друга.
        #
        # Трансформ нулевой, и это не упущение: diff_drive в Gazebo ведёт
        # odom не от точки спавна, а от начала координат мира -- сразу после
        # старта odom -> base_footprint равен позе робота в мире. Смещать
        # odom на точку спавна значило бы учесть её дважды, и соперник
        # оказывался бы вдвое дальше, чем он есть.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f"world_to_{name}_odom",
            arguments=[
                "--x", "0.0",
                "--y", "0.0",
                "--z", "0.0",
                "--yaw", "0.0",
                "--pitch", "0.0",
                "--roll", "0.0",
                "--frame-id", "world",
                "--child-frame-id", f"{name}/odom",
            ],
            parameters=[{"use_sim_time": True}],
        ),
        TimerAction(
            period=SPAWN_DELAY,
            actions=[
                Node(
                    package="gazebo_ros",
                    executable="spawn_entity.py",
                    namespace=name,
                    arguments=[
                        "-topic", f"/{name}/robot_description",
                        "-entity", name,
                        "-robot_namespace", name,
                        "-x", str(robot["x"]),
                        "-y", str(robot["y"]),
                        "-z", str(SPAWN_Z),
                        "-Y", str(robot["yaw"]),
                    ],
                    output="screen",
                ),
            ],
        ),
    ]


def generate_launch_description():
    package_share = get_package_share_directory("sim_kobuki")
    gazebo_ros_share = get_package_share_directory("gazebo_ros")

    # Сценарии 2 и 3 идут по той же карте, что и первый: подменяем имя мира,
    # оставляя выбор расстановки за SPAWNS.
    world = PathJoinSubstitution(
        [
            package_share,
            "worlds",
            [
                PythonExpression(
                    [
                        "'",
                        LaunchConfiguration("map"),
                        "'.replace('_m2', '').replace('_m3', '')",
                    ]
                ),
                ".world",
            ],
        ]
    )
    headless = LaunchConfiguration("headless")
    lidar = LaunchConfiguration("lidar")

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, "launch", "gzserver.launch.py")
        ),
        launch_arguments={
            "world": world,
            "verbose": "true",
            # плагины ROS живут в gzserver: init отдаёт /clock, factory
            # обслуживает сервис спавна
            "init": "true",
            "factory": "true",
            # /clock по умолчанию идёт на 10 Гц, а это потолок частоты для
            # всех нод на симулированном времени: таймер ROS не срабатывает
            # чаще, чем приходит время
            "params_file": os.path.join(
                package_share, "config", "gazebo_clock.yaml"
            ),
        }.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, "launch", "gzclient.launch.py")
        ),
        condition=UnlessCondition(headless),
    )

    actions = [
        DeclareLaunchArgument(
            "map",
            default_value="totami",
            description="World from sim_kobuki/worlds, without .world.",
        ),
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            choices=["true", "false"],
            description="Run only the Gazebo server when true.",
        ),
        DeclareLaunchArgument(
            "lidar",
            default_value="false",
            choices=["true", "false"],
            description="Attach the Livox lidar to every robot.",
        ),
        gzserver,
        gzclient,
    ]

    # Точки спавна зависят от карты, а её значение известно только в момент
    # запуска: LaunchConfiguration на этапе сборки описания не читается.
    # OpaqueFunction выполняется уже с подставленными аргументами.
    def spawn_robots(context):
        name = LaunchConfiguration("map").perform(context)
        robots = SPAWNS.get(name, SPAWNS[DEFAULT_MAP])

        spawned = []
        for robot in robots:
            spawned += robot_actions(robot, package_share, lidar)

        return spawned

    actions.append(OpaqueFunction(function=spawn_robots))

    return LaunchDescription(actions)
