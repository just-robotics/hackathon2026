#!/usr/bin/env python3

"""Детектор соперника: находит второго Kobuki в облаке своего лидара.

На каждый скан:
1. облако переводится в систему мира по собственной позе на момент скана
   (localization/pose, ground truth p3d) и статическому TF до лидара;
2. вычитается пустой полигон: пол, стены из .world и всё за ними;
3. остаток разбивается на кластеры, и среди них ищутся похожие на Kobuki
   (segmentation.detect_robot);
4. трекер связывает детекции во времени и по движению восстанавливает курс.

Соперник публикуется в opponent/odom как nav_msgs/Odometry во фрейме world.
Это тот же тип, что у localization/pose, поэтому оценку можно напрямую
сравнить с ground truth соперника (detector_eval.py).
"""

import array
import dataclasses
import math
import time
from collections import deque

import numpy as np
import rclpy

from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from jr_map.sdf_map_server import collect_boxes
from jr_perception.segmentation import (
    RobotModel,
    arena_bounds,
    cluster_xy,
    detect_robot,
    foreground_mask,
    split_clusters,
)
from jr_perception.tracker import OMEGA, THETA, V, X, Y, Tracker, TrackerConfig


def declare_dataclass(node: Node, cls, prefix: str):
    """Объявить поля dataclass параметрами ноды и собрать из них экземпляр

    :node нода
    :cls класс-dataclass со значениями по умолчанию
    :prefix префикс имён параметров: robot.radius, tracker.gate

    :return экземпляр cls
    """
    values = {
        field.name: node.declare_parameter(
            f"{prefix}.{field.name}", field.default
        ).value
        for field in dataclasses.fields(cls)
    }
    return cls(**values)


def stamp_seconds(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def quaternion_matrix(quaternion) -> np.ndarray:
    """Матрица поворота по кватерниону (x, y, z, w)"""
    x, y, z, w = quaternion
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def slerp(q0: np.ndarray, q1: np.ndarray, fraction: float) -> np.ndarray:
    """Сферическая интерполяция кватернионов (x, y, z, w)"""
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1, dot = -q1, -dot

    if dot > 0.9995:
        # почти совпадают: линейная интерполяция без деления на sin(0)
        quaternion = q0 + fraction * (q1 - q0)
        return quaternion / np.linalg.norm(quaternion)

    angle = math.acos(dot)
    return (
        math.sin((1.0 - fraction) * angle) * q0 + math.sin(fraction * angle) * q1
    ) / math.sin(angle)


def interpolate_pose(poses: deque, moment: float):
    """Поза на момент moment по буферу поз

    :poses буфер (время, позиция, кватернион) по возрастанию времени
    :moment момент, с

    :return (позиция, кватернион) или None, если момент вне буфера
    """
    if not poses or moment < poses[0][0] or moment > poses[-1][0]:
        return None

    # скан почти всегда моложе большей части буфера, поэтому поиск с конца
    for i in range(len(poses) - 1, 0, -1):
        t0, p0, q0 = poses[i - 1]
        t1, p1, q1 = poses[i]
        if t0 <= moment <= t1:
            fraction = 0.0 if t1 == t0 else (moment - t0) / (t1 - t0)
            return p0 + fraction * (p1 - p0), slerp(q0, q1, fraction)

    _, position, quaternion = poses[0]
    return position, quaternion


def cloud_to_xyz(message: PointCloud2) -> np.ndarray:
    """Координаты точек облака как массив (N, 3)

    Поля x, y, z ожидаются float32, как их пишет gazebo_ros_ray_sensor.
    """
    if message.width * message.height == 0:
        return np.zeros((0, 3))

    offsets = {field.name: field.offset for field in message.fields}
    order = ">" if message.is_bigendian else "<"
    dtype = np.dtype(
        {
            "names": ["x", "y", "z"],
            "formats": [order + "f4"] * 3,
            "offsets": [offsets["x"], offsets["y"], offsets["z"]],
            "itemsize": message.point_step,
        }
    )
    cloud = np.frombuffer(
        message.data, dtype=dtype, count=message.width * message.height
    )
    points = np.stack([cloud["x"], cloud["y"], cloud["z"]], axis=1).astype(
        np.float64
    )
    return points[np.isfinite(points).all(axis=1)]


def xyz_to_cloud(header: Header, points: np.ndarray) -> PointCloud2:
    """Собрать PointCloud2 из массива (N, 3)"""
    message = PointCloud2()
    message.header = header
    message.height = 1
    message.width = len(points)
    message.fields = [
        PointField(name=name, offset=4 * i, datatype=PointField.FLOAT32, count=1)
        for i, name in enumerate("xyz")
    ]
    message.is_bigendian = False
    message.point_step = 12
    message.row_step = 12 * len(points)
    message.is_dense = True
    # array('B') ложится в поле как есть, а bytes rclpy проверял бы
    # поэлементно в Python
    message.data = array.array("B", points.astype(np.float32).tobytes())
    return message


def track_to_odometry(header: Header, child_frame: str, track) -> Odometry:
    """Трек соперника как nav_msgs/Odometry

    Пока курс не известен, его дисперсия pi^2, а скорости -- 1 (м/с)^2 и
    1 (рад/с)^2: оценки нет.
    """
    x, y, theta, v, omega = track.state

    message = Odometry()
    message.header = header
    message.child_frame_id = child_frame
    message.pose.pose.position.x = float(x)
    message.pose.pose.position.y = float(y)
    message.pose.pose.orientation.z = math.sin(theta / 2.0)
    message.pose.pose.orientation.w = math.cos(theta / 2.0)

    # порядок x, y, z, roll, pitch, yaw; высота и наклоны известны -- робот
    # стоит на полу
    pose_covariance = np.diag([0.0, 0.0, 1e-6, 1e-6, 1e-6, 0.0])
    for a, i in zip((0, 1, 5), (X, Y, THETA)):
        for b, j in zip((0, 1, 5), (X, Y, THETA)):
            pose_covariance[a, b] = track.cov[i, j]
    message.pose.covariance = pose_covariance.reshape(-1).tolist()

    message.twist.twist.linear.x = float(v)
    message.twist.twist.angular.z = float(omega)
    twist_covariance = np.diag([1.0, 1e-6, 1e-6, 1e-6, 1e-6, 1.0])
    if track.heading_known:
        for a, i in zip((0, 5), (V, OMEGA)):
            for b, j in zip((0, 5), (V, OMEGA)):
                twist_covariance[a, b] = track.cov[i, j]
    message.twist.covariance = twist_covariance.reshape(-1).tolist()

    return message


def make_markers(
    header: Header,
    clusters: list,
    detections: list,
    tracks: list,
    selected,
    radius: float,
) -> MarkerArray:
    """Отладочные маркеры: кластеры по классам, треки и курс соперника

    Кластеры: зелёный -- центр найден фитом обода, жёлтый -- похож на робота,
    но обода не видно, серый -- не робот. Треки: синий -- выбранный соперник,
    фиолетовый -- другие подтверждённые, серый -- неподтверждённые.
    """
    markers = MarkerArray()

    clear = Marker()
    clear.header = header
    clear.action = Marker.DELETEALL
    markers.markers.append(clear)

    for i, (points, detection) in enumerate(zip(clusters, detections)):
        low, high = points.min(axis=0), points.max(axis=0)
        middle = (low + high) / 2.0
        size = np.maximum(high - low, 0.02)

        marker = Marker()
        marker.header = header
        marker.ns = "clusters"
        marker.id = i
        marker.type = Marker.CUBE
        marker.pose.position.x = float(middle[0])
        marker.pose.position.y = float(middle[1])
        marker.pose.position.z = float(middle[2])
        marker.pose.orientation.w = 1.0
        marker.scale.x, marker.scale.y, marker.scale.z = (float(s) for s in size)
        if detection is None:
            color = (0.6, 0.6, 0.6)
        elif detection.strong:
            color = (0.1, 0.8, 0.1)
        else:
            color = (0.9, 0.8, 0.1)
        marker.color.r, marker.color.g, marker.color.b = color
        marker.color.a = 0.4
        markers.markers.append(marker)

    for i, track in enumerate(tracks):
        x, y, theta = track.state[X], track.state[Y], track.state[THETA]

        marker = Marker()
        marker.header = header
        marker.ns = "tracks"
        marker.id = i
        marker.type = Marker.CYLINDER
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = 0.2
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = 2.0 * radius
        marker.scale.z = 0.4
        if track is selected:
            color, alpha = (0.1, 0.4, 1.0), 0.6
        elif track.confirmed:
            color, alpha = (0.8, 0.2, 0.8), 0.4
        else:
            color, alpha = (0.6, 0.6, 0.6), 0.3
        marker.color.r, marker.color.g, marker.color.b = color
        marker.color.a = alpha
        markers.markers.append(marker)

        if track is selected and track.heading_known:
            arrow = Marker()
            arrow.header = header
            arrow.ns = "heading"
            arrow.id = 0
            arrow.type = Marker.ARROW
            arrow.points = [
                Point(x=float(x), y=float(y), z=0.45),
                Point(
                    x=float(x + 0.5 * math.cos(theta)),
                    y=float(y + 0.5 * math.sin(theta)),
                    z=0.45,
                ),
            ]
            arrow.scale.x = 0.03
            arrow.scale.y = 0.06
            arrow.scale.z = 0.1
            arrow.color.r, arrow.color.g, arrow.color.b = (0.1, 0.4, 1.0)
            arrow.color.a = 1.0
            markers.markers.append(arrow)

    return markers


class RobotDetector(Node):
    """Нода детектора соперника"""

    def __init__(self):
        super().__init__("robot_detector")

        world = self.declare_parameter("world", "").value
        background_models = self.declare_parameter(
            "background_models", ["totami_built"]
        ).value
        z_slice = self.declare_parameter("z_slice", 0.25).value
        cloud_topic = self.declare_parameter("cloud_topic", "livox/lidar").value
        pose_topic = self.declare_parameter("pose_topic", "localization/pose").value
        self.world_frame = self.declare_parameter("world_frame", "world").value
        self.opponent_frame = self.declare_parameter(
            "opponent_frame", "opponent"
        ).value
        self.self_range = self.declare_parameter("self_range", 0.30).value
        self.wall_margin = self.declare_parameter("wall_margin", 0.08).value
        self.floor_z = self.declare_parameter("floor_z", 0.012).value
        self.floor_noise = self.declare_parameter("floor_noise", 0.06).value
        self.ceiling_z = self.declare_parameter("ceiling_z", 0.70).value
        self.cluster_tolerance = self.declare_parameter(
            "cluster_tolerance", 0.10
        ).value
        self.log_period = self.declare_parameter("log_period", 5.0).value
        self.model = declare_dataclass(self, RobotModel, "robot")
        self.tracker = Tracker(declare_dataclass(self, TrackerConfig, "tracker"))

        # Фреймы робота несут префикс пространства имён: defender/base_footprint
        namespace = self.get_namespace().strip("/")
        prefix = f"{namespace}/" if namespace else ""
        self.base_frame = self.declare_parameter(
            "base_frame", f"{prefix}base_footprint"
        ).value

        if not world:
            self.get_logger().error("Параметр world не задан")
            raise SystemExit(1)

        self.boxes = collect_boxes(world, z_slice, models=set(background_models))
        if not self.boxes:
            self.get_logger().error(
                f"В {world} у моделей {list(background_models)} нет "
                f"box-коллизий на высоте {z_slice} м: фон не построен"
            )
            raise SystemExit(1)
        self.bounds = arena_bounds(self.boxes)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # поворот и смещение лидара в base_frame: трансформ статический,
        # поэтому берётся один раз
        self.sensor_offset = None

        # 50 Гц p3d -- это 2 с истории, с запасом на задержку облака
        self.poses = deque(maxlen=100)
        # скан, для которого ещё не пришла поза на его момент
        self.pending = None

        self.create_subscription(
            PointCloud2, cloud_topic, self.on_cloud, qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry, pose_topic, self.on_pose, qos_profile_sensor_data
        )
        self.odometry_publisher = self.create_publisher(
            Odometry, "opponent/odom", 10
        )
        self.foreground_publisher = self.create_publisher(
            PointCloud2, "opponent/foreground", qos_profile_sensor_data
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray, "opponent/markers", 10
        )

        self.stats = self.empty_stats()
        self.last_log = time.monotonic()

        min_x, min_y, max_x, max_y = self.bounds
        self.get_logger().info(
            f"Фон: {len(self.boxes)} боксов моделей {list(background_models)} "
            f"из {world}, арена x [{min_x:.2f}, {max_x:.2f}], "
            f"y [{min_y:.2f}, {max_y:.2f}]. Облако {cloud_topic}, "
            f"поза {pose_topic}, база {self.base_frame}"
        )

    @staticmethod
    def empty_stats() -> dict:
        return dict.fromkeys(
            ("frames", "seconds", "points", "clusters", "candidates", "found"), 0
        )

    def on_pose(self, message: Odometry):
        moment = stamp_seconds(message.header.stamp)
        if self.poses and moment < self.poses[-1][0]:
            # время пошло назад: симулятор перезапущен
            self.poses.clear()

        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        self.poses.append(
            (
                moment,
                np.array([position.x, position.y, position.z]),
                np.array([orientation.x, orientation.y, orientation.z, orientation.w]),
            )
        )
        self.flush()

    def on_cloud(self, message: PointCloud2):
        if self.pending is not None:
            self.get_logger().warn(
                "Скан пропущен: поза на его момент так и не пришла. "
                "Проверьте, что публикуется localization/pose",
                throttle_duration_sec=5.0,
            )
        self.pending = message
        self.flush()

    def flush(self):
        """Обработать отложенный скан, как только есть поза на его момент

        Поза и облако публикуются на одном шаге симуляции, но в любом
        порядке. Скан ждёт позу со штампом не раньше своего, чтобы не
        экстраполировать: 20 мс поворота на 1 рад/с -- это уже 5 см на стене
        в 4 м.
        """
        if self.pending is None or not self.poses:
            return

        moment = stamp_seconds(self.pending.header.stamp)
        if moment > self.poses[-1][0]:
            return

        message, self.pending = self.pending, None
        pose = interpolate_pose(self.poses, moment)
        if pose is None:
            self.get_logger().warn(
                "Скан старше буфера поз, пропущен", throttle_duration_sec=5.0
            )
            return

        self.process(message, moment, pose)

    def sensor_in_world(self, frame: str, pose: tuple):
        """Поворот и положение лидара в мире на момент скана

        :frame фрейм облака
        :pose (позиция, кватернион) base_frame в мире

        :return (rotation, position) или (None, None), если нет TF
        """
        if self.sensor_offset is None:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame, frame, Time()
                )
            except TransformException as error:
                self.get_logger().warn(
                    f"Нет TF {self.base_frame} -> {frame}: {error}",
                    throttle_duration_sec=5.0,
                )
                return None, None

            translation = transform.transform.translation
            rotation = transform.transform.rotation
            self.sensor_offset = (
                quaternion_matrix([rotation.x, rotation.y, rotation.z, rotation.w]),
                np.array([translation.x, translation.y, translation.z]),
            )

        position, quaternion = pose
        base_rotation = quaternion_matrix(quaternion)
        offset_rotation, offset_translation = self.sensor_offset
        return (
            base_rotation @ offset_rotation,
            position + base_rotation @ offset_translation,
        )

    def process(self, message: PointCloud2, moment: float, pose: tuple):
        """Найти соперника в скане и опубликовать результат"""
        started = time.perf_counter()

        rotation, position = self.sensor_in_world(message.header.frame_id, pose)
        if rotation is None:
            return

        points = cloud_to_xyz(message)
        # ближние точки -- свои пластины: лучи круче -45 градусов упираются в
        # среднюю пластину на дальности до 0.25 м
        points = points[np.linalg.norm(points, axis=1) >= self.self_range]
        world = points @ rotation.T + position

        foreground = world[
            foreground_mask(
                world,
                position,
                self.boxes,
                self.bounds,
                self.wall_margin,
                self.floor_z,
                self.floor_noise,
                self.ceiling_z,
            )
        ]
        labels, count = cluster_xy(foreground[:, :2], self.cluster_tolerance)
        clusters = split_clusters(foreground, labels, count)
        detections = [
            detect_robot(cluster, position[:2], self.model) for cluster in clusters
        ]
        opponent = self.tracker.step(
            moment, [detection for detection in detections if detection is not None]
        )

        header = Header(stamp=message.header.stamp, frame_id=self.world_frame)
        if opponent is not None:
            self.odometry_publisher.publish(
                track_to_odometry(header, self.opponent_frame, opponent)
            )
        self.foreground_publisher.publish(xyz_to_cloud(header, foreground))
        self.marker_publisher.publish(
            make_markers(
                header,
                clusters,
                detections,
                self.tracker.tracks,
                opponent,
                self.model.radius,
            )
        )

        stats = self.stats
        stats["frames"] += 1
        stats["seconds"] += time.perf_counter() - started
        stats["points"] += len(foreground)
        stats["clusters"] += len(clusters)
        stats["candidates"] += sum(detection is not None for detection in detections)
        stats["found"] += opponent is not None
        self.report(opponent)

    def report(self, opponent):
        """Раз в log_period секунд -- строка о нагрузке и результате"""
        now = time.monotonic()
        if now - self.last_log < self.log_period:
            return

        stats = self.stats
        frames = max(stats["frames"], 1)
        text = (
            f"{stats['frames']} сканов за {now - self.last_log:.0f} с: "
            f"обработка {1000.0 * stats['seconds'] / frames:.1f} мс, "
            f"переднего плана {stats['points'] / frames:.0f} точек, "
            f"кластеров {stats['clusters'] / frames:.1f}, "
            f"похожих на робота {stats['candidates'] / frames:.1f}, "
            f"соперник найден в {100.0 * stats['found'] / frames:.0f}% сканов"
        )
        if opponent is not None:
            heading = (
                f"курс {math.degrees(opponent.state[THETA]):.0f} град"
                if opponent.heading_known
                else "курс пока неизвестен"
            )
            text += (
                f"; сейчас ({opponent.state[X]:.2f}, {opponent.state[Y]:.2f}), "
                f"{heading}"
            )

        self.get_logger().info(text)
        self.stats = self.empty_stats()
        self.last_log = now


def main():
    rclpy.init()
    node = RobotDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
