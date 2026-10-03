#!/usr/bin/env python3

"""Сверка детектора соперника с ground truth симулятора.

Сравнивает оценку /<observer>/opponent/odom с точной позой соперника
/<opponent>/localization/pose и раз в period секунд печатает таблицу по
корзинам дистанции между роботами:

- найден -- доля сканов, на которых детектор выдал соперника верно;
- ложных -- оценки дальше false_distance от соперника: трек сел не на того;
- ошибка позиции -- по верным оценкам;
- ошибка курса -- когда соперник едет быстрее min_speed и детектор уже
  знает курс: у стоящего робота курс по лидару не наблюдаем.

Часы сканов -- opponent/foreground: детектор публикует его на каждый
обработанный скан, даже пустым.
"""

import bisect
import math
from collections import deque

import numpy as np
import rclpy

from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


# границы корзин дистанции, м
BINS = (2.0, 4.0, 6.0, 8.0)

# курс считается известным, пока его СКО меньше 90 градусов: неизвестный
# детектор публикует с дисперсией pi^2
HEADING_KNOWN_VARIANCE = (math.pi / 2.0) ** 2


def stamp_seconds(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def yaw_of(orientation) -> float:
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2),
    )


def wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def bin_name(index: int) -> str:
    edges = (0.0,) + BINS
    if index < len(BINS):
        return f"{edges[index]:.0f}-{edges[index + 1]:.0f} м"
    return f"{BINS[-1]:.0f}+ м"


class PoseBuffer:
    """Позы робота по времени с поиском ближайшей"""

    def __init__(self, seconds: float = 30.0):
        self.seconds = seconds
        self.times = []
        self.poses = []

    def add(self, moment: float, pose: tuple):
        if self.times and moment < self.times[-1]:
            # время пошло назад: симулятор перезапущен
            self.times.clear()
            self.poses.clear()

        self.times.append(moment)
        self.poses.append(pose)

        if moment - self.times[0] > 2.0 * self.seconds:
            cut = bisect.bisect_left(self.times, moment - self.seconds)
            del self.times[:cut]
            del self.poses[:cut]

    def nearest(self, moment: float, tolerance: float = 0.05):
        """Ближайшая к моменту поза или None, если ближе tolerance нет"""
        i = bisect.bisect_left(self.times, moment)
        best = None
        for j in (i - 1, i):
            if 0 <= j < len(self.times):
                if best is None or abs(self.times[j] - moment) < abs(
                    self.times[best] - moment
                ):
                    best = j

        if best is None or abs(self.times[best] - moment) > tolerance:
            return None
        return self.poses[best]


class DetectorEval(Node):
    """Нода сверки детектора с ground truth"""

    def __init__(self):
        super().__init__("detector_eval")

        observer = self.declare_parameter("observer", "defender").value
        opponent = self.declare_parameter("opponent", "attacker").value
        period = self.declare_parameter("period", 10.0).value
        self.min_speed = self.declare_parameter("min_speed", 0.1).value
        self.false_distance = self.declare_parameter("false_distance", 0.5).value
        self.title = f"{observer} ищет {opponent}"

        self.observer_poses = PoseBuffer()
        self.opponent_poses = PoseBuffer()
        # моменты обработанных детектором сканов и оценки по ним
        self.frames = deque()
        self.estimates = {}
        self.rows = [self.empty_row() for _ in range(len(BINS) + 1)]

        self.create_subscription(
            PointCloud2,
            f"/{observer}/opponent/foreground",
            self.on_frame,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry, f"/{observer}/opponent/odom", self.on_estimate, 10
        )
        self.create_subscription(
            Odometry,
            f"/{observer}/localization/pose",
            lambda message: self.on_pose(self.observer_poses, message),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            f"/{opponent}/localization/pose",
            lambda message: self.on_pose(self.opponent_poses, message),
            qos_profile_sensor_data,
        )
        self.create_timer(period, self.report)

        self.get_logger().info(
            f"Сверка: {self.title}, отчёт каждые {period:.0f} с"
        )

    @staticmethod
    def empty_row() -> dict:
        return {"frames": 0, "found": 0, "false": 0, "position": [], "heading": []}

    def on_pose(self, buffer: PoseBuffer, message: Odometry):
        pose = message.pose.pose
        velocity = message.twist.twist.linear
        buffer.add(
            stamp_seconds(message.header.stamp),
            (
                pose.position.x,
                pose.position.y,
                yaw_of(pose.orientation),
                math.hypot(velocity.x, velocity.y),
            ),
        )

    def on_frame(self, message: PointCloud2):
        stamp = message.header.stamp
        self.frames.append((stamp_seconds(stamp), (stamp.sec, stamp.nanosec)))

    def on_estimate(self, message: Odometry):
        stamp = message.header.stamp
        pose = message.pose.pose
        self.estimates[(stamp.sec, stamp.nanosec)] = (
            pose.position.x,
            pose.position.y,
            yaw_of(pose.orientation),
            message.pose.covariance[35] < HEADING_KNOWN_VARIANCE,
        )

    def evaluate(self, horizon: float):
        """Разобрать сканы не новее horizon

        Оценка приходит через миллисекунды после скана, поэтому скан
        разбирается с запасом, когда оценка по нему уже точно дошла бы.
        """
        while self.frames and self.frames[0][0] <= horizon:
            moment, key = self.frames.popleft()
            estimate = self.estimates.pop(key, None)
            observer = self.observer_poses.nearest(moment)
            opponent = self.opponent_poses.nearest(moment)
            if observer is None or opponent is None:
                continue

            distance = math.hypot(opponent[0] - observer[0], opponent[1] - observer[1])
            row = self.rows[bisect.bisect_right(BINS, distance)]
            row["frames"] += 1
            if estimate is None:
                continue

            error = math.hypot(estimate[0] - opponent[0], estimate[1] - opponent[1])
            if error > self.false_distance:
                row["false"] += 1
                continue

            row["found"] += 1
            row["position"].append(error)
            if opponent[3] >= self.min_speed and estimate[3]:
                row["heading"].append(abs(wrap(estimate[2] - opponent[2])))

        # оценки без скана остаются, только если скан потерялся
        if len(self.estimates) > 1000:
            self.estimates.clear()

    def report(self, final: bool = False):
        if self.opponent_poses.times:
            horizon = (
                math.inf if final else self.opponent_poses.times[-1] - 0.5
            )
            self.evaluate(horizon)

        lines = [
            f"{self.title}",
            f"{'дистанция':<10} {'сканов':>7} {'найден':>7} {'ложных':>7}"
            f"  {'позиция, см: сред / p95':>24}  {'курс, град: сред / p95':>23}",
        ]
        total = self.empty_row()
        for index, row in enumerate(self.rows):
            for key in total:
                total[key] += row[key]
            lines.append(self.format_row(bin_name(index), row))
        lines.append(self.format_row("всего", total))

        self.get_logger().info("\n".join(lines))

    @staticmethod
    def format_row(name: str, row: dict) -> str:
        def spread(values: list, scale: float) -> str:
            if not values:
                return "-"
            values = np.asarray(values) * scale
            return f"{values.mean():.1f} / {np.percentile(values, 95):.1f}"

        found = (
            f"{100.0 * row['found'] / row['frames']:.0f}%" if row["frames"] else "-"
        )
        return (
            f"{name:<10} {row['frames']:>7} {found:>7} {row['false']:>7}"
            f"  {spread(row['position'], 100.0):>24}"
            f"  {spread(row['heading'], 180.0 / math.pi):>23}"
        )


def main():
    rclpy.init()
    node = DetectorEval()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.report(final=True)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
