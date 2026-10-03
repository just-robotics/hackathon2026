#!/usr/bin/env python3

"""Записать телеметрию обоих роботов и их пути в JSON.

Пишется всё, что нужно для разбора поведения: команда и факт по скорости,
боковое отклонение и ошибка курса от контроллера, позиции роботов и путь,
который в этот момент выдал планировщик.

Запуск внутри контейнера duel:
    python3 record_telemetry.py [секунды] [файл]
"""

import json
import sys
import time

import rclpy

from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Float32, Float64


SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 45.0
OUT = sys.argv[2] if len(sys.argv) > 2 else "/tmp/telemetry.json"
ROBOTS = ("attacker", "defender")
SCALARS = ("e_lat", "e_theta", "v_cmd", "v_meas", "w_cmd")


class Recorder(Node):
    """Собирает кадры состояния с фиксированным шагом"""

    def __init__(self):
        super().__init__("telemetry_recorder")

        self.state = {robot: {} for robot in ROBOTS}
        self.frames = []
        self.start = None

        for robot in ROBOTS:
            for name in SCALARS:
                # тип зависит от ноды: часть диагностики идёт Float32
                for kind in (Float64, Float32):
                    self.create_subscription(
                        kind,
                        f"/{robot}/control/{name}",
                        self.make_scalar(robot, name),
                        10,
                    )

            self.create_subscription(
                Odometry,
                f"/{robot}/localization/pose",
                self.make_pose(robot),
                10,
            )
            self.create_subscription(
                Path,
                f"/{robot}/planning/trajectory",
                self.make_path(robot),
                1,
            )

        self.create_timer(0.1, self.tick)

    def make_scalar(self, robot, name):
        """Обработчик одного скалярного топика"""
        return lambda message: self.state[robot].__setitem__(
            name, float(message.data)
        )

    def make_pose(self, robot):
        """Обработчик позы робота"""

        def callback(message):
            self.state[robot]["x"] = message.pose.pose.position.x
            self.state[robot]["y"] = message.pose.pose.position.y

        return callback

    def make_path(self, robot):
        """Обработчик пути планировщика"""

        def callback(message):
            self.state[robot]["path"] = [
                (pose.pose.position.x, pose.pose.position.y)
                for pose in message.poses
            ]

        return callback

    def tick(self):
        """Снять кадр и завершить запись по истечении времени"""
        now = time.time()

        if self.start is None:
            self.start = now

        frame = {"t": round(now - self.start, 2)}
        for robot in ROBOTS:
            frame[robot] = dict(self.state[robot])
        self.frames.append(frame)

        if now - self.start >= SECONDS:
            with open(OUT, "w") as handle:
                json.dump(self.frames, handle)
            print(f"записано {len(self.frames)} кадров в {OUT}")
            raise SystemExit


def main():
    rclpy.init()
    node = Recorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass


if __name__ == "__main__":
    main()
