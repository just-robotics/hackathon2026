#!/usr/bin/env python3

"""Верхний уровень планирования: подсказка игровому MPC, куда ехать.

Игровой MPC видит только горизонт в пару метров, поэтому в лабиринте он
встаёт перед первой же стеной: все манёвры одинаково упираются, и выбрать
обход не из чего. Эта нода решает глобальную задачу -- ищет путь к зоне по
карте занятости -- и отдаёт MPC ближайшую точку этого пути как цель.

Разделение ответственности: здесь только проходимость, без соперника. Игра,
уклонение и выбор манёвра остаются за MPC, который эту точку получает как
goal и сам решает, как к ней ехать.

Путь ищется волновым алгоритмом по сетке с раздутыми на радиус робота
стенами, поэтому он заведомо проходим. Пересчитывается на каждом тике: это
дешевле, чем отслеживать, не сошёл ли робот с прежнего пути.
"""

import math

import rclpy
import tf2_ros

import heapq

import numpy as np

from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration as RclDuration
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)


OCCUPIED = 100


class AttackStrategy(Node):
    """Ищет путь к зоне по карте и ведёт по нему игровой MPC"""

    def __init__(self):
        super().__init__("attack_strategy")

        self.declare_parameter("base_frame", "attacker/base_footprint")
        self.declare_parameter("output_frame", "world")
        self.declare_parameter("goal", [2.7, 2.7])

        # Запас до стен при поиске пути. Радиус Kobuki 0.175 м, остальное --
        # на неточность следования: путь, проложенный впритирку, робот не
        # пройдёт, так как MPC режет углы.
        self.declare_parameter("clearance", 0.24)

        # Насколько далеко по найденному пути ставить цель для MPC. Ближе --
        # робот ведёт себя осторожнее в поворотах, дальше -- срезает углы.
        self.declare_parameter("lookahead", 1.0)

        self.declare_parameter("rate", 5.0)

        self.base_frame = self.get_parameter("base_frame").value
        self.output_frame = self.get_parameter("output_frame").value
        self.goal = np.array(self.get_parameter("goal").value, dtype=float)
        self.clearance = self.get_parameter("clearance").value
        self.lookahead = self.get_parameter("lookahead").value

        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)

        self.passable = None
        self.resolution = None
        self.origin = None

        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, "/map", self.on_map, map_qos)

        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(
            PointStamped,
            "planning/strategy_goal",
            qos,
        )
        self.path_publisher = self.create_publisher(
            Path,
            "planning/global_path",
            qos,
        )

        self.create_timer(1.0 / self.get_parameter("rate").value, self.decide)

        self.get_logger().info(
            f"Стратегия: путь к {self.goal.tolist()} по карте, "
            f"запас до стен {self.clearance} м"
        )

    def on_map(self, message: OccupancyGrid) -> None:
        """Построить маску проходимости из карты занятости

        Стены раздуваются на радиус робота с запасом, поэтому любой путь по
        этой маске робот физически проходит.

        :message карта занятости
        """
        grid = np.array(message.data, dtype=np.int8).reshape(
            message.info.height, message.info.width
        )

        self.resolution = message.info.resolution
        self.origin = (
            message.info.origin.position.x,
            message.info.origin.position.y,
        )

        occupied = grid == OCCUPIED
        blocked = occupied.copy()
        cells = int(round(self.clearance / self.resolution))

        # раздувание сдвигами: быстрее поячеечного обхода и не тянет scipy
        for dy in range(-cells, cells + 1):
            for dx in range(-cells, cells + 1):
                if dx * dx + dy * dy > cells * cells:
                    continue
                blocked |= np.roll(np.roll(occupied, dy, 0), dx, 1)

        self.passable = ~blocked

        self.get_logger().info(
            f"Карта: проходимо {int(self.passable.sum())} из "
            f"{self.passable.size} ячеек"
        )

    def to_cell(self, point):
        """Координаты мира в индексы сетки

        :point (x, y) в метрах

        :return (строка, столбец)
        """
        return (
            int((point[1] - self.origin[1]) / self.resolution),
            int((point[0] - self.origin[0]) / self.resolution),
        )

    def to_world(self, cell):
        """Индексы сетки в координаты мира

        :cell (строка, столбец)

        :return (x, y) в метрах
        """
        return (
            self.origin[0] + (cell[1] + 0.5) * self.resolution,
            self.origin[1] + (cell[0] + 0.5) * self.resolution,
        )

    def nearest_passable(self, cell):
        """Ближайшая проходимая ячейка

        Робот может оказаться в раздутой зоне у стены: формально непроходимо,
        но ехать откуда-то надо.

        :cell исходная ячейка

        :return проходимая ячейка либо None
        """
        height, width = self.passable.shape

        if (
            0 <= cell[0] < height
            and 0 <= cell[1] < width
            and self.passable[cell]
        ):
            return cell

        for radius in range(1, 20):
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    if max(abs(dy), abs(dx)) != radius:
                        continue
                    probe = (cell[0] + dy, cell[1] + dx)
                    if (
                        0 <= probe[0] < height
                        and 0 <= probe[1] < width
                        and self.passable[probe]
                    ):
                        return probe

        return None

    def find_path(self, start, goal):
        """Найти путь по проходимым ячейкам

        :start начальная ячейка
        :goal целевая ячейка

        :return список ячеек от начала к цели либо None
        """
        height, width = self.passable.shape
        came = {}
        best = {start: 0.0}
        queue = [(0.0, 0.0, start)]

        while queue:
            _, cost, current = heapq.heappop(queue)

            if current == goal:
                break

            if cost > best.get(current, float("inf")):
                continue

            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue

                    following = (current[0] + dy, current[1] + dx)

                    if not (
                        0 <= following[0] < height
                        and 0 <= following[1] < width
                    ):
                        continue

                    if not self.passable[following]:
                        continue

                    step = cost + math.hypot(dx, dy)

                    if step < best.get(following, float("inf")):
                        best[following] = step
                        came[following] = current
                        estimate = step + math.hypot(
                            following[0] - goal[0], following[1] - goal[1]
                        )
                        heapq.heappush(queue, (estimate, step, following))

        if goal not in came and goal != start:
            return None

        path = [goal]
        while path[-1] != start:
            previous = came.get(path[-1])
            if previous is None:
                return None
            path.append(previous)

        return path[::-1]

    def own_position(self):
        """Своя позиция в кадре вывода

        :return (x, y) либо None, если трансформа ещё нет
        """
        try:
            transform = self.buffer.lookup_transform(
                self.output_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=RclDuration(seconds=0.1),
            )
        except tf2_ros.TransformException:
            return None

        return (
            transform.transform.translation.x,
            transform.transform.translation.y,
        )

    def decide(self) -> None:
        """Найти путь и опубликовать ближайшую его точку как цель"""
        if self.passable is None:
            return

        position = self.own_position()

        if position is None:
            return

        start = self.nearest_passable(self.to_cell(position))
        goal = self.nearest_passable(self.to_cell(self.goal))

        if start is None or goal is None:
            return

        cells = self.find_path(start, goal)

        if cells is None:
            self.get_logger().warn(
                "Путь к зоне не найден", throttle_duration_sec=5.0
            )
            return

        points = [self.to_world(cell) for cell in cells]

        # цель для MPC -- точка на расстоянии lookahead по пути: так он ведёт
        # робота вдоль коридора, а не напрямик через стену
        target = points[-1]
        travelled = 0.0

        for first, second in zip(points, points[1:]):
            travelled += math.dist(first, second)
            if travelled >= self.lookahead:
                target = second
                break

        message = PointStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.output_frame
        message.point.x = float(target[0])
        message.point.y = float(target[1])
        self.publisher.publish(message)

        path = Path()
        path.header = message.header
        for point in points:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.path_publisher.publish(path)


def main():
    rclpy.init()
    node = AttackStrategy()
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
