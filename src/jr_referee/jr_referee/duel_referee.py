#!/usr/bin/env python3

"""Судья соревнования: следит за условиями победы и останавливает роботов.

Соревнование заканчивается в двух случаях:

  атакующий заехал в синюю зону  -- победа атакующего;
  защитник подошёл к атакующему ближе стоп-дистанции -- победа защитника.

Проверяется первое выполнившееся условие; дальше исход не меняется, даже
если потом выполнится второе. Роботы останавливаются сразу: судья выставляет
параметр stopped обоим контурам MPC каждого робота, а тот обнуляет выход в
ближайшем такте, не дожидаясь таймаута телеметрии.

Попадание в зону считается по пересечению футпринта робота с прямоугольником
зоны, а не по центру: робот заехал, как только коснулся её колесом.
"""

import math

import rclpy

from geometry_msgs.msg import Point
from jr_msgs.msg import DuelStatus
from nav_msgs.msg import Path
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.duration import Duration as RclDuration
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from visualization_msgs.msg import Marker

import tf2_ros


class DuelReferee(Node):
    """Следит за условиями победы и объявляет исход"""

    def __init__(self):
        super().__init__("duel_referee")

        self.declare_parameter("attacker_frame", "attacker/base_footprint")
        self.declare_parameter("defender_frame", "defender/base_footprint")
        self.declare_parameter("output_frame", "world")

        # Синяя зона totami: центр и полуразмеры прямоугольника.
        self.declare_parameter("zone_center", [3.5, 0.0])
        self.declare_parameter("zone_half_size", [0.5, 1.0])

        # Радиус футпринта робота: Kobuki круглый, диаметр корпуса 0.35 м.
        self.declare_parameter("robot_radius", 0.175)

        # Дистанция поимки: зазор между краями роботов, а не между их
        # центрами. Оба футпринта круглые, поэтому из расстояния между
        # центрами вычитаются два радиуса.
        self.declare_parameter("catch_distance", 0.1)

        self.declare_parameter("rate", 20.0)

        self.attacker_frame = self.get_parameter("attacker_frame").value
        self.defender_frame = self.get_parameter("defender_frame").value
        self.output_frame = self.get_parameter("output_frame").value
        self.zone_center = self.get_parameter("zone_center").value
        self.zone_half = self.get_parameter("zone_half_size").value
        self.robot_radius = self.get_parameter("robot_radius").value
        self.catch_distance = self.get_parameter("catch_distance").value

        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)

        self.attacker_path = None
        self.result = DuelStatus.IN_PROGRESS
        self.stop_sent = False

        self.create_subscription(
            Path,
            "/attacker/planning/trajectory",
            self.on_path,
            1,
        )

        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(DuelStatus, "duel/status", qos)
        self.zone_marker = self.create_publisher(Marker, "duel/zone", qos)

        # Контуры MPC, которым судья выставляет stopped. Имена нод зависят от
        # режима продольного контура, поэтому берём оба возможных: лишний
        # вызов на несуществующую ноду просто не найдёт сервис.
        self.controllers = [
            f"/{robot}/control/{node}"
            for robot in ("attacker", "defender")
            for node in (
                "swarm_cc_mpc_node",
                "swarm_acc_mpc_node",
                "swarm_lat_mpc_node",
            )
        ]

        self.create_timer(1.0 / self.get_parameter("rate").value, self.judge)

        self.get_logger().info(
            f"Судья: зона {self.zone_center} +-{self.zone_half}, "
            f"поимка при зазоре {self.catch_distance} м между краями, "
            f"футпринт {self.robot_radius} м"
        )

    def on_path(self, message: Path) -> None:
        """Запомнить путь атакующего для длины вдоль траектории

        :message путь планировщика
        """
        self.attacker_path = message

    def position(self, frame: str):
        """Позиция робота в кадре вывода

        :frame фрейм робота

        :return (x, y) либо None, если трансформа ещё нет
        """
        try:
            transform = self.buffer.lookup_transform(
                self.output_frame,
                frame,
                rclpy.time.Time(),
                timeout=RclDuration(seconds=0.1),
            )
        except tf2_ros.TransformException:
            return None

        return (
            transform.transform.translation.x,
            transform.transform.translation.y,
        )

    def in_zone(self, position) -> bool:
        """Пересекает ли футпринт робота синюю зону

        Футпринт круглый, зона прямоугольная, поэтому берём ближайшую точку
        прямоугольника к центру робота: если она ближе радиуса, фигуры
        пересеклись.

        :position позиция робота

        :return True если робот коснулся зоны
        """
        nearest_x = min(
            max(position[0], self.zone_center[0] - self.zone_half[0]),
            self.zone_center[0] + self.zone_half[0],
        )
        nearest_y = min(
            max(position[1], self.zone_center[1] - self.zone_half[1]),
            self.zone_center[1] + self.zone_half[1],
        )

        gap = math.hypot(position[0] - nearest_x, position[1] - nearest_y)
        return gap <= self.robot_radius

    def path_length(self, position) -> float:
        """Длина пути атакующего до цели вдоль траектории

        Считается от текущей позы робота: точки пути, оставшиеся позади,
        планировщик уже отбросил, так как строит путь от своей позиции.

        :position позиция атакующего

        :return длина в метрах, NaN если пути нет
        """
        if self.attacker_path is None or len(self.attacker_path.poses) < 2:
            return float("nan")

        points = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in self.attacker_path.poses
        ]

        total = math.dist(position, points[0])
        for first, second in zip(points, points[1:]):
            total += math.dist(first, second)

        return total

    def stop_robots(self) -> None:
        """Выставить stopped всем контурам управления

        Вызывается один раз: повторять незачем, параметр остаётся выставлен.
        """
        if self.stop_sent:
            return

        self.stop_sent = True

        parameter = Parameter()
        parameter.name = "stopped"
        parameter.value = ParameterValue()
        parameter.value.type = ParameterType.PARAMETER_BOOL
        parameter.value.bool_value = True

        for node in self.controllers:
            client = self.create_client(SetParameters, f"{node}/set_parameters")

            if not client.wait_for_service(timeout_sec=0.5):
                # у робота может не быть этого режима продольного контура
                continue

            request = SetParameters.Request()
            request.parameters = [parameter]
            client.call_async(request)
            self.get_logger().info(f"{node}: stopped -> true")

    def publish_zone(self) -> None:
        """Нарисовать синюю зону для RViz"""
        marker = Marker()
        marker.header.frame_id = self.output_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "duel"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.03
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.pose.orientation.w = 1.0

        left = self.zone_center[0] - self.zone_half[0]
        right = self.zone_center[0] + self.zone_half[0]
        bottom = self.zone_center[1] - self.zone_half[1]
        top = self.zone_center[1] + self.zone_half[1]

        for x, y in (
            (left, bottom),
            (right, bottom),
            (right, top),
            (left, top),
            (left, bottom),
        ):
            point = Point()
            point.x = float(x)
            point.y = float(y)
            marker.points.append(point)

        self.zone_marker.publish(marker)

    def judge(self) -> None:
        """Проверить условия и опубликовать статус"""
        attacker = self.position(self.attacker_frame)
        defender = self.position(self.defender_frame)

        if attacker is None or defender is None:
            return

        self.publish_zone()

        in_zone = self.in_zone(attacker)

        # зазор между краями: расстояние между центрами минус два радиуса
        distance = math.dist(attacker, defender) - 2.0 * self.robot_radius
        caught = distance <= self.catch_distance

        # исход фиксируется первым выполнившимся условием и дальше не
        # меняется, даже если сработает второе
        if self.result == DuelStatus.IN_PROGRESS:
            if in_zone:
                self.result = DuelStatus.ATTACKER_WON
                self.get_logger().info("Атакующий заехал в синюю зону")
            elif caught:
                self.result = DuelStatus.DEFENDER_WON
                self.get_logger().info(
                    f"Защитник догнал атакующего: {distance:.3f} м"
                )

            if self.result != DuelStatus.IN_PROGRESS:
                self.stop_robots()

        status = DuelStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.header.frame_id = self.output_frame
        status.result = self.result
        status.goal_distance_euclidean = math.dist(attacker, self.zone_center)
        status.goal_distance_path = self.path_length(attacker)
        status.robots_distance = distance
        status.attacker_in_zone = in_zone
        status.defender_caught = caught

        self.publisher.publish(status)


def main():
    rclpy.init()
    node = DuelReferee()
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
