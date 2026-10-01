#!/usr/bin/env python3

"""Планировщик на потенциальном поле для дуэли.

Путь строится градиентным спуском по полю: цель притягивает, стены и
соперник отталкивают. Одна нода на обе роли, поведение задаётся параметром
role.

  attacker -- цель это синяя зона, соперник отталкивает;
  defender -- цель это перехват атакующего, отталкивают только стены.

Защитник целится не в текущую позу соперника, а в точку его предсказанного
пути: погоня следом всегда отстаёт, а перехват срезает угол. Точка выбирается
по времени сближения, то есть с учётом того, что защитник тоже едет.

Отталкивание от соперника у атакующего берётся по всему предсказанному пути,
а не по одной текущей позе: так он обходит место, куда защитник только
собирается.

Публикуется nav_msgs/Path -- тот же тип, что у прежнего планировщика, чтобы
контроллер MPC не менять.
"""

import math

import numpy as np
import rclpy
import tf2_ros

from autoware_perception_msgs.msg import PredictedObjects
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)


OCCUPIED = 100


class PotentialFieldPlanner(Node):
    """Строит путь градиентным спуском по потенциальному полю"""

    def __init__(self):
        super().__init__("potential_field_planner")

        self.declare_parameter("role", "attacker")
        self.declare_parameter("base_frame", "attacker/base_footprint")
        self.declare_parameter("output_frame", "world")
        self.declare_parameter("goal", [3.5, 0.0])
        self.declare_parameter("rate", 50.0)

        # Шаг спуска и длина пути. Шаг мельче -- путь глаже, но точек больше.
        # Точек хватает на 15 м: диагональ totami около 11 м, плюс запас на
        # обход соперника, иначе путь обрывается на полдороге.
        self.declare_parameter("step", 0.1)
        self.declare_parameter("max_points", 150)

        # Коэффициенты поля. Притяжение линейно по расстоянию, отталкивание
        # спадает гауссианой: у неё нет разрыва на границе радиуса, из-за
        # которого путь дёргается, когда препятствие то входит в радиус, то
        # выходит.
        self.declare_parameter("attraction", 1.0)
        self.declare_parameter("wall_repulsion", 2.0)
        self.declare_parameter("wall_sigma", 0.45)
        self.declare_parameter("opponent_repulsion", 3.0)
        self.declare_parameter("opponent_sigma", 0.8)

        # Перехват: насколько далеко по предсказанному пути смотреть и с
        # какой скоростью защитник рассчитывает сближаться.
        self.declare_parameter("intercept_speed", 0.5)

        # Радиус, внутри которого отталкивание затухает к нулю у цели.
        self.declare_parameter("goal_clearance", 1.5)

        # Визуализация поля: отдельная частота, потому что сетка на порядок
        # дороже одного спуска. Считается только при наличии подписчиков.
        self.declare_parameter("field_resolution", 0.1)
        self.declare_parameter("field_rate", 2.0)

        self.role = self.get_parameter("role").value
        self.base_frame = self.get_parameter("base_frame").value
        self.output_frame = self.get_parameter("output_frame").value
        self.goal = np.array(self.get_parameter("goal").value, dtype=float)
        self.step = self.get_parameter("step").value
        self.max_points = self.get_parameter("max_points").value
        self.attraction = self.get_parameter("attraction").value
        self.wall_repulsion = self.get_parameter("wall_repulsion").value
        self.wall_sigma = self.get_parameter("wall_sigma").value
        self.opponent_repulsion = self.get_parameter("opponent_repulsion").value
        self.opponent_sigma = self.get_parameter("opponent_sigma").value
        self.intercept_speed = self.get_parameter("intercept_speed").value
        self.goal_clearance = self.get_parameter("goal_clearance").value

        self.field_resolution = self.get_parameter("field_resolution").value

        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)

        self.walls = None
        self.map_bounds = None
        self.opponent_path = []

        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, "/map", self.on_map, map_qos)
        self.create_subscription(
            PredictedObjects,
            "perception/object_recognition/objects",
            self.on_objects,
            1,
        )

        self.publisher = self.create_publisher(Path, "planning/trajectory", 1)
        self.field_publisher = self.create_publisher(
            OccupancyGrid,
            "planning/potential_field",
            1,
        )

        self.goal_shown = self.goal
        self.create_timer(1.0 / self.get_parameter("rate").value, self.plan)
        self.create_timer(
            1.0 / self.get_parameter("field_rate").value,
            lambda: self.publish_field(self.goal_shown),
        )

        self.get_logger().info(
            f"Планировщик на потенциальном поле, роль {self.role}, "
            f"цель {self.goal.tolist()}, кадр {self.output_frame}"
        )

    def on_map(self, message: OccupancyGrid) -> None:
        """Запомнить занятые ячейки как координаты стен

        Поле считается по точкам, а не по сетке: занятых ячеек мало (в totami
        меньше тысячи), и расстояние до них векторизуется numpy без обхода
        всей карты на каждом шаге спуска.

        :message карта занятости
        """
        grid = np.array(message.data, dtype=np.int8).reshape(
            message.info.height, message.info.width
        )
        rows, columns = np.nonzero(grid == OCCUPIED)

        resolution = message.info.resolution
        origin = message.info.origin.position

        self.walls = np.column_stack(
            (
                origin.x + (columns + 0.5) * resolution,
                origin.y + (rows + 0.5) * resolution,
            )
        )

        # границы карты: по ним строится сетка визуализации поля
        self.map_bounds = (
            origin.x,
            origin.y,
            origin.x + message.info.width * resolution,
            origin.y + message.info.height * resolution,
        )

        self.get_logger().info(f"Карта: {len(self.walls)} занятых ячеек")

    def on_objects(self, message: PredictedObjects) -> None:
        """Запомнить предсказанный путь соперника

        :message объекты восприятия
        """
        if not message.objects:
            self.opponent_path = []
            return

        obstacle = message.objects[0]
        path = [
            (
                obstacle.kinematics.initial_pose_with_covariance.pose.position.x,
                obstacle.kinematics.initial_pose_with_covariance.pose.position.y,
            )
        ]

        if obstacle.kinematics.predicted_paths:
            predicted = obstacle.kinematics.predicted_paths[0]
            path = [(pose.position.x, pose.position.y) for pose in predicted.path]

        self.opponent_path = path

    def own_position(self):
        """Своя поза в кадре вывода

        :return numpy-вектор (x, y) либо None, если трансформа ещё нет
        """
        try:
            # без таймаута: ждать здесь нельзя, ожидание блокирует поток и
            # роняет частоту перепланирования до частоты прихода TF
            transform = self.buffer.lookup_transform(
                self.output_frame,
                self.base_frame,
                rclpy.time.Time(),
            )
        except tf2_ros.TransformException:
            return None

        return np.array(
            [
                transform.transform.translation.x,
                transform.transform.translation.y,
            ]
        )

    def intercept_point(self, start):
        """Точка перехвата на предсказанном пути соперника

        Идём по предсказанным позам и берём первую, до которой защитник
        успевает раньше соперника. Если не успевает никуда, целимся в конец
        пути: там разрыв наименьший.

        :start своя позиция

        :return numpy-вектор цели
        """
        if not self.opponent_path:
            return None

        # шаг прогноза детектора; при одной точке падать некуда
        horizon_step = 0.5

        for index, point in enumerate(self.opponent_path):
            target = np.array(point)
            reach = np.linalg.norm(target - start) / max(
                self.intercept_speed, 1e-3
            )
            if reach <= index * horizon_step:
                return target

        return np.array(self.opponent_path[-1])

    def repulsive_points(self):
        """Точки, от которых путь отталкивается

        Стены отталкивают обе роли. Соперник отталкивает только атакующего:
        защитнику он цель, а не препятствие.

        :return (массив точек, коэффициент, сигма) для стен и соперника
        """
        groups = []

        if self.walls is not None and len(self.walls):
            groups.append(
                (self.walls, self.wall_repulsion, self.wall_sigma)
            )

        if self.role == "attacker" and self.opponent_path:
            groups.append(
                (
                    np.array(self.opponent_path),
                    self.opponent_repulsion,
                    self.opponent_sigma,
                )
            )

        return groups

    def descend(self, start, goal):
        """Построить путь градиентным спуском

        :start своя позиция
        :goal цель

        :return список точек пути
        """
        groups = self.repulsive_points()
        # сторона обхода выбирается один раз на путь, см. локальный минимум
        self.detour = 0.0
        points = [start.copy()]
        point = start.copy()
        closest = float("inf")
        stalled = 0

        for _ in range(self.max_points):
            to_goal = goal - point
            distance = np.linalg.norm(to_goal)

            if distance < self.step:
                points.append(goal.copy())
                break

            # Спуск застрял: цель не приближается несколько шагов подряд.
            # Так бывает, когда поле уравновесилось, а обход водит по кругу;
            # без этой проверки путь дорисовывал сотню точек на одном месте и
            # контроллер получал клубок вместо траектории.
            if distance < closest - 1e-3:
                closest = distance
                stalled = 0
            else:
                stalled += 1
                if stalled > 20:
                    break

            force = self.attraction * to_goal / max(distance, 1e-6)

            # Чем ближе цель, тем слабее отталкивание. Без этого цель у самой
            # стены недостижима: в синей зоне totami до стены 0.5 м, и на
            # таком расстоянии отталкивание превышает притяжение, так что
            # спуск встаёт, не дойдя.
            # Квадратично, а не линейно: цель синей зоны стоит в 0.5 м от
            # восточной стены, и при линейном затухании отталкивание у цели
            # оставалось почти равным притяжению. Силы гасили друг друга,
            # спуск срывался в обход и наматывал петли, не доходя 0.66 м.
            fade = min(1.0, distance / self.goal_clearance) ** 2

            for obstacles, strength, sigma in groups:
                delta = point - obstacles
                distances = np.linalg.norm(delta, axis=1)
                distances[distances < 1e-6] = 1e-6

                # отсекаем далёкие: на расстоянии больше 3 сигм вклад
                # пренебрежимо мал, а точек в карте тысячи
                close = distances < 3.0 * sigma
                if not close.any():
                    continue

                weights = np.exp(
                    -(distances[close] ** 2) / (2 * sigma ** 2)
                )
                contribution = (
                    delta[close] / distances[close, None] * weights[:, None]
                ).sum(axis=0)
                force = force + strength * fade * contribution

            norm = np.linalg.norm(force)

            # Локальный минимум: препятствие ровно между нами и целью, силы
            # погасили друг друга. Чистое поле здесь встаёт намертво, поэтому
            # сворачиваем вбок -- по касательной к направлению на цель. Знак
            # запоминается, иначе на следующем шаге спуск качнётся обратно и
            # путь задрожит на месте.
            if norm < self.attraction * 0.2:
                direction = to_goal / max(distance, 1e-6)
                tangent = np.array([-direction[1], direction[0]])

                if self.detour == 0.0:
                    # обходим с той стороны, куда уже смещает поле
                    self.detour = 1.0 if np.dot(force, tangent) >= 0 else -1.0

                force = force + self.attraction * self.detour * tangent
                norm = np.linalg.norm(force)

            if norm < 1e-6:
                break

            point = point + self.step * force / norm
            points.append(point.copy())

        return points

    def publish_field(self, goal) -> None:
        """Опубликовать потенциал на сетке для наглядности

        Планировщик сам сетку не строит -- он берёт градиент в точке и сразу
        шагает. Эта карта нужна только чтобы увидеть, куда поле толкает
        робота, и подобрать коэффициенты отталкивания.

        Публикуется потенциал, а не силы: спуск ищет именно его минимум.
        Значения нормируются в 0..100, так как OccupancyGrid хранит байты;
        абсолютная величина смысла не имеет, важен рельеф.

        :goal текущая цель роли
        """
        if self.map_bounds is None:
            return

        if self.field_publisher.get_subscription_count() == 0:
            # сетка на порядок дороже одного спуска, поэтому считаем её
            # только когда кто-то смотрит
            return

        min_x, min_y, max_x, max_y = self.map_bounds
        resolution = self.field_resolution

        width = int((max_x - min_x) / resolution)
        height = int((max_y - min_y) / resolution)

        xs = min_x + (np.arange(width) + 0.5) * resolution
        ys = min_y + (np.arange(height) + 0.5) * resolution
        mesh_x, mesh_y = np.meshgrid(xs, ys)
        cells = np.column_stack((mesh_x.ravel(), mesh_y.ravel()))

        # притяжение: линейно растёт с расстоянием до цели, поэтому в самой
        # цели у потенциала минимум
        distance_to_goal = np.linalg.norm(cells - goal, axis=1)
        potential = self.attraction * distance_to_goal

        fade = np.minimum(1.0, distance_to_goal / self.goal_clearance) ** 2

        for obstacles, strength, sigma in self.repulsive_points():
            # Расстояние от каждой ячейки до ближайшего препятствия группы:
            # гауссиана быстро спадает, поэтому ближайшего достаточно.
            #
            # Считаем блоками по ячейкам: разность всех со всеми для сетки
            # 100x100 и 639 стен заняла бы сотню мегабайт единым массивом.
            nearest = np.empty(len(cells))
            block = 512

            for begin in range(0, len(cells), block):
                chunk = cells[begin:begin + block]
                deltas = chunk[:, None, :] - obstacles[None, :, :]
                nearest[begin:begin + block] = np.linalg.norm(
                    deltas, axis=2
                ).min(axis=1)

            potential += (
                strength * sigma * fade
                * np.exp(-(nearest ** 2) / (2 * sigma ** 2))
            )

        low = potential.min()
        high = potential.max()
        scaled = (potential - low) / max(high - low, 1e-6) * 100.0

        field = OccupancyGrid()
        field.header.stamp = self.get_clock().now().to_msg()
        field.header.frame_id = self.output_frame
        field.info.resolution = resolution
        field.info.width = width
        field.info.height = height
        field.info.origin.position.x = min_x
        field.info.origin.position.y = min_y
        field.info.origin.orientation.w = 1.0
        field.data = scaled.astype(np.int8).reshape(-1).tolist()

        self.field_publisher.publish(field)

    def plan(self) -> None:
        """Построить и опубликовать путь"""
        start = self.own_position()

        if start is None:
            return

        if self.role == "defender":
            goal = self.intercept_point(start)
            if goal is None:
                return
        else:
            goal = self.goal

        # цель защитника меняется каждый тик, а поле рисуется реже и своим
        # таймером, поэтому оно берёт последнюю использованную
        self.goal_shown = goal

        points = self.descend(start, goal)

        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.output_frame

        for point in points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        # курс в каждой точке направлен по пути: контроллеру нужна
        # ориентация, а спуск даёт только позиции
        for index in range(len(path.poses) - 1):
            current = path.poses[index].pose.position
            following = path.poses[index + 1].pose.position
            yaw = math.atan2(
                following.y - current.y, following.x - current.x
            )
            path.poses[index].pose.orientation.z = math.sin(yaw / 2.0)
            path.poses[index].pose.orientation.w = math.cos(yaw / 2.0)

        if len(path.poses) > 1:
            path.poses[-1].pose.orientation = path.poses[-2].pose.orientation

        self.publisher.publish(path)


def main():
    rclpy.init()
    node = PotentialFieldPlanner()
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
