#!/usr/bin/env python3

"""Игровой MPC: план против худшего ответа соперника.

Планировщик решает робастную задачу

    U_A* = argmin_{U_A} max_{U_D} J(U_A, U_D),

то есть выбирает манёвр, который лучше всего сохраняет возможность выполнить
задачу даже при самом опасном ответе соперника. Отличие от потенциального
поля в том, что соперник здесь не препятствие с фиксированной позой, а
игрок, который на наш манёвр отвечает своим.

Минимакс решается перебором по сетке: конечный набор своих манёвров против
конечного набора ответов соперника, каждая пара прокатывается по модели
робота. Перебор выбран не для простоты -- внутренняя задача max невыпуклая,
и градиентные решатели на ней не дают гарантий, сходясь к седловым точкам.
Сетка берёт честный максимум, а стоит она доли миллисекунды, так как все
пары катятся разом средствами numpy.

Роли симметричны и отличаются знаком J: атакующий его минимизирует, защитник
максимизирует. Публикуется nav_msgs/Path, как и прежде, поэтому контроллер
MPC остаётся прежним.
"""

import math

import numpy as np
import rclpy
import tf2_ros

from autoware_perception_msgs.msg import PredictedObjects
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.duration import Duration as RclDuration
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)


OCCUPIED = 100


class GameMpcPlanner(Node):
    """Минимакс-MPC: манёвр против худшего ответа соперника"""

    def __init__(self):
        super().__init__("game_mpc_planner")

        self.declare_parameter("role", "attacker")
        self.declare_parameter("base_frame", "attacker/base_footprint")
        self.declare_parameter("output_frame", "world")
        self.declare_parameter("goal", [3.5, 0.0])
        self.declare_parameter("rate", 20.0)

        # Горизонт. Короткий не видит перехвата, длинный делает прогноз
        # ответа соперника недостоверным: он тоже пересчитывает план.
        self.declare_parameter("horizon", 2.0)
        self.declare_parameter("dt", 0.1)

        # Пределы робота. Взяты из контроллера MPC, который отрабатывает
        # путь: планировать то, что он не отследит, бессмысленно.
        self.declare_parameter("v_max", 0.5)
        self.declare_parameter("v_min", 0.0)
        self.declare_parameter("w_max", 1.5)

        # Сетка манёвров: сколько вариантов скорости и поворота перебирать.
        self.declare_parameter("speed_options", 3)
        self.declare_parameter("turn_options", 11)

        # Сетка ответов соперника. Грубее своей: его точная реакция всё равно
        # неизвестна, важно накрыть диапазон от прямого преследования до
        # перехвата.
        self.declare_parameter("opponent_turn_options", 9)
        self.declare_parameter("opponent_v_max", 0.5)

        # Веса J. Расстояние до цели -- основная часть, остальное штрафы.
        self.declare_parameter("goal_weight", 1.0)
        self.declare_parameter("catch_penalty", 50.0)
        self.declare_parameter("wall_penalty", 20.0)
        self.declare_parameter("effort_weight", 0.05)

        # Дистанции. catch_distance -- зазор между краями, как у судьи.
        self.declare_parameter("catch_distance", 0.1)
        self.declare_parameter("robot_radius", 0.175)
        self.declare_parameter("wall_clearance", 0.25)

        self.role = self.get_parameter("role").value
        self.base_frame = self.get_parameter("base_frame").value
        self.output_frame = self.get_parameter("output_frame").value
        self.goal = np.array(self.get_parameter("goal").value, dtype=float)
        self.horizon = self.get_parameter("horizon").value
        self.dt = self.get_parameter("dt").value
        self.v_max = self.get_parameter("v_max").value
        self.v_min = self.get_parameter("v_min").value
        self.w_max = self.get_parameter("w_max").value
        self.opponent_v_max = self.get_parameter("opponent_v_max").value
        self.goal_weight = self.get_parameter("goal_weight").value
        self.catch_penalty = self.get_parameter("catch_penalty").value
        self.wall_penalty = self.get_parameter("wall_penalty").value
        self.effort_weight = self.get_parameter("effort_weight").value
        self.catch_distance = self.get_parameter("catch_distance").value
        self.robot_radius = self.get_parameter("robot_radius").value
        self.wall_clearance = self.get_parameter("wall_clearance").value

        self.steps = max(1, int(round(self.horizon / self.dt)))

        # Сетки управлений строятся один раз: они не зависят от состояния.
        speeds = np.linspace(
            self.v_min,
            self.v_max,
            self.get_parameter("speed_options").value,
        )
        turns = np.linspace(
            -self.w_max,
            self.w_max,
            self.get_parameter("turn_options").value,
        )
        self.own_controls = np.array(
            [(v, w) for v in speeds for w in turns]
        )

        opponent_turns = np.linspace(
            -self.w_max,
            self.w_max,
            self.get_parameter("opponent_turn_options").value,
        )
        self.opponent_controls = np.array(
            [(self.opponent_v_max, w) for w in opponent_turns]
        )

        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)

        self.walls = None
        self.opponent_state = None

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
        self.create_timer(1.0 / self.get_parameter("rate").value, self.plan)

        self.get_logger().info(
            f"Игровой MPC, роль {self.role}: {len(self.own_controls)} манёвров "
            f"против {len(self.opponent_controls)} ответов, горизонт "
            f"{self.horizon} с шагом {self.dt} с"
        )

    def on_map(self, message: OccupancyGrid) -> None:
        """Запомнить занятые ячейки карты

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

        self.get_logger().info(f"Карта: {len(self.walls)} занятых ячеек")

    def on_objects(self, message: PredictedObjects) -> None:
        """Запомнить позу и курс соперника

        Берётся только текущее состояние: его предсказанный путь здесь не
        нужен, ответы соперника планировщик перебирает сам.

        :message объекты восприятия
        """
        if not message.objects:
            self.opponent_state = None
            return

        kinematics = message.objects[0].kinematics
        pose = kinematics.initial_pose_with_covariance.pose
        q = pose.orientation

        self.opponent_state = np.array(
            [
                pose.position.x,
                pose.position.y,
                math.atan2(
                    2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z),
                ),
            ]
        )

    def own_state(self):
        """Своя поза и курс в кадре вывода

        :return массив (x, y, yaw) либо None, если трансформа ещё нет
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

        q = transform.transform.rotation

        return np.array(
            [
                transform.transform.translation.x,
                transform.transform.translation.y,
                math.atan2(
                    2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z),
                ),
            ]
        )

    def rollout(self, state, controls):
        """Прокатить модель робота для набора управлений

        Модель -- одноколейный робот с постоянным управлением на горизонте:
        его менять незачем, так как план всё равно пересчитывается каждый
        тик, а исполняется только первый участок.

        :state начальное (x, y, yaw)
        :controls массив (N, 2) из пар скорость и поворот

        :return массив (N, steps, 2) с траекториями
        """
        count = len(controls)
        x = np.full(count, state[0])
        y = np.full(count, state[1])
        yaw = np.full(count, state[2])

        v = controls[:, 0]
        w = controls[:, 1]

        track = np.empty((count, self.steps, 2))

        for step in range(self.steps):
            yaw = yaw + w * self.dt
            x = x + v * np.cos(yaw) * self.dt
            y = y + v * np.sin(yaw) * self.dt
            track[:, step, 0] = x
            track[:, step, 1] = y

        return track

    def wall_cost(self, track):
        """Штраф за приближение к стенам

        :track траектории (N, steps, 2)

        :return вектор штрафов по траекториям
        """
        if self.walls is None or not len(self.walls):
            return np.zeros(len(track))

        points = track.reshape(-1, 2)
        nearest = np.empty(len(points))
        block = 1024

        # блоками: разность всех точек со всеми стенами единым массивом
        # заняла бы сотни мегабайт
        for begin in range(0, len(points), block):
            chunk = points[begin:begin + block]
            deltas = chunk[:, None, :] - self.walls[None, :, :]
            nearest[begin:begin + block] = np.linalg.norm(
                deltas, axis=2
            ).min(axis=1)

        nearest = nearest.reshape(len(track), self.steps)

        # штраф растёт, когда зазор до стены меньше запаса
        violation = np.maximum(0.0, self.wall_clearance - nearest)
        return self.wall_penalty * violation.sum(axis=1)

    def cost(self, own_track, opponent_track):
        """Матрица J для всех пар своих манёвров и ответов соперника

        J всегда считается с точки зрения атакующего: меньше -- лучше ему.
        Поэтому расстояние до цели берётся от траектории атакующего, кто бы
        ни вызывал метод. Для защитника это чужая траектория: его задача не
        стоять у зоны, а не пускать туда соперника.

        :own_track свои траектории (A, steps, 2)
        :opponent_track траектории соперника (D, steps, 2)

        :return матрица (A, D) значений J
        """
        # чья траектория идёт к цели: у защитника это соперник
        if self.role == "attacker":
            goal_track = own_track
        else:
            goal_track = opponent_track
        # Расстояние до цели: берём минимум по горизонту, а не конечную
        # точку. Иначе манёвр, который касается зоны и проезжает дальше,
        # считался бы хуже того, что к ней только подходит.
        to_goal = np.linalg.norm(goal_track - self.goal, axis=2)

        # Минимум по горизонту, а не конечная точка: манёвр, который касается
        # зоны и проезжает дальше, иначе считался бы хуже того, что к ней
        # только подходит.
        #
        # Плюс средняя удалённость за горизонт: без неё стоянка на месте
        # стоит столько же, сколько движение к цели, и при близком сопернике
        # планировщик предпочитал замереть, лишь бы не нарваться на штраф.
        goal_cost = self.goal_weight * (
            to_goal.min(axis=1) + 0.5 * to_goal.mean(axis=1)
        )

        # приводим к форме матрицы (A, D): у атакующего цель зависит от его
        # манёвра, у защитника -- от ответа соперника
        if self.role == "attacker":
            goal_term = goal_cost[:, None]
        else:
            goal_term = goal_cost[None, :]

        # зазор между краями роботов на каждом шаге для каждой пары
        gap = (
            np.linalg.norm(
                own_track[:, None, :, :] - opponent_track[None, :, :, :],
                axis=3,
            )
            - 2.0 * self.robot_radius
        )

        # Поимка: штраф тем больше, чем глубже нарушена стоп-дистанция и чем
        # раньше это случилось. Вес падает по шагам горизонта, иначе ранняя и
        # поздняя поимка стоят почти одинаково, и при неизбежном контакте
        # манёвр, оттягивающий его, ничем не лучше лобового.
        violation = np.maximum(0.0, self.catch_distance - gap)
        decay = np.linspace(1.0, 0.1, self.steps)
        catch_cost = self.catch_penalty * (violation * decay).sum(axis=2)

        return goal_term + catch_cost

    def plan(self) -> None:
        """Решить минимакс и опубликовать выбранную траекторию"""
        state = self.own_state()

        if state is None:
            return

        own_track = self.rollout(state, self.own_controls)

        if self.opponent_state is None:
            # Соперник ещё не виден: ставим его бесконечно далеко, тогда
            # штраф за поимку равен нулю и задача вырождается в обычный MPC
            # до цели.
            opponent_track = np.full((1, self.steps, 2), 1e3)
        else:
            opponent_track = self.rollout(
                self.opponent_state, self.opponent_controls
            )

        payoff = self.cost(own_track, opponent_track)

        # стены и усилие зависят только от своего манёвра, поэтому входят
        # вне минимакса
        own_penalty = self.wall_cost(own_track)
        own_penalty = own_penalty + self.effort_weight * np.abs(
            self.own_controls[:, 1]
        )

        if self.role == "attacker":
            # худший ответ соперника на каждый наш манёвр, затем лучший
            # манёвр против этого худшего случая
            worst = payoff.max(axis=1) + own_penalty
            best = int(np.argmin(worst))
        else:
            # защитник максимизирует ту же J, поэтому худший случай для него
            # это минимум по ответам атакующего, а штрафы по-прежнему вычитают
            worst = payoff.min(axis=1) - own_penalty
            best = int(np.argmax(worst))

        self.publish(own_track[best], state)

    def publish(self, track, state) -> None:
        """Опубликовать выбранную траекторию

        :track выбранная траектория (steps, 2)
        :state текущее состояние робота
        """
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.output_frame

        points = np.vstack(([state[:2]], track))

        for index, point in enumerate(points):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])

            # курс направлен по траектории: контроллеру нужна ориентация
            if index + 1 < len(points):
                following = points[index + 1]
                yaw = math.atan2(
                    following[1] - point[1], following[0] - point[0]
                )
            else:
                yaw = state[2]

            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            path.poses.append(pose)

        if len(path.poses) > 1:
            path.poses[-1].pose.orientation = path.poses[-2].pose.orientation

        self.publisher.publish(path)


def main():
    rclpy.init()
    node = GameMpcPlanner()
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
