#!/usr/bin/env python3

"""Фейковый детектор соперника: поза из TF вместо распознавания.

Отдаёт всю цепочку восприятия Autoware -- detection, tracking, prediction --
но берёт позу соперника из дерева TF, а не из облака лидара. Это заглушка
под настоящий детектор: топики, типы сообщений и частоты те же, поэтому
подписчиков менять не придётся, когда детектор появится.

Чего заглушка не воспроизводит: соперник виден всегда, даже за стеной и вне
поля зрения, существование достоверно, а ковариации нулевые. Скорость и
ускорение считаются численно по приходящим позам, направление движения --
из скорости, поэтому у стоящего робота оно берётся из его ориентации.
"""

import math

import rclpy
import tf2_ros

from autoware_perception_msgs.msg import (
    DetectedObject,
    DetectedObjectKinematics,
    DetectedObjects,
    ObjectClassification,
    PredictedObject,
    PredictedObjects,
    PredictedPath,
    Shape,
    TrackedObject,
    TrackedObjectKinematics,
    TrackedObjects,
)
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose
from rclpy.duration import Duration as RclDuration
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from unique_identifier_msgs.msg import UUID


# Kobuki в габаритном прямоугольнике: диаметр корпуса и высота с башенкой.
ROBOT_SIZE = (0.35, 0.35, 0.42)

# Соперник -- робот, а не участник дорожного движения. В классификации
# Autoware ближе всего UNKNOWN: любой другой класс тянет за собой чужие
# допущения о поведении в планировщике.
ROBOT_LABEL = ObjectClassification.UNKNOWN


def yaw_of(pose: Pose) -> float:
    """Достать курс из кватерниона позы

    :pose поза объекта

    :return курс в радианах
    """
    q = pose.orientation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def yaw_to_quaternion(pose: Pose, yaw: float) -> None:
    """Записать курс в ориентацию позы

    :pose поза, которую правим на месте
    :yaw курс в радианах
    """
    pose.orientation.z = math.sin(yaw / 2.0)
    pose.orientation.w = math.cos(yaw / 2.0)


class FakeOpponentDetector(Node):
    """Публикует соперника как объект восприятия, беря позу из TF"""

    def __init__(self):
        super().__init__("fake_opponent_detector")

        self.declare_parameter("target_frame", "attacker/base_footprint")
        self.declare_parameter("reference_frame", "base_footprint")
        self.declare_parameter("output_frame", "map")
        self.declare_parameter("rate", 10.0)
        self.declare_parameter("prediction_horizon", 3.0)
        self.declare_parameter("prediction_step", 0.5)

        self.target_frame = self.get_parameter("target_frame").value
        self.output_frame = self.get_parameter("output_frame").value
        self.horizon = self.get_parameter("prediction_horizon").value
        self.step = self.get_parameter("prediction_step").value
        rate = self.get_parameter("rate").value

        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)

        # UUID постоянен на всё время работы: соперник один и тот же объект,
        # и трекер, который придёт на смену заглушке, обязан держать id
        # стабильным между кадрами.
        self.object_id = UUID()
        self.object_id.uuid = [1] * 16

        self.previous = None
        self.velocity = (0.0, 0.0)
        self.acceleration = (0.0, 0.0)

        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE)
        self.detection = self.create_publisher(
            DetectedObjects,
            "object_recognition/detection/objects",
            qos,
        )
        self.tracking = self.create_publisher(
            TrackedObjects,
            "object_recognition/tracking/objects",
            qos,
        )
        self.prediction = self.create_publisher(
            PredictedObjects,
            "object_recognition/objects",
            qos,
        )

        self.create_timer(1.0 / rate, self.publish)
        self.missing_logged = False

        self.get_logger().info(
            f"Соперник берётся из TF {self.output_frame} -> "
            f"{self.target_frame}, {rate} Гц"
        )

    def lookup(self):
        """Получить позу соперника из TF

        :return (Pose, stamp) либо (None, None) если трансформа ещё нет
        """
        try:
            transform = self.buffer.lookup_transform(
                self.output_frame,
                self.target_frame,
                rclpy.time.Time(),
                timeout=RclDuration(seconds=0.1),
            )
        except tf2_ros.TransformException as error:
            if not self.missing_logged:
                self.get_logger().warn(f"Нет трансформа соперника: {error}")
                self.missing_logged = True
            return None, None

        self.missing_logged = False

        pose = Pose()
        pose.position.x = transform.transform.translation.x
        pose.position.y = transform.transform.translation.y
        pose.position.z = transform.transform.translation.z
        pose.orientation = transform.transform.rotation

        return pose, transform.header.stamp

    def update_motion(self, pose: Pose, stamp) -> None:
        """Пересчитать скорость и ускорение по двум последним позам

        :pose текущая поза соперника
        :stamp метка времени трансформа
        """
        now = stamp.sec + stamp.nanosec * 1e-9

        if self.previous is None:
            self.previous = (pose.position.x, pose.position.y, now)
            return

        previous_x, previous_y, previous_time = self.previous
        dt = now - previous_time

        # трансформ может не обновиться между тиками таймера: делить на такой
        # шаг нельзя, а прошлые скорость и ускорение остаются в силе
        if dt < 1e-3:
            return

        vx = (pose.position.x - previous_x) / dt
        vy = (pose.position.y - previous_y) / dt

        self.acceleration = (
            (vx - self.velocity[0]) / dt,
            (vy - self.velocity[1]) / dt,
        )
        self.velocity = (vx, vy)
        self.previous = (pose.position.x, pose.position.y, now)

    def speed_and_heading(self, pose: Pose) -> tuple:
        """Скорость вдоль курса и сам курс движения

        Autoware ждёт twist в системе объекта, то есть продольную скорость.
        У стоящего робота направление из скорости не восстановить, поэтому
        курс берётся из ориентации.

        :pose текущая поза соперника

        :return (скорость, курс)
        """
        vx, vy = self.velocity
        speed = math.hypot(vx, vy)

        if speed < 1e-3:
            return 0.0, yaw_of(pose)

        return speed, math.atan2(vy, vx)

    def predicted_path(self, pose: Pose) -> PredictedPath:
        """Проэкстраполировать движение соперника по прямой

        Заглушка не знает ни намерений соперника, ни стен: путь строится
        равномерным движением с текущей скоростью. Настоящий предиктор
        заменит это моделью поведения.

        :pose текущая поза соперника

        :return предсказанный путь
        """
        path = PredictedPath()
        path.confidence = 1.0
        path.time_step = Duration()
        path.time_step.sec = int(self.step)
        path.time_step.nanosec = int((self.step % 1.0) * 1e9)

        vx, vy = self.velocity
        steps = int(self.horizon / self.step) + 1

        for index in range(steps):
            moment = index * self.step
            step_pose = Pose()
            step_pose.position.x = pose.position.x + vx * moment
            step_pose.position.y = pose.position.y + vy * moment
            step_pose.position.z = pose.position.z
            step_pose.orientation = pose.orientation
            path.path.append(step_pose)

        return path

    def classification(self) -> ObjectClassification:
        """Классификация объекта: соперник опознан достоверно"""
        label = ObjectClassification()
        label.label = ROBOT_LABEL
        label.probability = 1.0
        return label

    def shape(self) -> Shape:
        """Габариты соперника"""
        shape = Shape()
        shape.type = Shape.BOUNDING_BOX
        shape.dimensions.x, shape.dimensions.y, shape.dimensions.z = ROBOT_SIZE
        return shape

    def publish(self) -> None:
        """Опубликовать соперника во все три топика цепочки восприятия"""
        pose, stamp = self.lookup()

        if pose is None:
            return

        self.update_motion(pose, stamp)
        speed, heading = self.speed_and_heading(pose)

        # в detection и tracking ориентация -- это направление движения
        moving_pose = Pose()
        moving_pose.position = pose.position
        moving_pose.orientation = pose.orientation
        yaw_to_quaternion(moving_pose, heading)

        header_stamp = self.get_clock().now().to_msg()

        detected = DetectedObject()
        detected.existence_probability = 1.0
        detected.classification.append(self.classification())
        detected.shape = self.shape()
        detected.kinematics.pose_with_covariance.pose = moving_pose
        detected.kinematics.has_position_covariance = False
        detected.kinematics.orientation_availability = (
            DetectedObjectKinematics.AVAILABLE
        )
        detected.kinematics.twist_with_covariance.twist.linear.x = speed
        detected.kinematics.has_twist = True
        detected.kinematics.has_twist_covariance = False

        detections = DetectedObjects()
        detections.header.stamp = header_stamp
        detections.header.frame_id = self.output_frame
        detections.objects.append(detected)
        self.detection.publish(detections)

        tracked = TrackedObject()
        tracked.object_id = self.object_id
        tracked.existence_probability = 1.0
        tracked.classification.append(self.classification())
        tracked.shape = self.shape()
        tracked.kinematics.pose_with_covariance.pose = moving_pose
        tracked.kinematics.orientation_availability = (
            TrackedObjectKinematics.AVAILABLE
        )
        tracked.kinematics.twist_with_covariance.twist.linear.x = speed
        tracked.kinematics.acceleration_with_covariance.accel.linear.x = (
            math.hypot(*self.acceleration)
        )
        tracked.kinematics.is_stationary = speed < 1e-3

        tracks = TrackedObjects()
        tracks.header.stamp = header_stamp
        tracks.header.frame_id = self.output_frame
        tracks.objects.append(tracked)
        self.tracking.publish(tracks)

        predicted = PredictedObject()
        predicted.object_id = self.object_id
        predicted.existence_probability = 1.0
        predicted.classification.append(self.classification())
        predicted.shape = self.shape()
        predicted.kinematics.initial_pose_with_covariance.pose = moving_pose
        predicted.kinematics.initial_twist_with_covariance.twist.linear.x = (
            speed
        )
        predicted.kinematics.predicted_paths.append(self.predicted_path(pose))

        predictions = PredictedObjects()
        predictions.header.stamp = header_stamp
        predictions.header.frame_id = self.output_frame
        predictions.objects.append(predicted)
        self.prediction.publish(predictions)


def main():
    rclpy.init()
    node = FakeOpponentDetector()
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
