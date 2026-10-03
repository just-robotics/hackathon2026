#!/usr/bin/env python3
"""FAST-LIO2 -> base_footprint in lio_odom; TF lio_odom -> wheel odom.

AMCL owns map -> lio_odom. hsl_real/map_kinematic_state publishes the
map-corrected /localization/kinematic_state; this bridge is local odometry.

FAST-LIO2 estimates motion in camera_init (initial LiDAR IMU body).
The fixed extrinsics and robot.start anchor convert this to local lio_odom.
AMCL supplies the independent static-map correction above this frame.
The bridge never broadcasts map -> odom in the real FAST-LIO mode.
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


def matrix(translation, quaternion) -> np.ndarray:
    """4x4 по переносу (x, y, z) и кватерниону (x, y, z, w)"""
    x, y, z, w = quaternion
    t = np.eye(4)
    t[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    t[:3, 3] = translation
    return t


def quaternion(rotation: np.ndarray) -> tuple:
    """Кватернион (x, y, z, w) по матрице поворота 3x3"""
    trace = np.trace(rotation)
    if trace > 0.0:
        s = 2.0 * math.sqrt(trace + 1.0)
        return (
            (rotation[2, 1] - rotation[1, 2]) / s,
            (rotation[0, 2] - rotation[2, 0]) / s,
            (rotation[1, 0] - rotation[0, 1]) / s,
            0.25 * s,
        )
    i = int(np.argmax(np.diag(rotation)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2.0 * math.sqrt(1.0 + rotation[i, i] - rotation[j, j] - rotation[k, k])
    q = [0.0, 0.0, 0.0, 0.0]
    q[i] = 0.25 * s
    q[j] = (rotation[j, i] + rotation[i, j]) / s
    q[k] = (rotation[k, i] + rotation[i, k]) / s
    q[3] = (rotation[k, j] - rotation[j, k]) / s
    return tuple(q)


def rotation_vector(rotation: np.ndarray) -> np.ndarray:
    """Ось * угол по матрице поворота"""
    angle = math.acos(max(-1.0, min(1.0, (np.trace(rotation) - 1.0) / 2.0)))
    if angle < 1e-9:
        return np.zeros(3)
    axis = np.array(
        [
            rotation[2, 1] - rotation[1, 2],
            rotation[0, 2] - rotation[2, 0],
            rotation[1, 0] - rotation[0, 1],
        ]
    ) / (2.0 * math.sin(angle))
    return axis * angle


def stamp_seconds(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


class FastLioBridge(Node):
    def __init__(self):
        super().__init__("fastlio_bridge")
        source = self.declare_parameter("fastlio_topic", "/Odometry").value
        target = self.declare_parameter("odometry_topic", "/localization/lio_odometry").value
        self.map_frame = self.declare_parameter("map_frame", "lio_odom").value
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.base_frame = self.declare_parameter("base_frame", "base_footprint").value
        self.lidar_frame = self.declare_parameter("lidar_frame", "livox").value
        # положение лидара в IMU, как extrinsic_T в конфиге FAST-LIO2
        self.extrinsic = np.array(
            self.declare_parameter("extrinsic_t", [-0.011, -0.02329, 0.04412]).value
        )
        # тип не фиксирован: x:=1 из командной строки -- целое, и узел не падал бы
        start = [
            float(self.declare_parameter(name, 0.0, ParameterDescriptor(dynamic_typing=True)).value)
            for name in ("x", "y", "yaw")
        ]
        self.publish_tf = self.declare_parameter("publish_tf", True).value
        # TF map -> odom датируется вперёд на столько, как transform_tolerance AMCL
        self.tf_tolerance = self.declare_parameter("tf_tolerance", 0.1).value
        self.max_wheel_tf_age = self.declare_parameter("max_wheel_tf_age", 0.1).value

        x, y, yaw = start
        self.map_from_start = matrix((x, y, 0.0), (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)))
        self.map_from_ci = None
        self.body_from_base = None
        self.last = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.publisher = self.create_publisher(Odometry, target, 50)
        self.create_subscription(Odometry, source, self.on_odometry, 50)
        self.get_logger().info(
            f"{source} -> {target} ({self.map_frame} -> {self.base_frame}), "
            f"старт ({x:.3f}, {y:.3f}, {math.degrees(yaw):.1f} град)"
            + (f", TF {self.map_frame} -> {self.odom_frame}" if self.publish_tf else "")
        )

    def calibrate(self) -> bool:
        """body -> base по TF лидара на базе и смещению лидара в IMU"""
        try:
            t = self.tf_buffer.lookup_transform(self.base_frame, self.lidar_frame, Time())
        except TransformException as error:
            self.get_logger().warning(
                f"Нет TF {self.base_frame} -> {self.lidar_frame}: {error}",
                throttle_duration_sec=5.0,
            )
            return False
        tr, q = t.transform.translation, t.transform.rotation
        base_from_lidar = matrix((tr.x, tr.y, tr.z), (q.x, q.y, q.z, q.w))
        body_from_lidar = np.eye(4)
        body_from_lidar[:3, 3] = self.extrinsic
        self.body_from_base = body_from_lidar @ np.linalg.inv(base_from_lidar)
        # camera_init -- body на старте, а база на старте стоит в map_from_start
        self.map_from_ci = self.map_from_start @ np.linalg.inv(self.body_from_base)
        return True

    def on_odometry(self, message: Odometry):
        if self.map_from_ci is None and not self.calibrate():
            return

        p, q = message.pose.pose.position, message.pose.pose.orientation
        ci_from_body = matrix((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))
        map_from_base = self.map_from_ci @ ci_from_body @ self.body_from_base
        moment = stamp_seconds(message.header.stamp)

        out = Odometry()
        out.header.stamp = message.header.stamp
        out.header.frame_id = self.map_frame
        out.child_frame_id = self.base_frame
        position = map_from_base[:3, 3]
        out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = (
            float(v) for v in position
        )
        (
            out.pose.pose.orientation.x,
            out.pose.pose.orientation.y,
            out.pose.pose.orientation.z,
            out.pose.pose.orientation.w,
        ) = (float(v) for v in quaternion(map_from_base[:3, :3]))
        out.pose.covariance[0] = out.pose.covariance[7] = out.pose.covariance[14] = 1e-4
        out.pose.covariance[21] = out.pose.covariance[28] = out.pose.covariance[35] = 1e-4

        # скорости в base_link по разности с прошлой позой
        if self.last is not None and moment > self.last[0]:
            dt = moment - self.last[0]
            previous = self.last[1]
            linear = previous[:3, :3].T @ (position - previous[:3, 3]) / dt
            angular = rotation_vector(previous[:3, :3].T @ map_from_base[:3, :3]) / dt
            t = out.twist.twist
            t.linear.x, t.linear.y, t.linear.z = (float(v) for v in linear)
            t.angular.x, t.angular.y, t.angular.z = (float(v) for v in angular)
        self.last = (moment, map_from_base)
        self.publisher.publish(out)

        if self.publish_tf:
            self.publish_map_to_odom(message.header.stamp, map_from_base)

    def publish_map_to_odom(self, stamp, map_from_base: np.ndarray):
        # Поза FAST-LIO2 датирована концом скана и бывает на 10-20 мс новее
        # последней колёсной одометрии: тогда берётся последний TF -- за это
        # время робот сдвигается на миллиметры. Без ожидания: обработчик,
        # ждущий TF, отставал от поз, и детектор не дожидался позы на скан.
        try:
            t = self.tf_buffer.lookup_transform(self.odom_frame, self.base_frame, Time.from_msg(stamp))
        except TransformException:
            try:
                t = self.tf_buffer.lookup_transform(self.odom_frame, self.base_frame, Time())
            except TransformException as error:
                self.get_logger().warning(
                    f"Нет TF {self.odom_frame} -> {self.base_frame}: {error}",
                    throttle_duration_sec=5.0,
                )
                return
        if abs(stamp_seconds(stamp)-stamp_seconds(t.header.stamp)) > self.max_wheel_tf_age:
            self.get_logger().warning("Wheel TF too old for FAST-LIO bridge", throttle_duration_sec=5.0)
            return
        tr, q = t.transform.translation, t.transform.rotation
        odom_from_base = matrix((tr.x, tr.y, tr.z), (q.x, q.y, q.z, q.w))
        map_from_odom = map_from_base @ np.linalg.inv(odom_from_base)

        out = TransformStamped()
        out.header.stamp = (Time.from_msg(stamp) + Duration(seconds=self.tf_tolerance)).to_msg()
        out.header.frame_id = self.map_frame
        out.child_frame_id = self.odom_frame
        out.transform.translation.x, out.transform.translation.y, out.transform.translation.z = (
            float(v) for v in map_from_odom[:3, 3]
        )
        (
            out.transform.rotation.x,
            out.transform.rotation.y,
            out.transform.rotation.z,
            out.transform.rotation.w,
        ) = (float(v) for v in quaternion(map_from_odom[:3, :3]))
        self.tf_broadcaster.sendTransform(out)


def main():
    rclpy.init()
    node = FastLioBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
