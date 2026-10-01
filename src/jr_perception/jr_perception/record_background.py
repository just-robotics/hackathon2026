#!/usr/bin/env python3

"""Записать фон для детектора по бэгу с неподвижного лидара.

Облако переводится в frame по /tf_static из того же бэга, по нижним точкам
подгоняется плоскость пола, и ячейки, занятые выше пола в заметной доле
сканов, сохраняются в .npz. Детектор читает его параметром background_file.

Соперник на время записи не должен стоять на месте: лучше всего бэг, где он
ездит, -- его след размазан по многим ячейкам и в фон не попадает.

    ros2 run jr_perception record_background.py <бэг> <фон.npz>
"""

import argparse

import numpy as np
import rosbag2_py

from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage

from jr_perception import background
from jr_perception.robot_detector import cloud_to_xyz, quaternion_matrix


def chain_to(frame: str, target: str, edges: dict) -> tuple:
    """Трансформ из frame в target по статическим TF

    :frame исходный фрейм
    :target целевой фрейм, предок frame
    :edges {child: (parent, rotation, translation)}

    :return (rotation, translation): p_target = rotation @ p + translation
    """
    rotation, translation = np.eye(3), np.zeros(3)
    while frame != target:
        if frame not in edges:
            raise SystemExit(f"В /tf_static бэга нет пути от {frame} до {target}")
        parent, step_rotation, step_translation = edges[frame]
        rotation = step_rotation @ rotation
        translation = step_rotation @ translation + step_translation
        frame = parent
    return rotation, translation


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("bag", help="каталог бэга")
    parser.add_argument("output", help="куда сохранить фон, .npz")
    parser.add_argument("--topic", default="/livox/lidar")
    parser.add_argument("--frame", default="base_link", help="фрейм фона")
    parser.add_argument("--cell", type=float, default=0.05, help="шаг сетки, м")
    parser.add_argument(
        "--share",
        type=float,
        default=0.10,
        help="доля сканов, в которой ячейка должна быть занята",
    )
    parser.add_argument(
        "--min-height", type=float, default=0.03, help="точки ниже -- пол, м"
    )
    arguments = parser.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=arguments.bag, storage_id=""),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=[arguments.topic, "/tf_static"]))

    edges = {}
    clouds = []
    cloud_frame = None
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == "/tf_static":
            for transform in deserialize_message(data, TFMessage).transforms:
                t = transform.transform.translation
                r = transform.transform.rotation
                edges[transform.child_frame_id] = (
                    transform.header.frame_id,
                    quaternion_matrix([r.x, r.y, r.z, r.w]),
                    np.array([t.x, t.y, t.z]),
                )
        else:
            message = deserialize_message(data, PointCloud2)
            cloud_frame = message.header.frame_id
            points = cloud_to_xyz(message)
            clouds.append(points[np.linalg.norm(points, axis=1) > 0.3])

    if not clouds:
        raise SystemExit(f"В бэге нет {arguments.topic}")

    rotation, translation = chain_to(cloud_frame, arguments.frame, edges)
    scans = [points @ rotation.T + translation for points in clouds]

    plane, floor_std = background.fit_floor(np.concatenate(scans[:50]))
    keys = background.occupied_cells(
        scans, arguments.cell, arguments.share, arguments.min_height, plane
    )
    background.save(arguments.output, keys, arguments.cell, plane, arguments.frame)

    a, b, c = plane
    print(
        f"{len(scans)} сканов из {arguments.topic} ({cloud_frame} -> {arguments.frame}).\n"
        f"Пол: z = {a:.4f} x + {b:.4f} y + {c:.3f}, СКО {100 * floor_std:.1f} см, "
        f"лидар над полом {translation[2] - c:.3f} м.\n"
        f"Фон: {len(keys)} ячеек по {arguments.cell} м -> {arguments.output}"
    )


if __name__ == "__main__":
    main()
