"""Робот против коробок по известным размерам и переключение трекера.

Геометрия -- как на полигоне: Kobuki (корпус R 0.178 до 0.12 м, пластины
R 0.17 на 0.131, 0.187 и 0.397, между средней и верхней только стойки),
коробки 0.15x0.15x0.40 и 0.40x0.60x0.20. Точки -- то, что видит лидар с
одной стороны, z -- высота над полом.
"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src/jr_perception"))
from jr_perception.segmentation import Detection, RobotModel, face_fit, hole_ratio, inspect_cluster  # noqa: E402
from jr_perception.tracker import Tracker, TrackerConfig  # noqa: E402

# как в robot_detector_fastlio.param.yaml
MODEL = RobotModel(
    max_height=0.50, min_top=0.25, min_points=10, min_extent=0.22,
    face_rms=0.025, min_thickness=0.06, max_hole_ratio=0.8, min_rim_points=12,
)
OBSERVER = np.array([0.0, 0.0])
RNG = np.random.default_rng(0)


def visible_arc(center, radius, heights, count):
    """Точки ближней к наблюдателю половины окружности на высотах heights"""
    toward = math.atan2(OBSERVER[1] - center[1], OBSERVER[0] - center[0])
    angles = toward + RNG.uniform(-1.2, 1.2, count)
    z = RNG.choice(heights, count)
    return np.c_[center[0] + radius * np.cos(angles), center[1] + radius * np.sin(angles), z]


def kobuki(center):
    center = np.asarray(center, dtype=float)
    body = visible_arc(center, 0.178, np.linspace(0.03, 0.12, 10), 120)
    plates = visible_arc(center, 0.170, np.array([0.131, 0.187, 0.397, 0.40]), 80)
    # стойки на радиусе 0.145: редкие точки между пластинами
    posts = visible_arc(center, 0.145, np.linspace(0.20, 0.39, 20), 12)
    points = np.vstack([body, plates, posts])
    points[:, :2] += RNG.normal(0.0, 0.008, (len(points), 2))
    return points


def box_faces(corner, sides, height, count=300):
    """Две видимые грани коробки, угол -- ближе к наблюдателю"""
    corner = np.asarray(corner, dtype=float)
    away = corner / np.linalg.norm(corner)
    u = np.array([away[0] - away[1], away[0] + away[1]]) / math.sqrt(2.0)
    v = np.array([away[0] + away[1], -away[0] + away[1]]) / math.sqrt(2.0)
    t = RNG.uniform(0.0, 1.0, count)
    first = t[: count // 2, None] * sides[0] * u
    second = t[count // 2:, None] * sides[1] * v
    xy = corner + np.vstack([first, second]) + RNG.normal(0.0, 0.008, (count, 2))
    return np.c_[xy, RNG.uniform(0.02, height, count)]


def test_kobuki_passes_shape_checks():
    points = kobuki([1.5, 0.3])
    thickness, faces = face_fit(points[:, :2])
    ratio, _ = hole_ratio(points[:, 2], MODEL)
    assert faces > MODEL.face_rms and thickness > MODEL.min_thickness
    assert ratio < MODEL.max_hole_ratio
    detection, reason = inspect_cluster(points, OBSERVER, MODEL)
    assert detection is not None, reason


def test_small_box_is_rejected():
    points = box_faces([1.5, 0.3], (0.15, 0.15), 0.40)
    detection, reason = inspect_cluster(points, OBSERVER, MODEL)
    assert detection is None, reason


def test_two_small_boxes_side_by_side_are_rejected_by_faces():
    # вплотную -- одна грань 0.30, шириной как робот
    points = box_faces([1.5, 0.3], (0.30, 0.15), 0.40)
    detection, reason = inspect_cluster(points, OBSERVER, MODEL)
    assert detection is None
    assert reason.startswith("гран")


def test_large_box_on_its_edge_is_rejected():
    # 0.40x0.60x0.20 на ребре: грани 0.6 и 0.2, высота 0.40
    points = box_faces([1.2, -0.4], (0.60, 0.20), 0.40)
    detection, reason = inspect_cluster(points, OBSERVER, MODEL)
    assert detection is None, reason


def test_solid_object_without_plate_gap_is_rejected():
    # круглый, как робот, но сплошной по высоте (ноги, урна)
    center = np.array([1.5, 0.3])
    points = visible_arc(center, 0.178, np.linspace(0.03, 0.42, 40), 300)
    inside = np.c_[center + RNG.normal(0.0, 0.06, (60, 2)), RNG.uniform(0.03, 0.42, 60)]
    detection, reason = inspect_cluster(np.vstack([points, inside]), OBSERVER, MODEL)
    assert detection is None
    assert reason.startswith("нет пробела")


def test_recorded_wall_fragment_cannot_open_a_track():
    """Реальный фрагмент стены: 6 точек обода ложились на окружность корпуса"""
    import json
    fixture = json.loads((Path(__file__).parent / "fixtures/detector_wall_fragment.json").read_text())
    points = np.array(fixture["points"])
    detection, reason = inspect_cluster(points, np.array(fixture["observer"])[:2], MODEL)
    assert detection is None or not detection.strong, reason


def test_tracker_switches_from_box_to_placed_robot():
    """Трек коробки держится на слабых детекциях; робот поставили рядом"""
    def run(switch_after):
        tracker = Tracker(TrackerConfig(switch_after=switch_after, reacquire_time=3.0))
        box, robot = np.array([1.0, 1.0]), np.array([2.0, 1.5])
        chosen = []
        for step in range(100):
            time = 0.1 * step
            # коробка: одна надёжная детекция открыла трек, дальше только слабые
            detections = [Detection(box, 0.1, strong=step == 0)]
            if time >= 3.0:
                detections.append(Detection(robot, 0.03, strong=True))
            selected = tracker.step(time, detections)
            chosen.append(None if selected is None else selected.mean[:2].copy())
        return chosen

    before = run(0.0)
    assert np.allclose(before[-1], [1.0, 1.0], atol=0.05)  # без правила -- залип на коробке
    after = run(1.0)
    assert np.allclose(after[-1], [2.0, 1.5], atol=0.05)
    switched = next(i for i, c in enumerate(after) if c is not None and np.allclose(c, [2.0, 1.5], atol=0.05))
    assert 0.1 * switched <= 3.0 + 1.5  # робот выдан не позже ~1.5 с после появления
