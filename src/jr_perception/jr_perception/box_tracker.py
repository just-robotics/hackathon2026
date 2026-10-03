"""Трекинг коробок на сцене, чтобы не путать их с роботом.

Коробка -- кластер, точки которого лежат на вертикальных гранях
(segmentation.plane_test). Такие кластеры по скану связываются во времени в
треки неподвижных предметов. Трек подтверждается, набрав confirm_hits
наблюдений «на гранях», и тогда детекции робота внутри него до трекера
робота не доходят: по одному скану угол коробки можно принять за дугу
корпуса, а трек коробки помнит, что здесь стоит предмет.

Чтобы стоящий робот не стал коробкой: сильные детекции робота внутри трека
засчитываются против него, и трек подтверждается, только пока наблюдений
«на гранях» втрое больше. Трек, сдвинувшийся дальше max_travel, -- не
коробка, и он удаляется.

Тип коробки -- по известным размерам (параметр sizes): ближайший по высоте
верха, если видимый габарит в него помещается.
"""

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class BoxTrackerConfig:
    """Параметры трекера коробок"""

    # наблюдение дальше этого от центра трека -- другой предмет, м
    gate: float = 0.25
    # столько наблюдений «на гранях» нужно, чтобы трек стал коробкой
    confirm_hits: int = 5
    # сколько живёт трек без наблюдений: неподтверждённый и подтверждённый,
    # с. Коробки неподвижны и надолго закрываются роботами, поэтому второе
    # большое.
    tentative_age: float = 1.0
    max_age: float = 10.0
    # доля нового наблюдения в центре и курсе трека
    smoothing: float = 0.2
    # центр ушёл от места рождения дальше этого -- это не коробка, м
    max_travel: float = 0.3
    # детекция робота не дальше этого от граней коробки -- внутри неё, м
    contain_margin: float = 0.05
    # допуск на габарит и высоту при выборе типа, м
    size_tolerance: float = 0.08


def wrap_half(angle: float) -> float:
    """Привести угол к [-pi/2, pi/2): у прямоугольника курс по модулю pi"""
    return (angle + math.pi / 2.0) % math.pi - math.pi / 2.0


class BoxTrack:
    """Один трек коробки"""

    def __init__(self, identifier: int, time: float, box):
        self.id = identifier
        self.center = np.array(box.center, dtype=float)
        self.birth = self.center.copy()
        self.yaw = float(box.yaw)
        self.size = np.array(box.size, dtype=float)
        self.top = float(box.top)
        self.first_seen = time
        self.last_seen = time
        self.hits = 1
        self.planar_hits = int(box.planar)
        self.robot_hits = 0
        self.kind = ""

    def update(self, time: float, box, smoothing: float):
        self.center += smoothing * (np.asarray(box.center) - self.center)
        self.yaw = wrap_half(self.yaw + smoothing * wrap_half(box.yaw - self.yaw))
        # видна обычно часть граней: габарит -- наибольший из увиденного
        self.size = np.maximum(self.size, box.size)
        self.top = max(self.top, float(box.top))
        self.last_seen = time
        self.hits += 1
        self.planar_hits += int(box.planar)

    def confirmed(self, config: BoxTrackerConfig) -> bool:
        return (
            self.planar_hits >= config.confirm_hits
            and self.planar_hits > 3 * self.robot_hits
        )

    def contains(self, point: np.ndarray, margin: float) -> bool:
        """Точка внутри прямоугольника трека, расширенного на margin"""
        offset = np.asarray(point) - self.center
        cos, sin = math.cos(self.yaw), math.sin(self.yaw)
        along = offset[0] * cos + offset[1] * sin
        across = -offset[0] * sin + offset[1] * cos
        return (
            abs(along) <= self.size[0] / 2.0 + margin
            and abs(across) <= self.size[1] / 2.0 + margin
        )


class BoxTracker:
    """Треки коробок и проверка детекций робота против них"""

    def __init__(self, config: BoxTrackerConfig, sizes: list):
        """
        :config параметры
        :sizes известные коробки, плоский список длина, ширина, высота подряд
        """
        self.config = config
        self.sizes = [
            (max(sizes[i], sizes[i + 1]), min(sizes[i], sizes[i + 1]), sizes[i + 2])
            for i in range(0, len(sizes) - 2, 3)
        ]
        self.tracks = []
        self.time = None
        self.next_id = 1

    def step(self, time: float, boxes: list, detections: list) -> list:
        """Учесть предметы скана и детекции робота

        :time момент скана, с
        :boxes список segmentation.Box
        :detections детекции робота этого скана (segmentation.Detection)

        :return детекции, которые не внутри подтверждённой коробки
        """
        config = self.config
        if self.time is not None and time < self.time:
            # время пошло назад: бэг по кругу
            self.tracks = []
        self.time = time

        planar = [box for box in boxes if box.planar]
        pairs = sorted(
            (float(np.linalg.norm(track.center - box.center)), i, j)
            for i, track in enumerate(self.tracks)
            for j, box in enumerate(planar)
        )
        used_tracks, used_boxes = set(), set()
        for distance, i, j in pairs:
            if distance > config.gate:
                break
            if i in used_tracks or j in used_boxes:
                continue
            self.tracks[i].update(time, planar[j], config.smoothing)
            used_tracks.add(i)
            used_boxes.add(j)
        for j, box in enumerate(planar):
            if j not in used_boxes:
                self.tracks.append(BoxTrack(self.next_id, time, box))
                self.next_id += 1

        # сильная детекция робота внутри трека -- голос против коробки
        for detection in detections:
            if detection.strong:
                for track in self.tracks:
                    if track.contains(detection.center, config.contain_margin):
                        track.robot_hits += 1

        self.tracks = [
            track
            for track in self.tracks
            if time - track.last_seen
            <= (config.max_age if track.confirmed(config) else config.tentative_age)
            and np.linalg.norm(track.center - track.birth) <= config.max_travel
        ]
        for track in self.tracks:
            track.kind = self.kind_of(track)

        boxes_now = [track for track in self.tracks if track.confirmed(config)]
        return [
            detection
            for detection in detections
            if not any(
                track.contains(detection.center, config.contain_margin) for track in boxes_now
            )
        ]

    def kind_of(self, track: BoxTrack) -> str:
        """Тип по известным размерам: ближайший по высоте, куда влезает габарит"""
        tolerance = self.config.size_tolerance
        best, best_error = "", tolerance
        for index, (length, width, height) in enumerate(self.sizes):
            fits = track.size[0] <= length + tolerance and track.size[1] <= width + tolerance
            error = abs(track.top - height)
            if fits and error <= best_error:
                best, best_error = f"{length:.2f}x{width:.2f}x{height:.2f}", error
        return best

    def confirmed(self) -> list:
        return [track for track in self.tracks if track.confirmed(self.config)]
