"""Трекинг соперника по детекциям его центра.

Лидар видит Kobuki телом вращения, поэтому детектор отдаёт только центр.
Каждый трек ведёт позицию и скорость линейным фильтром Калмана с моделью
постоянной скорости: состояние [x, y, vx, vy], ускорение -- белый шум. У
стоящего робота такой фильтр усредняет центр по многим сканам, у едущего --
следует за ним без запаздывания, пока он не ускоряется резко.

Курс берётся из вектора скорости: у дифф-привода скорость направлена вдоль
корпуса. Пока робот стоит или крутится на месте, курс держится последний, а
его дисперсия растёт. Снаружи трек отдаёт [x, y, theta, v, omega] и
ковариацию в том же порядке.

Треков несколько: обрывок посторонней дуги тоже может породить трек.
Мебель не ездит, поэтому из подтверждённых треков соперником считается тот,
что заметно сдвинулся, а до первого движения -- тот, у кого больше надёжных
детекций.
"""

import math
from dataclasses import dataclass

import numpy as np


# порядок величин в Track.state и Track.cov
X, Y, THETA, V, OMEGA = range(5)


def wrap(angle: float) -> float:
    """Привести угол к [-pi, pi)"""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


@dataclass
class TrackerConfig:
    """Параметры трекера"""

    # Шум модели -- ускорение, м/с^2. Меньше -- сильнее сглаживание у
    # стоящего робота, но дольше догонять резкий старт и торможение.
    accel_std: float = 0.5
    # увод позиции, который модель не объясняет (толчки), м/sqrt(с)
    position_std: float = 0.01
    # неопределённость скорости нового трека, м/с
    initial_speed_std: float = 0.5
    # Курс берётся из скорости, когда она больше heading_speed и больше двух
    # своих СКО: иначе направление скорости -- это направление шума.
    heading_speed: float = 0.05
    # постоянная времени сглаживания omega, с
    omega_time: float = 0.5
    # порог квадрата расстояния Махаланобиса: chi2 с 2 степенями, 99%
    gate: float = 9.21
    # столько детекций нужно треку, чтобы считаться подтверждённым
    confirm_hits: int = 3
    # сколько трек живёт без детекций: неподтверждённый и подтверждённый, с
    tentative_coast: float = 0.35
    max_coast: float = 1.5
    max_tracks: int = 8
    # трек, сдвинувшийся от места рождения дальше этого, -- точно не мебель
    move_threshold: float = 0.3
    # Но сдвиг засчитывается только треку, набравшему столько надёжных
    # детекций: три детекции круглого предмета с разбросом 0.3 м иначе
    # «ехали» и уводили выбор от стоящего робота (бэг rot_1m).
    move_min_hits: int = 10


class Track:
    """Один трек: фильтр Калмана и счётчики для подтверждения и выбора"""

    def __init__(self, time: float, detection, config: TrackerConfig):
        self.config = config

        # фильтр: [x, y, vx, vy]
        self.mean = np.array([*detection.center, 0.0, 0.0])
        self.covariance = np.diag(
            [detection.sigma ** 2] * 2 + [config.initial_speed_std ** 2] * 2
        )

        self.heading = 0.0
        self.heading_known = False
        self.heading_time = time
        self.omega = 0.0

        self.time = time
        self.last_update = time
        self.hits = 1
        self.strong_hits = int(detection.strong)
        # Сдвиг считается только по надёжным детекциям: центр по центроиду
        # гуляет на ±10 см, и за минуту неподвижный предмет набрал бы
        # «движение» из одного шума.
        self.origin = None
        self.travel = 0.0
        self._measure_travel(detection)

    @property
    def confirmed(self) -> bool:
        return self.hits >= self.config.confirm_hits

    @property
    def moved(self) -> bool:
        return (
            self.strong_hits >= self.config.move_min_hits
            and self.travel >= self.config.move_threshold
        )

    @property
    def speed(self) -> float:
        return float(np.linalg.norm(self.mean[2:]))

    @property
    def state(self) -> np.ndarray:
        """[x, y, theta, v, omega]"""
        return np.array(
            [self.mean[0], self.mean[1], self.heading, self.speed, self.omega]
        )

    @property
    def cov(self) -> np.ndarray:
        """Ковариация state, 5x5

        Дисперсия курса -- поперечная к скорости дисперсия скорости, делённая
        на квадрат скорости: у медленного робота направление движения почти
        не определено. Больше pi^2 она не бывает.
        """
        cov = np.zeros((5, 5))
        cov[:2, :2] = self.covariance[:2, :2]

        velocity_cov = self.covariance[2:, 2:]
        along = np.array([math.cos(self.heading), math.sin(self.heading)])
        across = np.array([-along[1], along[0]])
        speed = self.speed

        if self.heading_known:
            cov[THETA, THETA] = min(
                float(across @ velocity_cov @ across) / max(speed ** 2, 1e-9),
                math.pi ** 2,
            )
        else:
            cov[THETA, THETA] = math.pi ** 2

        cov[V, V] = float(along @ velocity_cov @ along)
        cov[OMEGA, OMEGA] = 0.25 if self.heading_known else 1.0
        return cov

    def predict(self, time: float):
        """Продвинуть трек к моменту time

        :time момент, с
        """
        dt = time - self.time
        if dt <= 0.0:
            return
        self.time = time

        transition = np.eye(4)
        transition[0, 2] = transition[1, 3] = dt

        # ускорение как белый шум: дискретная модель постоянной скорости
        a = self.config.accel_std ** 2
        p = self.config.position_std ** 2 * dt
        noise = np.zeros((4, 4))
        for i in (0, 1):
            noise[i, i] = a * dt ** 4 / 4.0 + p
            noise[i, i + 2] = noise[i + 2, i] = a * dt ** 3 / 2.0
            noise[i + 2, i + 2] = a * dt ** 2

        self.mean = transition @ self.mean
        self.covariance = transition @ self.covariance @ transition.T + noise

        if not self._moving():
            # курс не обновляется, и его скорость поворота сходит на нет
            self.omega *= math.exp(-dt / self.config.omega_time)

    def distance(self, detection) -> float:
        """Квадрат расстояния Махаланобиса от прогноза до детекции"""
        innovation = detection.center - self.mean[:2]
        covariance = self.covariance[:2, :2] + np.eye(2) * detection.sigma ** 2
        return float(innovation @ np.linalg.solve(covariance, innovation))

    def update(self, time: float, detection):
        """Учесть детекцию центра

        :time момент детекции, с
        :detection segmentation.Detection
        """
        noise = np.eye(2) * detection.sigma ** 2
        innovation = detection.center - self.mean[:2]
        covariance = self.covariance[:2, :2] + noise
        gain = self.covariance[:, :2] @ np.linalg.inv(covariance)

        self.mean = self.mean + gain @ innovation
        # форма Джозефа: ковариация остаётся симметричной и положительной
        correction = np.eye(4)
        correction[:, :2] -= gain
        self.covariance = (
            correction @ self.covariance @ correction.T + gain @ noise @ gain.T
        )

        self._update_heading(time)

        self.hits += 1
        self.strong_hits += int(detection.strong)
        self.last_update = time
        self._measure_travel(detection)

    def _moving(self) -> bool:
        """Скорость достоверно отлична от нуля"""
        speed = self.speed
        if speed < self.config.heading_speed:
            return False
        along = self.mean[2:] / speed
        speed_std = math.sqrt(max(float(along @ self.covariance[2:, 2:] @ along), 0.0))
        return speed > 2.0 * speed_std

    def _update_heading(self, time: float):
        """Курс по направлению скорости, omega -- по его изменению

        Робот считается едущим вперёд: задним ходом курс развернётся на pi.
        """
        if not self._moving():
            return

        heading = math.atan2(self.mean[3], self.mean[2])
        dt = time - self.heading_time
        if self.heading_known and dt > 0.0:
            rate = wrap(heading - self.heading) / dt
            blend = dt / (self.config.omega_time + dt)
            self.omega += blend * (rate - self.omega)

        self.heading = heading
        self.heading_known = True
        self.heading_time = time

    def _measure_travel(self, detection):
        """Обновить наибольший сдвиг от первой надёжной детекции"""
        if not detection.strong:
            return

        center = np.array(detection.center, dtype=float)
        if self.origin is None:
            self.origin = center
            return

        self.travel = max(self.travel, float(np.linalg.norm(center - self.origin)))


class Tracker:
    """Набор треков и выбор среди них соперника"""

    def __init__(self, config: TrackerConfig):
        self.config = config
        self.tracks = []
        self.selected = None
        self.time = None

    def step(self, time: float, detections: list):
        """Продвинуть треки к моменту скана и учесть его детекции

        :time момент скана, с
        :detections список segmentation.Detection

        :return трек соперника или None
        """
        if self.time is not None and time < self.time:
            # время пошло назад: симулятор перезапущен
            self.tracks = []
            self.selected = None
        self.time = time

        for track in self.tracks:
            track.predict(time)

        # Жадное сопоставление: пары трек-детекция по возрастанию расстояния
        # Махаланобиса, каждый трек и каждая детекция используются один раз.
        pairs = sorted(
            (track.distance(detection), i, j)
            for i, track in enumerate(self.tracks)
            for j, detection in enumerate(detections)
        )
        used_tracks, used_detections = set(), set()
        for distance, i, j in pairs:
            if distance > self.config.gate:
                break
            if i in used_tracks or j in used_detections:
                continue
            self.tracks[i].update(time, detections[j])
            used_tracks.add(i)
            used_detections.add(j)

        self.tracks = [
            track
            for track in self.tracks
            if time - track.last_update
            <= (
                self.config.max_coast
                if track.confirmed
                else self.config.tentative_coast
            )
        ]

        # новые треки открываются только надёжными детекциями
        for j, detection in enumerate(detections):
            if j in used_detections or not detection.strong:
                continue
            if len(self.tracks) >= self.config.max_tracks:
                break
            self.tracks.append(Track(time, detection, self.config))

        return self._select()

    def _select(self):
        """Выбрать среди подтверждённых треков соперника

        Выбранный трек держится, пока жив, -- чтобы оценка не прыгала между
        треками. Сменить его может только сдвинувшийся трек, если сам
        выбранный ни разу не двигался: мебель не ездит.
        """
        confirmed = [track for track in self.tracks if track.confirmed]
        if not confirmed:
            self.selected = None
            return None

        moving = [track for track in confirmed if track.moved]
        current = self.selected if self.selected in confirmed else None
        if current is not None and (current.moved or not moving):
            return current

        self.selected = max(
            moving or confirmed,
            key=lambda track: (track.strong_hits / track.hits, track.hits),
        )
        return self.selected
