"""Трекинг соперника по детекциям его центра.

Лидар видит Kobuki телом вращения, поэтому детектор отдаёт только центр.
Курс восстанавливается из движения: у дифф-привода скорость всегда
направлена вдоль корпуса. Каждый трек -- EKF с моделью дифф-привода,
состояние [x, y, theta, v, omega].

Треков несколько: обрывок коробки, похожий на дугу, тоже может породить
трек. Коробки неподвижны, поэтому из подтверждённых треков соперником
считается тот, что хоть раз заметно сдвинулся, а до первого движения -- тот,
у кого больше надёжных детекций.
"""

import math
from collections import deque
from dataclasses import dataclass

import numpy as np


X, Y, THETA, V, OMEGA = range(5)


def wrap(angle: float) -> float:
    """Привести угол к [-pi, pi)"""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


@dataclass
class TrackerConfig:
    """Параметры трекера"""

    # шум модели: продольное и угловое ускорение
    accel_std: float = 0.5
    yaw_accel_std: float = 3.0
    # увод позиции, который модель не объясняет (проскальзывание, толчки),
    # м/sqrt(с)
    position_std: float = 0.02
    # Постоянная затухания omega, с. Пока робот стоит, omega не наблюдаема,
    # и без затухания курс продолжал бы вращаться с последней скоростью.
    omega_decay: float = 1.0
    # Пока курс не известен, скорость не оценивается, а позиция ведётся как
    # случайное блуждание с этим темпом, м/sqrt(с).
    wander_std: float = 0.3
    # порог квадрата расстояния Махаланобиса: chi2 с 2 степенями, 99%
    gate: float = 9.21
    # столько детекций нужно треку, чтобы считаться подтверждённым
    confirm_hits: int = 3
    # сколько трек живёт без детекций: неподтверждённый и подтверждённый, с
    tentative_coast: float = 0.35
    max_coast: float = 1.5
    max_tracks: int = 8
    # Курс берётся по перемещению центра, когда за окно seed_window центр
    # ушёл дальше seed_distance, а фильтр видит меньше половины этой
    # скорости: на старте и после разворота на месте.
    seed_distance: float = 0.12
    seed_window: float = 1.0
    seed_heading_std: float = 0.35
    # Робот считается едущим вперёд: заметная отрицательная скорость значит,
    # что курс развёрнут на pi.
    v_flip: float = 0.05
    # трек, сдвинувшийся от места рождения дальше этого, -- точно не коробка
    move_threshold: float = 0.3


class Track:
    """Один трек: EKF и счётчики для подтверждения и выбора"""

    def __init__(self, time: float, detection, config: TrackerConfig):
        self.config = config

        # Пока курс не известен, theta, v и omega не связаны с позицией:
        # нулевые перекрёстные ковариации оставляют их на месте при
        # обновлении, а predict их не трогает.
        self.state = np.array([*detection.center, 0.0, 0.0, 0.0])
        self.cov = np.diag(
            [detection.sigma ** 2, detection.sigma ** 2, math.pi ** 2, 0.0, 0.0]
        )
        self.heading_known = False

        self.time = time
        self.last_update = time
        self.hits = 1
        self.strong_hits = int(detection.strong)
        self.origin = self.state[:2].copy()
        self.travel = 0.0
        self.history = deque()
        self._remember(time, detection)

    @property
    def confirmed(self) -> bool:
        return self.hits >= self.config.confirm_hits

    @property
    def moved(self) -> bool:
        return self.travel >= self.config.move_threshold

    def predict(self, time: float):
        """Продвинуть трек к моменту time

        :time момент, с
        """
        dt = time - self.time
        if dt <= 0.0:
            return

        self.time = time
        config = self.config

        if not self.heading_known:
            drift = config.wander_std ** 2 * dt
            self.cov[X, X] += drift
            self.cov[Y, Y] += drift
            return

        x, y, theta, v, omega = self.state
        cos, sin = math.cos(theta), math.sin(theta)
        decay = math.exp(-dt / config.omega_decay)

        self.state = np.array(
            [
                x + v * cos * dt,
                y + v * sin * dt,
                wrap(theta + omega * dt),
                v,
                omega * decay,
            ]
        )

        jacobian = np.eye(5)
        jacobian[X, THETA] = -v * sin * dt
        jacobian[X, V] = cos * dt
        jacobian[Y, THETA] = v * cos * dt
        jacobian[Y, V] = sin * dt
        jacobian[THETA, OMEGA] = dt
        jacobian[OMEGA, OMEGA] = decay

        # ускорения как белый шум: продольное проходит в v и позицию вдоль
        # курса, угловое -- в omega и theta
        noise_input = np.array(
            [
                [0.5 * cos * dt ** 2, 0.0],
                [0.5 * sin * dt ** 2, 0.0],
                [0.0, 0.5 * dt ** 2],
                [dt, 0.0],
                [0.0, dt],
            ]
        )
        noise = noise_input @ np.diag(
            [config.accel_std ** 2, config.yaw_accel_std ** 2]
        ) @ noise_input.T
        noise[X, X] += config.position_std ** 2 * dt
        noise[Y, Y] += config.position_std ** 2 * dt

        self.cov = jacobian @ self.cov @ jacobian.T + noise

        # Пока робот стоит, дисперсия курса растёт без предела. Больше pi^2
        # она ничего не значит, а линеаризация с ней при первом же движении
        # дёргала бы курс скачками.
        if self.cov[THETA, THETA] > math.pi ** 2:
            scale = math.pi / math.sqrt(self.cov[THETA, THETA])
            self.cov[THETA, :] *= scale
            self.cov[:, THETA] *= scale

    def distance(self, detection) -> float:
        """Квадрат расстояния Махаланобиса от прогноза до детекции"""
        innovation = detection.center - self.state[:2]
        covariance = self.cov[:2, :2] + np.eye(2) * detection.sigma ** 2
        return float(innovation @ np.linalg.solve(covariance, innovation))

    def update(self, time: float, detection):
        """Учесть детекцию центра

        :time момент детекции, с
        :detection segmentation.Detection
        """
        noise = np.eye(2) * detection.sigma ** 2
        innovation = detection.center - self.state[:2]
        covariance = self.cov[:2, :2] + noise
        gain = self.cov[:, :2] @ np.linalg.inv(covariance)

        self.state = self.state + gain @ innovation
        self.state[THETA] = wrap(self.state[THETA])

        # форма Джозефа: ковариация остаётся симметричной и положительной
        correction = np.eye(5)
        correction[:, :2] -= gain
        self.cov = correction @ self.cov @ correction.T + gain @ noise @ gain.T

        if self.heading_known and self.state[V] < -self.config.v_flip:
            self._flip()

        self._remember(time, detection)
        self._seed_heading()

        self.hits += 1
        self.strong_hits += int(detection.strong)
        self.last_update = time
        self.travel = max(
            self.travel, float(np.linalg.norm(self.state[:2] - self.origin))
        )

    def _flip(self):
        """Развернуть курс на pi со сменой знака скорости

        Движение при этом не меняется: (theta + pi, -v) описывает ту же
        траекторию, что и (theta, v).
        """
        self.state[THETA] = wrap(self.state[THETA] + math.pi)
        self.state[V] = -self.state[V]
        self.cov[V, :] *= -1.0
        self.cov[:, V] *= -1.0

    def _remember(self, time: float, detection):
        """Запомнить надёжную детекцию для оценки курса по перемещению"""
        if detection.strong:
            self.history.append((time, np.array(detection.center, dtype=float)))

        while self.history and time - self.history[0][0] > self.config.seed_window:
            self.history.popleft()

    def _seed_heading(self):
        """Взять курс по перемещению центра, если фильтр его не видит

        В EKF курс поправляется только через скорость: при v около нуля
        якобиан позиции по theta нулевой, и тронувшийся вбок робот фильтр
        курсом не догонит. Поэтому на старте и после остановки курс задаётся
        прямо по перемещению за последнее окно.
        """
        if len(self.history) < 2:
            return

        (t0, p0), (t1, p1) = self.history[0], self.history[-1]
        shift = p1 - p0
        distance = float(np.linalg.norm(shift))
        span = t1 - t0
        if distance < self.config.seed_distance or span <= 0.0:
            return

        speed = distance / span
        if self.heading_known and abs(self.state[V]) >= 0.5 * speed:
            return

        self.state[THETA] = math.atan2(shift[1], shift[0])
        self.state[V] = speed
        self.state[OMEGA] = 0.0
        self.cov[THETA:, :] = 0.0
        self.cov[:, THETA:] = 0.0
        self.cov[THETA, THETA] = self.config.seed_heading_std ** 2
        self.cov[V, V] = (0.5 * speed) ** 2
        self.cov[OMEGA, OMEGA] = 1.0
        self.heading_known = True


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
        выбранный ни разу не двигался: коробки не ездят.
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
