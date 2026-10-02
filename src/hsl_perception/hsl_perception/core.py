"""Adapter around feature/detector's unchanged segmentation and Kalman tracker."""
from collections import Counter
import math
from dataclasses import replace
import numpy as np
from .segmentation import RobotModel, foreground_mask, cluster_xy, split_clusters, inspect_cluster, major_extent
from .tracker import Tracker, TrackerConfig


class StaticBackground:
    """Occupied cells only; observed dynamic cells must never be fed here."""
    def __init__(self, resolution, origin, width, height, data, margin=.08):
        if resolution <= 0 or len(data) != width * height:
            raise ValueError('invalid occupancy grid')
        self.resolution = resolution
        self.origin = np.asarray(origin)
        self.width, self.height = width, height
        self.grid = np.asarray(data).reshape(height, width)
        occupied = self.grid >= 50
        self.mask = occupied.copy()
        radius = math.ceil(margin / resolution + .71)
        # Raster approximation to the original analytic wall margin.
        padded = np.pad(occupied, radius)
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                if math.hypot(dx, dy) * resolution <= margin + .71 * resolution:
                    self.mask |= padded[radius+dy:radius+dy+height, radius+dx:radius+dx+width]

    def foreground(self, points):
        indices = np.floor((points[:, :2] - self.origin) / self.resolution).astype(int)
        x, y = indices.T
        inside = (x >= 0) & (y >= 0) & (x < self.width) & (y < self.height)
        keep = inside.copy()
        keep[inside] &= ~self.mask[y[inside], x[inside]]
        keep[inside] &= self.grid[y[inside], x[inside]] >= 0
        return keep

    def free_center(self, xy):
        """A fitted robot center cannot lie in a wall or unknown map cell."""
        x, y = np.floor((np.asarray(xy) - self.origin) / self.resolution).astype(int)
        return (0 <= x < self.width and 0 <= y < self.height
                and 0 <= self.grid[y, x] < 50)


class Detector:
    def __init__(self, model=None, tracker=None, cluster_tolerance=.10, self_range=.30, strong_arc_min_span_deg=0., allow_merged_strong=True, strong_min_inlier_fraction=0., strong_min_extent=0., strong_rectangle_ratio=0.):
        self.model = model or RobotModel()
        self.tracker = Tracker(tracker or TrackerConfig())
        self.cluster_tolerance = cluster_tolerance
        self.self_range = self_range
        self.strong_arc_min_span_deg = strong_arc_min_span_deg
        self.allow_merged_strong = allow_merged_strong
        self.strong_min_inlier_fraction = strong_min_inlier_fraction
        self.strong_min_extent = strong_min_extent
        self.strong_rectangle_ratio = strong_rectangle_ratio

    def step(self, points, sensor, static, stamp):
        points = np.asarray(points, dtype=float).reshape(-1, 3)
        points = points[np.isfinite(points).all(axis=1)]
        keep = foreground_mask(points, np.asarray(sensor), [], None, .08, .012, .06, .70)
        keep &= np.linalg.norm(points-np.asarray(sensor), axis=1) >= self.self_range
        foreground = points[keep & static.foreground(points)]
        labels, count = cluster_xy(foreground[:, :2], self.cluster_tolerance)
        clusters = split_clusters(foreground, labels, count)
        inspected = [inspect_cluster(cluster, np.asarray(sensor[:2]), self.model) for cluster in clusters]
        inspected = [(None, 'center in wall or unknown') if d is not None
                     and not static.free_center(d.center) else (d, reason)
                     for d, reason in inspected]
        if self.strong_min_extent > 0:
            inspected = [(replace(d, strong=False), 'short extent: weak only')
                         if d is not None and d.strong
                         and major_extent(c[:, :2]) < self.strong_min_extent else (d, reason)
                         for c, (d, reason) in zip(clusters, inspected)]
        if not self.allow_merged_strong:
            inspected = [(replace(d, strong=False) if d is not None and d.strong
                          and major_extent(c[:,:2]) > self.model.max_extent else d, reason)
                         for c,(d,reason) in zip(clusters,inspected)]
        if self.strong_arc_min_span_deg > 0 or self.strong_min_inlier_fraction > 0:
            checked = []
            for cluster, (detection, reason) in zip(clusters, inspected):
                if detection is not None and detection.strong:
                    rim = cluster[cluster[:,2] <= self.model.rim_max_z,:2]
                    inliers = np.abs(np.linalg.norm(rim-detection.center,axis=1)-self.model.radius) <= self.model.outlier
                    fraction = float(np.mean(inliers)) if len(rim) else 0.
                    rim = rim[inliers]
                    angles = np.sort(np.arctan2((rim-detection.center)[:,1],(rim-detection.center)[:,0]))
                    span = (2*np.pi-max(np.diff(np.r_[angles,angles[0]+2*np.pi]))) if len(angles)>1 else 0.
                    if (math.degrees(span) < self.strong_arc_min_span_deg
                            or fraction < self.strong_min_inlier_fraction):
                        # An ambiguous short fragment may continue a known robot,
                        # but cannot initialize a stationary box as a new robot.
                        detection = replace(detection, strong=False)
                        reason = 'ambiguous arc: weak only'
                checked.append((detection, reason))
            inspected = checked
        if self.strong_rectangle_ratio > 0:
            from .shape_confidence import rectangle_edge_mse
            checked = []
            for cluster, (detection, reason) in zip(clusters, inspected):
                if detection is not None and detection.strong:
                    rim = cluster[cluster[:,2] <= self.model.rim_max_z,:2]
                    circle_mse = float(np.mean((np.linalg.norm(rim-detection.center, axis=1)
                                               - self.model.radius)**2))
                    if rectangle_edge_mse(rim) < self.strong_rectangle_ratio * circle_mse:
                        detection = replace(detection, strong=False)
                        reason = 'rectangle fits better: weak only'
                checked.append((detection, reason))
            inspected = checked
        detections = [d for d, _ in inspected if d is not None]
        track = self.tracker.step(stamp, detections)
        return track, {'foreground_points': len(foreground), 'clusters': count,
                       'candidates': len(detections), 'strong_candidates': sum(d.strong for d in detections),
                       'rejections': dict(Counter(reason for d, reason in inspected if d is None)),
                       'weak_reasons': dict(Counter(reason or 'partial body'
                            for d, reason in inspected if d is not None and not d.strong)),
                       'tracks': len(self.tracker.tracks)}
