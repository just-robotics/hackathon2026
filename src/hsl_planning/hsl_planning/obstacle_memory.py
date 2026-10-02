"""Short-lived LiDAR occupancy, with visibility-based clearing in map coordinates."""
from math import atan2, floor, hypot, isfinite, pi


class ObstacleMemory:
    def __init__(self, resolution=.05, lifetime=8.):
        self.resolution = resolution
        self.lifetime = lifetime
        self.cells = {}
        self.last_stamp = None

    def update(self, points, origin, stamp):
        if self.last_stamp is not None and stamp < self.last_stamp:
            self.cells.clear()
        self.last_stamp = stamp
        hits, visibility = {}, {}
        for x, y, z in points:
            if not all(isfinite(v) for v in (x, y, z)) or not .08 <= z <= .60:
                continue
            distance = hypot(x-origin.x, y-origin.y)
            if not .25 <= distance <= 4.5:
                continue
            bearing = floor(atan2(y-origin.y, x-origin.x) * 180/pi)
            visibility[bearing] = min(distance, visibility.get(bearing, float('inf')))
            cell = (floor(x/self.resolution), floor(y/self.resolution))
            hits[cell] = (x,y,z,stamp)
        for cell, (x,y,z,last) in list(self.cells.items()):
            distance = hypot(x-origin.x,y-origin.y)
            bearing = floor(atan2(y-origin.y,x-origin.x)*180/pi)
            # Clear only where a current obstacle-band ray reaches beyond the
            # old return. Occluded/out-of-view cells retain bounded memory.
            if (stamp-last > self.lifetime or
                (cell not in hits and visibility.get(bearing,0.) > distance+.10)):
                del self.cells[cell]
        self.cells.update(hits)

    def forget(self, points):
        """Erase previously marked cells when their returns are reclassified."""
        for x,y,_ in points:
            cx,cy=floor(x/self.resolution),floor(y/self.resolution)
            for dx in (-1,0,1):
                for dy in (-1,0,1):
                    self.cells.pop((cx+dx,cy+dy),None)

    def points(self, stamp, exclude=None):
        return [(x,y,z) for x,y,z,last in self.cells.values()
                if 0 <= stamp-last <= self.lifetime and
                (exclude is None or hypot(x-exclude.x,y-exclude.y) > .35)]
