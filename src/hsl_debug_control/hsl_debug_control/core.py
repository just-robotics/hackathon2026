from math import atan2, hypot, pi


def angle_error(a, b):
    return (a - b + pi) % (2 * pi) - pi


def follow(own, points, max_speed):
    """Return linear and angular speed; own/points are x,y,yaw triples."""
    if not points:
        return 0.0, 0.0
    x, y, yaw = own
    target = points[-1]
    for point in points:
        if hypot(point[0] - x, point[1] - y) >= 0.45:
            target = point
            break
    distance = hypot(target[0] - x, target[1] - y)
    if distance < 0.08:
        error = angle_error(target[2], yaw)
        return 0.0, max(-1.0, min(1.0, 2.0 * error)) if abs(error) > 0.1 else 0.0
    heading = angle_error(atan2(target[1] - y, target[0] - x), yaw)
    angular = max(-1.2, min(1.2, 2.2 * heading))
    linear = min(max_speed, 0.8 * distance) * max(0.0, 1.0 - abs(heading) / 1.3)
    return linear, angular


def safe_follow(now, own, path, intent, pose_timeout=0.5, path_timeout=0.5,
                intent_timeout=0.6):
    """Guard the test controller against stale data and forbidden motion."""
    if not own or not path or not intent:
        return 0.0, 0.0
    pose, pose_stamp = own
    points, path_stamp = path
    behavior, max_speed, intent_stamp = intent
    if (behavior in (0, 1) or now - pose_stamp > pose_timeout
            or now - path_stamp > path_timeout
            or now - intent_stamp > intent_timeout):
        return 0.0, 0.0
    return follow(pose, points, max_speed)
