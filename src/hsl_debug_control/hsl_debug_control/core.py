from math import atan2, hypot, pi


DIRECT_MPPI_STATUSES = ("OK", "RECOVERY_MPPI")


def angle_error(a, b):
    return (a - b + pi) % (2 * pi) - pi


def select_control_command(mode, planner_status, mpc_command, mppi_command):
    """Use MPPI for its checked path, keeping MPC for recovery paths."""
    if mode == "mppi" and planner_status in DIRECT_MPPI_STATUSES:
        return mppi_command
    return mpc_command


def path_turning_decision(points, own, yaw, already_turning,
                          curve_forward_error=1.0):
    """Rotate in place for straight paths, but track checked curves jointly."""
    if len(points) < 2:
        return None, False, False
    start, end = points[0], points[-1]
    dx, dy = end[0] - start[0], end[1] - start[1]
    span = hypot(dx, dy)
    if span < 0.02:
        return None, False, False
    deviation = max(abs((x - start[0]) * dy - (y - start[1]) * dx) / span
                    for x, y in points)
    is_curve = deviation > 0.035
    # A short MPPI arc can have a nearly straight chord even though its first
    # few poses point somewhere else. Check the tangent that MPC will follow
    # next, rather than requiring the robot to face the far endpoint first.
    nearest = min(range(len(points)),
                  key=lambda i: hypot(points[i][0] - own[0],
                                      points[i][1] - own[1]))
    before = points[max(0, nearest - 2)]
    after = points[min(len(points) - 1, nearest + 3)]
    tangent_dx, tangent_dy = after[0] - before[0], after[1] - before[1]
    desired = (atan2(tangent_dy, tangent_dx)
               if hypot(tangent_dx, tangent_dy) >= 0.02 else atan2(dy, dx))
    error = angle_error(desired, yaw)
    threshold = (curve_forward_error if is_curve else
                 (0.75 if already_turning else 1.0))
    if abs(error) > threshold:
        return error, True, is_curve
    return None, False, is_curve


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


def safe_mpc_command(now, pose_stamp, scan_stamp, path, intent, command,
                     pose_timeout=1.2, scan_timeout=1.8,
                     path_timeout=1.0, intent_timeout=1.0,
                     command_timeout=0.5, rotation_error=None):
    """Stop the MPC output when the navigation contract is not current."""
    if not path or not intent:
        return 0.0, 0.0
    points, path_stamp = path
    behavior, max_speed, intent_stamp = intent
    if (behavior in (0, 1) or not points or max_speed <= 0
            or now - pose_stamp > pose_timeout
            or now - scan_stamp > scan_timeout
            or now - path_stamp > path_timeout
            or now - intent_stamp > intent_timeout):
        return 0.0, 0.0
    if rotation_error is not None:
        return 0.0, max(-1.0, min(1.0, 2.0 * rotation_error)) if abs(rotation_error) > 0.1 else 0.0
    if not command:
        return 0.0, 0.0
    linear, angular, command_stamp = command
    if now - command_stamp > command_timeout:
        return 0.0, 0.0
    return max(-max_speed, min(max_speed, linear)), angular


def match_is_active(now, state, timeout=0.5):
    """Fail closed until the common referee start, and if its heartbeat stops."""
    return bool(state and state[0] and 0.0 <= now - state[1] <= timeout)
