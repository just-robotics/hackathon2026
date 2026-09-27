"""Simple point-to-point patrol for the simulated opponent."""

from math import atan2, hypot, pi


def patrol_command(x, y, yaw, goal_x, goal_y):
    distance = hypot(goal_x - x, goal_y - y)
    if distance < 0.14:
        return 0.0, 0.0, True
    desired = atan2(goal_y - y, goal_x - x)
    error = (desired - yaw + pi) % (2 * pi) - pi
    angular = max(-0.8, min(0.8, 1.8 * error))
    linear = min(0.23, 0.6 * distance) if abs(error) < 0.35 else 0.0
    return linear, angular, False
