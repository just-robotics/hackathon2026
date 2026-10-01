DIRECT_MPPI_STATUSES = ("OK", "RECOVERY_MPPI")


def select_control_command(planner_status, mppi_command):
    """Accept only the direct command of a checked MPPI trajectory."""
    return mppi_command if planner_status in DIRECT_MPPI_STATUSES else None


def safe_motion_command(now, pose_stamp, scan_stamp, path, intent, command,
                     pose_timeout=1.2, scan_timeout=1.8,
                     path_timeout=1.0, intent_timeout=1.0,
                     command_timeout=0.5):
    """Stop the command when the navigation contract is not current."""
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
    if not command:
        return 0.0, 0.0
    linear, angular, command_stamp = command
    if now - command_stamp > command_timeout:
        return 0.0, 0.0
    return max(-max_speed, min(max_speed, linear)), angular


def match_is_active(now, state, timeout=0.5):
    """Fail closed until the common referee start, and if its heartbeat stops."""
    return bool(state and state[0] and 0.0 <= now - state[1] <= timeout)
