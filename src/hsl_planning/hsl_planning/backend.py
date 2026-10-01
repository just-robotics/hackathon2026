"""Select the controller backend while preserving stock MPC as a reserve."""


def resolve_backend(control_mode, local_backend="auto"):
    if control_mode not in ("mppi", "mpc"):
        raise ValueError("control_mode must be mppi or mpc")
    if local_backend == "auto":
        return "nav2_cpp" if control_mode == "mppi" else "python"
    if local_backend not in ("python", "nav2_cpp"):
        raise ValueError("local_backend must be auto, python or nav2_cpp")
    if local_backend == "nav2_cpp" and control_mode != "mppi":
        raise ValueError("nav2_cpp requires control_mode=mppi")
    return local_backend
