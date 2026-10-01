"""Select the native or Python MPPI backend."""


def resolve_backend(local_backend="auto"):
    if local_backend == "auto":
        return "nav2_cpp"
    if local_backend not in ("python", "nav2_cpp"):
        raise ValueError("local_backend must be auto, python or nav2_cpp")
    return local_backend
