"""Attach an RTX lidar approximating Livox Mid-360 onto livox_frame."""

import carb
import omni.usd
from pxr import Gf, Sdf

from constants import (
    LIDAR_ELEVATION_MAX_DEG,
    LIDAR_ELEVATION_MIN_DEG,
    LIDAR_FAR_M,
    LIDAR_FIRING_HZ,
    LIDAR_NEAR_M,
    LIDAR_SCAN_HZ,
)

_ELEVATION_ATTR = "omni:sensor:Core:emitterState:s001:elevationDeg"


def _set_attr(prim, name, value, type_name):
    attr = prim.GetAttribute(name)
    if not attr or not attr.IsValid():
        attr = prim.CreateAttribute(name, type_name)
    try:
        attr.Set(value)
    except Exception as exc:
        carb.log_warn(f"[turtlebot2] could not set {name}: {exc}")


def _beams_per_bank(elevations):
    """Length of one rising elevation run; Example_Rotary repeats it per azimuth bank."""
    for i in range(1, len(elevations)):
        if elevations[i] <= elevations[i - 1]:
            return i
    return len(elevations)


def _remap_elevations(prim):
    """Stretch the profile's elevations onto the Mid-360 FOV, keeping array length.

    LidarCore validates every emitterState array against numberOfEmitters, so the
    banks, fire times and azimuth offsets of the source profile stay untouched.
    """
    attr = prim.GetAttribute(_ELEVATION_ATTR)
    if not attr or not attr.IsValid():
        carb.log_warn(f"[turtlebot2] {_ELEVATION_ATTR} missing, keeping profile FOV")
        return
    current = attr.Get()
    if current is None or len(current) == 0:
        return
    values = list(current)
    bank = _beams_per_bank(values)
    span = LIDAR_ELEVATION_MAX_DEG - LIDAR_ELEVATION_MIN_DEG
    if bank > 1:
        ramp = [LIDAR_ELEVATION_MIN_DEG + span * i / (bank - 1) for i in range(bank)]
    else:
        ramp = [LIDAR_ELEVATION_MIN_DEG]
    attr.Set([ramp[i % bank] for i in range(len(values))])
    carb.log_info(
        f"[turtlebot2] lidar FOV {LIDAR_ELEVATION_MIN_DEG}..{LIDAR_ELEVATION_MAX_DEG} deg "
        f"over {len(values)} emitters ({bank} per bank)"
    )


def _configure_mid360(prim):
    _set_attr(prim, "omni:sensor:tickRate", float(LIDAR_SCAN_HZ), Sdf.ValueTypeNames.Float)
    _set_attr(prim, "omni:sensor:Core:scanRateBaseHz", float(LIDAR_SCAN_HZ), Sdf.ValueTypeNames.Float)
    _set_attr(prim, "omni:sensor:Core:patternFiringRateHz", float(LIDAR_FIRING_HZ), Sdf.ValueTypeNames.Float)
    _set_attr(prim, "omni:sensor:Core:nearRangeM", float(LIDAR_NEAR_M), Sdf.ValueTypeNames.Float)
    _set_attr(prim, "omni:sensor:Core:farRangeM", float(LIDAR_FAR_M), Sdf.ValueTypeNames.Float)
    _set_attr(prim, "omni:sensor:Core:maxReturns", 1, Sdf.ValueTypeNames.Int)
    _set_attr(prim, "omni:sensor:Core:accumulateOutputs", True, Sdf.ValueTypeNames.Bool)
    _remap_elevations(prim)


def _create_via_command(parent, lidar_path):
    import omni.kit.commands

    name = lidar_path.rsplit("/", 1)[-1]
    _, sensor = omni.kit.commands.execute(
        "IsaacSensorCreateRtxLidar",
        path=name,
        parent=parent,
        config="Example_Rotary",
        translation=Gf.Vec3d(0.0, 0.0, 0.0),
        orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),
    )
    if sensor is None:
        return lidar_path
    return str(sensor.GetPath()) if hasattr(sensor, "GetPath") else lidar_path


def _hide_livox_body(livox_prim_path):
    """Disable the livox collision; its shell is inside nearRangeM anyway."""
    from pxr import Usd, UsdPhysics

    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(livox_prim_path)
    if not root or not root.IsValid():
        return
    for prim in Usd.PrimRange(root):
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False)


def attach_mid360(livox_prim_path):
    """Create the RTX lidar under livox_frame and return its prim path."""
    _hide_livox_body(livox_prim_path)
    lidar_path = f"{livox_prim_path}/Lidar"
    stage = omni.usd.get_context().get_stage()
    existing = stage.GetPrimAtPath(lidar_path)
    if existing and existing.IsValid():
        _configure_mid360(existing)
        return lidar_path

    created = _create_via_command(livox_prim_path, lidar_path)
    prim = stage.GetPrimAtPath(created)
    if not prim or not prim.IsValid():
        parent = stage.GetPrimAtPath(livox_prim_path)
        for child in parent.GetChildren() if parent else []:
            type_name = child.GetTypeName()
            if "Lidar" in type_name or "lidar" in child.GetName().lower():
                prim = child
                created = str(child.GetPath())
                break
    if not prim or not prim.IsValid():
        raise RuntimeError(f"RTX lidar prim was not created at {created}")

    _configure_mid360(prim)
    carb.log_info(f"[turtlebot2] Mid-360-like lidar at {created}")
    return created
