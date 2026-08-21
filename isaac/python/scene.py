"""Load Simple Room (or a ground-plane fallback) and hide the table."""

import carb
import omni.kit.commands
import omni.usd
from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics

from constants import ROBOT_SPAWN, SIMPLE_ROOM_REL

ROOM_PRIM = "/World/SimpleRoom"


def _assets_root():
    for module_name in (
        "isaacsim.storage.native",
        "isaacsim.core.utils.nucleus",
        "omni.isaac.core.utils.nucleus",
    ):
        try:
            module = __import__(module_name, fromlist=["get_assets_root_path"])
            root = module.get_assets_root_path()
            if root:
                return root
        except Exception:
            continue
    return None


async def _wait_stage_loaded():
    try:
        from isaacsim.core.utils.stage import is_stage_loading
    except Exception:
        try:
            from omni.isaac.core.utils.stage import is_stage_loading
        except Exception:
            is_stage_loading = lambda: False
    import omni.kit.app

    app = omni.kit.app.get_app()
    for _ in range(300):
        await app.next_update_async()
        if not is_stage_loading():
            for _ in range(5):
                await app.next_update_async()
            return


def _hide_tables(stage):
    for prim in stage.Traverse():
        name = prim.GetName().lower()
        if "table" in name:
            prim.SetActive(False)
            carb.log_info(f"[turtlebot2] deactivated {prim.GetPath()}")


def _ensure_world(stage):
    if not stage.GetPrimAtPath("/World"):
        xform = UsdGeom.Xform.Define(stage, "/World")
        stage.SetDefaultPrim(xform.GetPrim())


def _add_ground_collider(stage, floor_z):
    path = "/World/groundCollider"
    if stage.GetPrimAtPath(path):
        return
    xform = UsdGeom.Xform.Define(stage, path)
    xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, floor_z - 0.01))
    xform.AddScaleOp().Set(Gf.Vec3d(50.0, 50.0, 0.02))
    cube = UsdGeom.Cube.Define(stage, f"{path}/collision")
    cube.CreateSizeAttr(1.0)
    UsdGeom.Imageable(cube.GetPrim()).CreatePurposeAttr("guide")
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    carb.log_info(f"[turtlebot2] added static ground collider at z={floor_z:.3f}")


def _room_floor_z(stage, room_path):
    """World Z of the lowest room geometry: the visible floor the robot must stand on."""
    prim = stage.GetPrimAtPath(room_path)
    if not prim or not prim.IsValid():
        return 0.0
    cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render]
    )
    box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    if box.IsEmpty():
        carb.log_warn("[turtlebot2] room bbox is empty, assuming floor z=0")
        return 0.0
    low, high = box.GetMin(), box.GetMax()
    carb.log_info(
        f"[turtlebot2] room bbox z {low[2]:.3f}..{high[2]:.3f}, xy "
        f"({low[0]:.2f},{low[1]:.2f})..({high[0]:.2f},{high[1]:.2f})"
    )
    return float(low[2])


def _floor_top_z(stage, room_path):
    """Top surface of the widest thin mesh under the origin, i.e. the room floor.

    The room bounding box is useless for placement: Simple Room spans z -3.8..6.2
    because of its shell, while the walkable floor sits near z=-0.77.
    """
    root = stage.GetPrimAtPath(room_path)
    if not root or not root.IsValid():
        return None
    cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render]
    )
    best_area = 0.0
    best_top = None
    for prim in Usd.PrimRange(root):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        if box.IsEmpty():
            continue
        low, high = box.GetMin(), box.GetMax()
        if high[2] - low[2] > 0.5:
            continue
        if not (low[0] <= 0.0 <= high[0] and low[1] <= 0.0 <= high[1]):
            continue
        area = (high[0] - low[0]) * (high[1] - low[1])
        if area > best_area:
            best_area, best_top = area, float(high[2])
    if best_top is not None:
        carb.log_info(f"[turtlebot2] floor mesh top z={best_top:.3f} area={best_area:.1f}")
    return best_top


def _count_colliders(stage, root_path):
    root = stage.GetPrimAtPath(root_path)
    if not root or not root.IsValid():
        return 0
    return sum(1 for prim in Usd.PrimRange(root) if prim.HasAPI(UsdPhysics.CollisionAPI))


def _ensure_physics_scene(stage):
    ours = "/World/physicsScene"
    extra = []
    found_ours = False
    for prim in stage.Traverse():
        if prim.GetTypeName() != "PhysicsScene":
            continue
        if str(prim.GetPath()) == ours:
            found_ours = True
        else:
            extra.append(prim)
    if not found_ours:
        scene = UsdPhysics.Scene.Define(stage, ours)
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr().Set(9.81)
        carb.log_info("[turtlebot2] created /World/physicsScene")
    else:
        scene = UsdPhysics.Scene.Get(stage, ours)
        if scene:
            scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
            scene.CreateGravityMagnitudeAttr().Set(9.81)
    for prim in extra:
        prim.SetActive(False)
        carb.log_info(f"[turtlebot2] deactivated extra physics scene {prim.GetPath()}")


def _ensure_lights(stage):
    """Купол плюс солнце: без комнаты своих источников на сцене нет."""
    dome_path = "/World/DomeLight"
    if not stage.GetPrimAtPath(dome_path):
        dome = UsdLux.DomeLight.Define(stage, dome_path)
        dome.CreateIntensityAttr(1200.0)
        dome.CreateColorAttr(Gf.Vec3f(0.65, 0.72, 0.85))
        carb.log_info(f"[turtlebot2] created {dome_path}")
    sun_path = "/World/DistantLight"
    if not stage.GetPrimAtPath(sun_path):
        sun = UsdLux.DistantLight.Define(stage, sun_path)
        sun.CreateIntensityAttr(3000.0)
        sun.CreateAngleAttr(0.53)
        UsdGeom.Xformable(sun.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(310.0, 0.0, 25.0))
        carb.log_info(f"[turtlebot2] created {sun_path}")


def _create_ground_scene(stage):
    """Пол, свет и физика без комнаты: сюда же откатываемся, если её нет."""
    _ensure_world(stage)
    _ensure_physics_scene(stage)
    omni.kit.commands.execute(
        "AddGroundPlaneCommand",
        stage=stage,
        planePath="/World/groundPlane",
        axis="Z",
        size=25.0,
        position=Gf.Vec3f(0, 0, 0),
        color=Gf.Vec3f(0.35, 0.35, 0.35),
    )
    _ensure_lights(stage)
    cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render]
    )
    box = cache.ComputeWorldBound(stage.GetPrimAtPath("/World/groundPlane")).ComputeAlignedRange()
    if box.IsEmpty():
        carb.log_warn("[turtlebot2] ground plane has no renderable geometry")
    else:
        low, high = box.GetMin(), box.GetMax()
        carb.log_info(
            f"[turtlebot2] ground plane render bbox ({low[0]:.1f},{low[1]:.1f},{low[2]:.2f})"
            f"..({high[0]:.1f},{high[1]:.1f},{high[2]:.2f})"
        )


async def load_simple_room(use_room=True):
    """Create a local /World, reference Simple Room into it, return the floor height.

    С use_room=False комната не грузится вовсе: остаётся плоскость с землёй на
    z=0, робот и лидар строятся поверх неё как обычно.
    """
    context = omni.usd.get_context()
    context.new_stage()
    await _wait_stage_loaded()
    stage = context.get_stage()
    _ensure_world(stage)

    if not use_room:
        carb.log_info("[turtlebot2] Simple Room disabled by flag")
        _create_ground_scene(stage)
        return 0.0

    root = _assets_root()
    if not root:
        carb.log_warn("[turtlebot2] no assets root, falling back to a ground plane")
        _create_ground_scene(stage)
        return 0.0

    usd_path = f"{root}{SIMPLE_ROOM_REL}"
    carb.log_info(f"[turtlebot2] referencing {usd_path}")
    try:
        from isaacsim.core.utils.stage import add_reference_to_stage

        add_reference_to_stage(usd_path=usd_path, prim_path=ROOM_PRIM)
        await _wait_stage_loaded()
    except Exception as exc:
        carb.log_error(f"[turtlebot2] Simple Room reference failed: {exc}")
        _create_ground_scene(stage)
        return 0.0

    stage = context.get_stage()
    _ensure_physics_scene(stage)
    _hide_tables(stage)
    floor_z = _floor_top_z(stage, ROOM_PRIM)
    if floor_z is None:
        floor_z = _room_floor_z(stage, ROOM_PRIM)
    colliders = _count_colliders(stage, ROOM_PRIM)
    carb.log_info(f"[turtlebot2] room colliders={colliders} floor_z={floor_z:.3f}")
    if colliders == 0:
        _add_ground_collider(stage, floor_z)
    return floor_z


def set_prim_pose(prim_path, translation=ROBOT_SPAWN):
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        return
    xform = UsdGeom.Xformable(prim)
    ops = {op.GetOpName(): op for op in xform.GetOrderedXformOps()}
    if "xformOp:translate" in ops:
        ops["xformOp:translate"].Set(Gf.Vec3d(*translation))
        return
    xform.AddTranslateOp().Set(Gf.Vec3d(*translation))
