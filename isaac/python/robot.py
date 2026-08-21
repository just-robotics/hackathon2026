"""Import TurtleBot 2 URDF into the current Isaac Sim stage."""

import os

import carb
import omni.kit.app
import omni.usd
from pxr import Usd, UsdPhysics

from constants import (
    IMPORT_REV,
    ROBOT_PRIM,
    ROBOT_SPAWN,
    ROBOT_SPAWN_HEIGHT,
    URDF_PATH,
    USD_CACHE_DIR,
    USD_CACHE_PATH,
)
from scene import set_prim_pose

# Isaac 6 URDF importer writes a folder named *.usd, with the composed asset inside.
_COMPOSED_USD_REL = os.path.join("turtlebot2.usd", "turtlebot2", "turtlebot2.usda")
_FLAT_USD_NAME = "turtlebot2_flat.usd"


def _find_prim_by_name(stage, name):
    for prim in stage.Traverse():
        if prim.GetName() == name:
            return prim
    return None


def _resolve_usd_file(path):
    if path and os.path.isfile(path):
        return path
    candidates = []
    if path:
        candidates.extend(
            [
                os.path.join(path, "turtlebot2", "turtlebot2.usda"),
                os.path.join(path, "turtlebot2.usda"),
            ]
        )
    candidates.append(os.path.join(USD_CACHE_DIR, _COMPOSED_USD_REL))
    for candidate in candidates:
        if os.path.isfile(candidate):
            return candidate
    if path and os.path.isdir(path):
        for root, _dirs, files in os.walk(path):
            for name in files:
                if name.endswith((".usda", ".usd")):
                    return os.path.join(root, name)
    return None


def _flatten_usd(src_path):
    """Bake payload/reference layers so add_reference_to_stage sees all links."""
    flat_path = os.path.join(USD_CACHE_DIR, _FLAT_USD_NAME)
    if os.path.isfile(flat_path) and os.path.getmtime(flat_path) >= os.path.getmtime(src_path):
        return flat_path
    src_stage = Usd.Stage.Open(src_path)
    if src_stage is None:
        raise RuntimeError(f"Could not open imported USD {src_path}")
    flattened = src_stage.Flatten()
    flattened.Export(flat_path)
    carb.log_info(f"[turtlebot2] flattened {src_path} -> {flat_path}")
    return flat_path


def _enable_urdf_extension():
    manager = omni.kit.app.get_app().get_extension_manager()
    ext_id = "isaacsim.asset.importer.urdf"
    try:
        if not manager.is_extension_enabled(ext_id):
            manager.set_extension_enabled_immediate(ext_id, True)
    except Exception as exc:
        carb.log_warn(f"[turtlebot2] could not enable {ext_id}: {exc}")


def _import_with_urdf_importer():
    from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig

    os.makedirs(USD_CACHE_DIR, exist_ok=True)
    last_error = None
    for usd_path in (USD_CACHE_PATH, USD_CACHE_DIR):
        kwargs = dict(
            urdf_path=URDF_PATH,
            usd_path=usd_path,
            merge_mesh=False,
            allow_self_collision=False,
            collision_from_visuals=False,
            fix_base=False,
            joint_drive_type="force",
            joint_target_type="velocity",
            override_joint_stiffness=0.0,
            override_joint_damping=1.0e4,
        )
        try:
            config = URDFImporterConfig(**kwargs)
        except TypeError:
            try:
                config = URDFImporterConfig(
                    urdf_path=URDF_PATH, usd_path=usd_path, fix_base=False
                )
            except TypeError:
                config = URDFImporterConfig(urdf_path=URDF_PATH, usd_path=usd_path)
        try:
            importer = URDFImporter(config)
            output_path = importer.import_urdf()
            resolved = _resolve_usd_file(output_path) or _resolve_usd_file(USD_CACHE_PATH)
            carb.log_info(f"[turtlebot2] URDFImporter wrote {output_path} -> {resolved}")
            if resolved:
                return resolved
        except Exception as exc:
            last_error = exc
            carb.log_warn(f"[turtlebot2] URDFImporter usd_path={usd_path} failed: {exc}")
    raise last_error or RuntimeError("URDFImporter failed")


def _add_reference(usd_path):
    from pxr import Sdf

    stage = omni.usd.get_context().get_stage()
    prim = stage.DefinePrim(ROBOT_PRIM, "Xform")
    prim.SetInstanceable(False)
    refs = prim.GetReferences()
    refs.ClearReferences()
    # Flattened asset default prim is /turtlebot2; bind it explicitly so the
    # Xform at ROBOT_PRIM actually receives Geometry/Physics children.
    try:
        refs.AddReference(Sdf.Reference(usd_path, Sdf.Path("/turtlebot2")))
    except Exception as exc:
        carb.log_warn(f"[turtlebot2] explicit primPath reference failed ({exc}), trying file defaultPrim")
        refs.AddReference(usd_path)
    if not prim.GetChildren():
        try:
            from isaacsim.core.utils.stage import add_reference_to_stage
        except Exception:
            from omni.isaac.core.utils.stage import add_reference_to_stage
        add_reference_to_stage(usd_path=usd_path, prim_path=ROBOT_PRIM)
    carb.log_info(
        f"[turtlebot2] robot children: {[c.GetName() for c in stage.GetPrimAtPath(ROBOT_PRIM).GetChildren()]}"
    )
    return ROBOT_PRIM


def _import_rev_path():
    return os.path.join(USD_CACHE_DIR, ".import_rev")


def _is_fresh(path):
    stamp = _import_rev_path()
    if not os.path.isfile(stamp):
        return False
    try:
        with open(stamp, encoding="utf-8") as handle:
            if handle.read().strip() != IMPORT_REV:
                return False
    except OSError:
        return False
    return bool(
        path and os.path.isfile(path) and os.path.getmtime(path) >= os.path.getmtime(URDF_PATH)
    )


def _write_import_rev():
    os.makedirs(USD_CACHE_DIR, exist_ok=True)
    with open(_import_rev_path(), "w", encoding="utf-8") as handle:
        handle.write(IMPORT_REV)


def _clear_stale_cache():
    import shutil

    if not os.path.isdir(USD_CACHE_DIR):
        return
    for name in os.listdir(USD_CACHE_DIR):
        path = os.path.join(USD_CACHE_DIR, name)
        try:
            if os.path.isdir(path):
                shutil.rmtree(path)
            else:
                os.remove(path)
        except Exception as exc:
            carb.log_warn(f"[turtlebot2] could not clear cache {path}: {exc}")


def ensure_robot_usd():
    """Return a composed USD file for TurtleBot 2, importing the URDF if needed."""
    if not os.path.isfile(URDF_PATH):
        raise FileNotFoundError(f"TurtleBot 2 URDF not found: {URDF_PATH}")
    flat_path = os.path.join(USD_CACHE_DIR, _FLAT_USD_NAME)
    if _is_fresh(flat_path):
        carb.log_info(f"[turtlebot2] reusing flattened USD {flat_path}")
        return flat_path
    cached = _resolve_usd_file(USD_CACHE_PATH)
    if _is_fresh(cached):
        flat_path = _flatten_usd(cached)
        _write_import_rev()
        return flat_path
    carb.log_info("[turtlebot2] URDF newer than cache, reimporting")
    _clear_stale_cache()
    _enable_urdf_extension()
    flat_path = _flatten_usd(_import_with_urdf_importer())
    _write_import_rev()
    return flat_path


def add_robot_to_stage(usd_path, floor_z=0.0):
    # Drop the robot from just above the floor so PhysX settles the wheels on it.
    spawn = (ROBOT_SPAWN[0], ROBOT_SPAWN[1], floor_z + ROBOT_SPAWN_HEIGHT)
    stage = omni.usd.get_context().get_stage()
    existing = stage.GetPrimAtPath(ROBOT_PRIM) if stage else None
    if existing and existing.IsValid():
        carb.log_info("[turtlebot2] robot already on stage")
        set_prim_pose(ROBOT_PRIM, spawn)
        return ROBOT_PRIM

    usd_file = _resolve_usd_file(usd_path)
    if not usd_file:
        raise RuntimeError(f"TurtleBot 2 USD not found from {usd_path}")
    robot_path = _add_reference(usd_file)
    set_prim_pose(robot_path, spawn)
    carb.log_info(f"[turtlebot2] spawned {robot_path} at z={spawn[2]:.3f} from {usd_file}")
    return robot_path


def spawn_turtlebot():
    """Import (or reuse cached USD) and add the robot to the current stage."""
    usd_path = ensure_robot_usd()
    return add_robot_to_stage(usd_path)


def find_link(robot_prim_path, link_name):
    stage = omni.usd.get_context().get_stage()
    direct = f"{robot_prim_path}/{link_name}"
    prim = stage.GetPrimAtPath(direct)
    if prim and prim.IsValid():
        return str(prim.GetPath())
    prefix = robot_prim_path.rstrip("/") + "/"
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if prim.GetName() == link_name and (path == robot_prim_path or path.startswith(prefix)):
            return path
    found = _find_prim_by_name(stage, link_name)
    return str(found.GetPath()) if found else None


def find_articulation_root(robot_prim_path):
    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(robot_prim_path)
    if root and root.HasAPI(UsdPhysics.ArticulationRootAPI):
        return robot_prim_path
    prefix = robot_prim_path.rstrip("/") + "/"
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if path.startswith(prefix) and prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            return path
    return find_link(robot_prim_path, "base_link") or robot_prim_path
