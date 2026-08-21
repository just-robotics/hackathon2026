"""PhysX friction and wheel velocity drives."""

import carb
import omni.usd
from pxr import PhysxSchema, UsdPhysics, UsdShade

# Velocity drive: tau = damping * (v_target - v). Stiffness stays 0.
# Isaac URDF import creates DriveAPI with maxForce only, so cmd_vel is ignored.
WHEEL_DRIVE_DAMPING = 1.0e4
WHEEL_DRIVE_MAX_FORCE = 1.0e4


def _bind_material(stage, prim, material_path, static_friction, dynamic_friction):
    mat = UsdShade.Material.Define(stage, material_path)
    api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    api.CreateStaticFrictionAttr(static_friction)
    api.CreateDynamicFrictionAttr(dynamic_friction)
    api.CreateRestitutionAttr(0.0)
    try:
        physx = PhysxSchema.PhysxMaterialAPI.Apply(mat.GetPrim())
        physx.CreateFrictionCombineModeAttr("average")
    except Exception:
        pass
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(
        mat, UsdShade.Tokens.weakerThanDescendants, "physics"
    )


def _iter_named(stage, names):
    wanted = set(names)
    for prim in stage.Traverse():
        if prim.GetName() in wanted:
            yield prim


def _bind_on_colliders(stage, root_prim, material_path, static_friction, dynamic_friction):
    _bind_material(stage, root_prim, material_path, static_friction, dynamic_friction)
    for child in root_prim.GetChildren():
        if child.HasAPI(UsdPhysics.CollisionAPI):
            _bind_material(stage, child, material_path, static_friction, dynamic_friction)


def configure_wheel_drives(robot_prim_path):
    """Give wheel joints a velocity drive so ArticulationController can apply cmd_vel."""
    stage = omni.usd.get_context().get_stage()
    for prim in _iter_named(stage, ("wheel_left_joint", "wheel_right_joint")):
        path = str(prim.GetPath())
        if not path.startswith(robot_prim_path.rstrip("/") + "/") and path != robot_prim_path:
            # Still apply: joints live under Physics/, which is under the robot root.
            pass
        drive = UsdPhysics.DriveAPI.Get(prim, "angular")
        if not drive:
            drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(0.0)
        drive.CreateDampingAttr(WHEEL_DRIVE_DAMPING)
        drive.CreateMaxForceAttr(WHEEL_DRIVE_MAX_FORCE)
        carb.log_info(
            f"[turtlebot2] velocity drive on {prim.GetPath()} damping={WHEEL_DRIVE_DAMPING}"
        )


def make_robot_dynamic(robot_prim_path):
    """Detach world-welded base joints and keep rigid bodies non-kinematic.

    An empty URDF ``base_footprint`` is imported as a PhysicsFixedJoint from the
    robot Xform (no RigidBodyAPI) to ``base_link``, which pins the robot in air.
    """
    stage = omni.usd.get_context().get_stage()
    prefix = robot_prim_path.rstrip("/")
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if path != prefix and not path.startswith(prefix + "/"):
            continue
        if "Joint" in prim.GetTypeName():
            joint = UsdPhysics.Joint(prim)
            targets = list(joint.GetBody0Rel().GetTargets()) if joint.GetBody0Rel() else []
            body0 = stage.GetPrimAtPath(targets[0]) if targets else None
            if not targets or not body0 or not body0.HasAPI(UsdPhysics.RigidBodyAPI):
                prim.SetActive(False)
                carb.log_info(f"[turtlebot2] disabled world-anchor joint {path}")
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            body = UsdPhysics.RigidBodyAPI(prim)
            body.CreateRigidBodyEnabledAttr(True)
            body.CreateKinematicEnabledAttr(False)


def tune_wheel_caster_friction(robot_prim_path):
    stage = omni.usd.get_context().get_stage()
    materials_root = f"{robot_prim_path}/Looks"
    for prim in _iter_named(stage, ("wheel_left_link", "wheel_right_link")):
        _bind_on_colliders(
            stage,
            prim,
            f"{materials_root}/wheel_friction",
            static_friction=1.4,
            dynamic_friction=1.2,
        )
        carb.log_info(f"[turtlebot2] high friction on {prim.GetPath()}")
    for prim in _iter_named(stage, ("caster_front_link", "caster_rear_link")):
        _bind_on_colliders(
            stage,
            prim,
            f"{materials_root}/caster_friction",
            static_friction=0.02,
            dynamic_friction=0.02,
        )
        carb.log_info(f"[turtlebot2] low friction on {prim.GetPath()}")
    configure_wheel_drives(robot_prim_path)
