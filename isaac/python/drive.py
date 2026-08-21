"""Apply velocity drives after PhysX is running and follow /cmd_vel."""

import carb
import omni.graph.core as og
import omni.kit.app
import omni.timeline

from constants import GRAPH_DRIVE, WHEEL_DISTANCE, WHEEL_RADIUS

WHEEL_DRIVE_DAMPING = 1.0e4
WHEEL_DRIVE_MAX_FORCE = 1.0e4


class WheelDriver:
    """Set PD gains on the Kobuki wheels and apply Twist from the OmniGraph subscriber."""

    def __init__(self, art_root):
        self.art_root = art_root
        self.art = None
        self.dof_indices = None
        self._physx_sub = None
        self._update_sub = None
        self._ready_logged = False

    def start(self):
        try:
            from omni.physx import get_physx_interface

            self._physx_sub = get_physx_interface().subscribe_physics_step_events(self._on_physics)
        except Exception as exc:
            carb.log_warn(f"[turtlebot2] physx step subscribe failed ({exc}), using app update")
            self._update_sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
                self._on_update
            )

    def stop(self):
        self._physx_sub = None
        self._update_sub = None
        self.art = None

    def _ensure_art(self):
        if self.art is not None:
            return True
        if not omni.timeline.get_timeline_interface().is_playing():
            return False
        try:
            from isaacsim.core.experimental.prims import Articulation

            art = Articulation(self.art_root, reset_xform_op_properties=False)
            names = list(art.dof_names)
            carb.log_info(f"[turtlebot2] DOFs: {names}")
            left = next((name for name in names if "wheel_left" in name), None)
            right = next((name for name in names if "wheel_right" in name), None)
            if left and right:
                self.dof_indices = art.get_dof_indices([left, right])
            art.set_dof_gains(stiffnesses=0.0, dampings=WHEEL_DRIVE_DAMPING)
            art.set_dof_max_efforts(WHEEL_DRIVE_MAX_FORCE)
            try:
                art.set_dof_drive_types("force")
            except Exception:
                pass
            self.art = art
            self._ready_logged = True
            carb.log_info(
                f"[turtlebot2] wheel driver ready left={left} right={right} damping={WHEEL_DRIVE_DAMPING}"
            )
            return True
        except Exception as exc:
            if not self._ready_logged:
                carb.log_warn(f"[turtlebot2] articulation not ready yet: {exc}")
            return False

    def _on_physics(self, _dt):
        self._apply()

    def _on_update(self, *_args):
        if omni.timeline.get_timeline_interface().is_playing():
            self._apply()

    def _twist(self):
        try:
            lin = og.Controller.attribute(f"{GRAPH_DRIVE}/SubscribeTwist.outputs:linearVelocity").get()
            ang = og.Controller.attribute(f"{GRAPH_DRIVE}/SubscribeTwist.outputs:angularVelocity").get()
        except Exception:
            return 0.0, 0.0
        if lin is None or ang is None:
            return 0.0, 0.0
        return float(lin[0]), float(ang[2])

    def _apply(self):
        if not self._ensure_art():
            return
        vx, wz = self._twist()
        half = WHEEL_DISTANCE * 0.5
        v_left = (vx - wz * half) / WHEEL_RADIUS
        v_right = (vx + wz * half) / WHEEL_RADIUS
        try:
            if self.dof_indices is not None:
                self.art.set_dof_velocity_targets([v_left, v_right], dof_indices=self.dof_indices)
            else:
                self.art.set_dof_velocity_targets([v_left, v_right])
        except Exception as exc:
            carb.log_warn(f"[turtlebot2] set_dof_velocity_targets failed: {exc}")
            self.art = None
