"""Kit extension: after AppReady, spawn TurtleBot 2 and start playback."""

import asyncio
import os
import sys
import traceback

import carb
import omni.ext
import omni.kit.app
import omni.timeline
import omni.usd


ROOM_SETTING = "/exts/isaacsim.hackathon.turtlebot/room"
_TRUTHY = {"1", "true", "yes", "on", "y"}
_FALSY = {"0", "false", "no", "off", "n"}


def _room_enabled():
    """Грузить ли Simple Room: env HACKATHON_ROOM, иначе Kit-настройка.

    Робот, лидар и ROS-графы строятся в любом случае: при false вместо комнаты
    под робота кладётся плоскость с землёй.
    """
    raw = os.environ.get("HACKATHON_ROOM")
    if raw is not None:
        value = raw.strip().lower()
        if value in _TRUTHY:
            return True
        if value in _FALSY:
            return False
        carb.log_warn(f"[turtlebot2] HACKATHON_ROOM={raw!r} не распознан, беру Kit-настройку")
    setting = carb.settings.get_settings().get(ROOM_SETTING)
    return True if setting is None else bool(setting)


def _isaac_root():
    env = os.environ.get("HACKATHON_ISAAC_ROOT")
    if env:
        return env
    ext_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    return os.path.abspath(os.path.join(ext_root, "..", ".."))


def _ensure_python_path():
    root = _isaac_root()
    python_dir = os.path.join(root, "python")
    if python_dir not in sys.path:
        sys.path.insert(0, python_dir)
    os.environ.setdefault("HACKATHON_ISAAC_ROOT", root)


class TurtleBotExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        self._ext_id = ext_id
        self._task = None
        self._wheel_driver = None
        _ensure_python_path()
        try:
            carb.settings.get_settings().set("/rtx/post/dlss/execMode", 1)
        except Exception:
            pass
        carb.log_info("[turtlebot2] extension startup")
        self._task = asyncio.ensure_future(self._setup())

    def on_shutdown(self):
        if self._task and not self._task.done():
            self._task.cancel()
        self._task = None
        if self._wheel_driver is not None:
            self._wheel_driver.stop()
            self._wheel_driver = None

    async def _wait_until_ready(self):
        app = omni.kit.app.get_app()
        manager = app.get_extension_manager()
        for i in range(6000):
            await app.next_update_async()
            try:
                if not manager.is_extension_enabled("isaacsim.ros2.bridge"):
                    manager.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
            except Exception:
                pass
            stage_ok = omni.usd.get_context().get_stage() is not None
            if stage_ok and i >= 60:
                carb.log_info("[turtlebot2] app ready, building scene")
                return
        carb.log_warn("[turtlebot2] timed out waiting for AppReady, building scene anyway")

    async def _setup(self):
        try:
            await self._wait_until_ready()
            await self._build_scene()
        except Exception as exc:
            carb.log_error(f"[turtlebot2] scene setup failed: {exc}")
            carb.log_error(traceback.format_exc())

    async def _build_scene(self):
        _ensure_python_path()
        from drive import WheelDriver
        from mid360 import attach_mid360
        from physics import make_robot_dynamic, tune_wheel_caster_friction
        from robot import add_robot_to_stage, ensure_robot_usd, find_articulation_root, find_link
        from ros_graphs import build_all
        from scene import load_simple_room

        app = omni.kit.app.get_app()
        usd_path = ensure_robot_usd()
        floor_z = await load_simple_room(use_room=_room_enabled())
        await app.next_update_async()

        robot_prim = add_robot_to_stage(usd_path, floor_z=floor_z)
        for _ in range(10):
            await app.next_update_async()

        make_robot_dynamic(robot_prim)
        art_root = find_articulation_root(robot_prim)
        chassis = find_link(robot_prim, "base_link") or art_root
        livox = find_link(robot_prim, "livox_frame")
        carb.log_info(f"[turtlebot2] art_root={art_root} chassis={chassis} livox={livox}")
        if not livox:
            names = []
            stage = omni.usd.get_context().get_stage()
            if stage:
                for prim in stage.Traverse():
                    path = str(prim.GetPath())
                    if path.startswith(robot_prim):
                        names.append(path)
            carb.log_error("[turtlebot2] prims under robot: " + ", ".join(names[:80]))
            raise RuntimeError("livox_frame link was not imported")

        tune_wheel_caster_friction(robot_prim)
        lidar_prim = attach_mid360(livox)
        await app.next_update_async()

        build_all(art_root, chassis, livox, lidar_prim)
        await app.next_update_async()

        omni.timeline.get_timeline_interface().play()
        self._wheel_driver = WheelDriver(art_root)
        self._wheel_driver.start()
        carb.log_info("[turtlebot2] scene ready, timeline playing")
        await self._report_settling(chassis)

    async def _report_settling(self, prim_path):
        from pxr import UsdGeom

        app = omni.kit.app.get_app()
        for _ in range(3):
            for _ in range(120):
                await app.next_update_async()
            prim = omni.usd.get_context().get_stage().GetPrimAtPath(prim_path)
            if not prim or not prim.IsValid():
                return
            pose = UsdGeom.XformCache().GetLocalToWorldTransform(prim)
            translation = pose.ExtractTranslation()
            carb.log_info(
                f"[turtlebot2] {prim_path} world z={translation[2]:.4f} "
                f"xy=({translation[0]:.3f}, {translation[1]:.3f})"
            )
