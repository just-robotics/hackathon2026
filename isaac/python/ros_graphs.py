"""OmniGraph: cmd_vel, Livox PointCloud2, odom/tf, /clock."""

import carb
import omni.graph.core as og
import omni.usd
from pxr import Sdf

from constants import (
    BASE_FRAME,
    CMD_VEL_TOPIC,
    GRAPH_CLOCK,
    GRAPH_DRIVE,
    GRAPH_LIDAR,
    GRAPH_TF,
    LIDAR_FRAME,
    LIDAR_TOPIC,
    MAX_ANGULAR_SPEED,
    MAX_LINEAR_SPEED,
    ODOM_FRAME,
    ODOM_TOPIC,
    WHEEL_DISTANCE,
    WHEEL_JOINTS,
    WHEEL_RADIUS,
)


def _remove_if_exists(path):
    stage = omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath(path):
        stage.RemovePrim(path)


def _try_set(attr_path, value):
    try:
        og.Controller.attribute(attr_path).set(value)
    except Exception:
        pass


def _graph_config(path):
    cfg = {"graph_path": path, "evaluator_name": "execution"}
    stage = getattr(og, "GraphPipelineStage", None)
    if stage is not None:
        cfg["pipeline_stage"] = stage.GRAPH_PIPELINE_STAGE_SIMULATION
    return cfg


def build_drive_graph(robot_prim_path):
    _remove_if_exists(GRAPH_DRIVE)
    keys = og.Controller.Keys
    og.Controller.edit(
        _graph_config(GRAPH_DRIVE),
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("SubscribeTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                ("BreakLinVel", "omni.graph.nodes.BreakVector3"),
                ("BreakAngVel", "omni.graph.nodes.BreakVector3"),
                ("DiffController", "isaacsim.robot.wheeled_robots.DifferentialController"),
                ("ArtController", "isaacsim.core.nodes.IsaacArticulationController"),
            ],
            keys.SET_VALUES: [
                ("Context.inputs:useDomainIDEnvVar", True),
                ("SubscribeTwist.inputs:topicName", CMD_VEL_TOPIC),
                ("DiffController.inputs:maxAngularSpeed", MAX_ANGULAR_SPEED),
                ("DiffController.inputs:maxLinearSpeed", MAX_LINEAR_SPEED),
                ("DiffController.inputs:wheelDistance", WHEEL_DISTANCE),
                ("DiffController.inputs:wheelRadius", WHEEL_RADIUS),
                ("ArtController.inputs:robotPath", robot_prim_path),
                ("ArtController.inputs:jointNames", WHEEL_JOINTS),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "SubscribeTwist.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "DiffController.inputs:execIn"),
                ("OnPlaybackTick.outputs:deltaSeconds", "DiffController.inputs:dt"),
                ("OnPlaybackTick.outputs:tick", "ArtController.inputs:execIn"),
                ("SubscribeTwist.outputs:linearVelocity", "BreakLinVel.inputs:tuple"),
                ("BreakLinVel.outputs:x", "DiffController.inputs:linearVelocity"),
                ("SubscribeTwist.outputs:angularVelocity", "BreakAngVel.inputs:tuple"),
                ("BreakAngVel.outputs:z", "DiffController.inputs:angularVelocity"),
                ("DiffController.outputs:velocityCommand", "ArtController.inputs:velocityCommand"),
                ("Context.outputs:context", "SubscribeTwist.inputs:context"),
            ],
        },
    )
    carb.log_info(
        f"[turtlebot2] drive graph {GRAPH_DRIVE} topic={CMD_VEL_TOPIC} robot={robot_prim_path}"
    )


def build_lidar_graph(lidar_prim_path):
    try:
        _build_lidar_omnigraph(lidar_prim_path)
    except Exception as exc:
        carb.log_warn(f"[turtlebot2] lidar OmniGraph failed ({exc}), using Replicator writer")
        _publish_lidar_writer(lidar_prim_path)


def _build_lidar_omnigraph(lidar_prim_path):
    _remove_if_exists(GRAPH_LIDAR)
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": GRAPH_LIDAR, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("CreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("LidarHelper", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
            ],
            keys.SET_VALUES: [
                ("Context.inputs:useDomainIDEnvVar", True),
                ("CreateRenderProduct.inputs:cameraPrim", [Sdf.Path(lidar_prim_path)]),
                ("LidarHelper.inputs:topicName", LIDAR_TOPIC),
                ("LidarHelper.inputs:type", "point_cloud"),
                ("LidarHelper.inputs:frameId", LIDAR_FRAME),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "CreateRenderProduct.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "LidarHelper.inputs:execIn"),
                (
                    "CreateRenderProduct.outputs:renderProductPath",
                    "LidarHelper.inputs:renderProductPath",
                ),
                ("Context.outputs:context", "LidarHelper.inputs:context"),
            ],
        },
    )
    for name in ("fullScan", "publishFullScan", "full_scan"):
        _try_set(f"{GRAPH_LIDAR}/LidarHelper.inputs:{name}", True)
    carb.log_info(f"[turtlebot2] lidar graph {GRAPH_LIDAR} topic={LIDAR_TOPIC}")


def _publish_lidar_writer(lidar_prim_path):
    import omni.replicator.core as rep

    hydra = rep.create.render_product(lidar_prim_path, [1, 1], name="LivoxMid360")
    writer = rep.writers.get("RtxLidarROS2PublishPointCloud")
    writer.initialize(topicName=LIDAR_TOPIC, frameId=LIDAR_FRAME)
    writer.attach([hydra])
    carb.log_info(f"[turtlebot2] lidar writer topic={LIDAR_TOPIC} frame={LIDAR_FRAME}")


def build_tf_odom_graph(chassis_prim_path, livox_prim_path):
    _remove_if_exists(GRAPH_TF)
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": GRAPH_TF, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("ComputeOdom", "isaacsim.core.nodes.IsaacComputeOdometry"),
                ("PublishOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                ("PublishRawTF", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
                ("ComputeTransformTree", "isaacsim.core.nodes.IsaacComputeTransformTree"),
                ("PublishTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
            ],
            keys.SET_VALUES: [
                ("Context.inputs:useDomainIDEnvVar", True),
                ("ComputeOdom.inputs:chassisPrim", [Sdf.Path(chassis_prim_path)]),
                ("PublishOdom.inputs:topicName", ODOM_TOPIC),
                ("PublishOdom.inputs:odomFrameId", ODOM_FRAME),
                ("PublishOdom.inputs:chassisFrameId", BASE_FRAME),
                ("PublishRawTF.inputs:childFrameId", BASE_FRAME),
                ("PublishRawTF.inputs:parentFrameId", ODOM_FRAME),
                ("ComputeTransformTree.inputs:parentPrim", Sdf.Path(chassis_prim_path)),
                ("ComputeTransformTree.inputs:targetPrims", [Sdf.Path(livox_prim_path)]),
                ("PublishTF.inputs:topicName", "tf_static"),
                ("PublishTF.inputs:staticPublisher", True),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "ComputeOdom.inputs:execIn"),
                ("ComputeOdom.outputs:execOut", "PublishOdom.inputs:execIn"),
                ("ComputeOdom.outputs:angularVelocity", "PublishOdom.inputs:angularVelocity"),
                ("ComputeOdom.outputs:linearVelocity", "PublishOdom.inputs:linearVelocity"),
                ("ComputeOdom.outputs:orientation", "PublishOdom.inputs:orientation"),
                ("ComputeOdom.outputs:position", "PublishOdom.inputs:position"),
                ("ReadSimTime.outputs:simulationTime", "PublishOdom.inputs:timeStamp"),
                ("OnPlaybackTick.outputs:tick", "PublishRawTF.inputs:execIn"),
                ("ComputeOdom.outputs:orientation", "PublishRawTF.inputs:rotation"),
                ("ComputeOdom.outputs:position", "PublishRawTF.inputs:translation"),
                ("ReadSimTime.outputs:simulationTime", "PublishRawTF.inputs:timeStamp"),
                ("OnPlaybackTick.outputs:tick", "ComputeTransformTree.inputs:execIn"),
                ("ComputeTransformTree.outputs:execOut", "PublishTF.inputs:execIn"),
                ("ComputeTransformTree.outputs:childFrames", "PublishTF.inputs:childFrames"),
                ("ComputeTransformTree.outputs:orientations", "PublishTF.inputs:orientations"),
                ("ComputeTransformTree.outputs:parentFrames", "PublishTF.inputs:parentFrames"),
                ("ComputeTransformTree.outputs:translations", "PublishTF.inputs:translations"),
                ("ReadSimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
                ("Context.outputs:context", "PublishOdom.inputs:context"),
                ("Context.outputs:context", "PublishRawTF.inputs:context"),
                ("Context.outputs:context", "PublishTF.inputs:context"),
            ],
        },
    )
    carb.log_info(f"[turtlebot2] tf/odom graph {GRAPH_TF}")


def build_clock_graph():
    _remove_if_exists(GRAPH_CLOCK)
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": GRAPH_CLOCK, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            keys.SET_VALUES: [
                ("Context.inputs:useDomainIDEnvVar", True),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ("Context.outputs:context", "PublishClock.inputs:context"),
            ],
        },
    )
    carb.log_info(f"[turtlebot2] clock graph {GRAPH_CLOCK}")


def build_all(robot_prim_path, chassis_prim_path, livox_prim_path, lidar_prim_path):
    for name, fn in (
        ("drive", lambda: build_drive_graph(robot_prim_path)),
        ("lidar", lambda: build_lidar_graph(lidar_prim_path)),
        ("tf/odom", lambda: build_tf_odom_graph(chassis_prim_path, livox_prim_path)),
        ("clock", build_clock_graph),
    ):
        try:
            fn()
        except Exception as exc:
            carb.log_error(f"[turtlebot2] {name} graph failed: {exc}")
