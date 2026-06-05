"""OmniGraph wiring for /clock, /tf, /tf_static, and the head D435.

The Isaac Sim ROS 2 bridge ships OmniGraph nodes that publish on the
render thread — significantly lower CPU than equivalent rclpy publishers
for camera frames and TF. We use them for:

  * ``/clock``                          sim time
  * ``/tf`` + ``/tf_static``            articulation transforms
  * ``/camera/color/image_raw``         D435 RGB
  * ``/camera/depth/image_raw``         D435 depth
  * ``/camera/color/camera_info``       monocular intrinsics

The custom ``rm_ros_interfaces`` / ``r2d3_model_interfaces`` topics go
through the rclpy ``sim_adapter`` (Py-3.11 binding lives on the bridge
side). See ``sim_topics`` for the contract.
"""
from __future__ import annotations

import logging
from typing import Optional

from . import sim_topics as t

logger = logging.getLogger(__name__)


# D435 nominal intrinsics: 640x480, ~69 deg HFOV.  Refined later from
# the realsense2_description xacro defaults if a calibration target is
# rendered.
D435_RGB_WIDTH = 640
D435_RGB_HEIGHT = 480
D435_HFOV_DEG = 69.0

# Camera prim path the URDF→USD converter writes for the head D435.
HEAD_CAMERA_LINK = (
    "/r2d3_v1/Geometry/base_link_underpan/body_base_link/platform_base_link/"
    "head_link1/head_link2/head_camera_bottom_screw_frame/head_camera_link"
)
HEAD_CAMERA_PRIM = f"{HEAD_CAMERA_LINK}/Camera"

ROBOT_PRIM = "/r2d3_v1"


# ---------------------------------------------------------------------------
# Camera prim authoring
# ---------------------------------------------------------------------------
def _ensure_camera_prim() -> str:
    """Create a UsdGeom.Camera at ``HEAD_CAMERA_PRIM`` if it isn't there yet.

    The urdf_usd_converter authors the head_camera_link as a plain Xform
    with cube visuals — it doesn't insert a Camera prim. ROS 2 image
    publishers need an actual Camera to render through.
    """
    import omni.usd
    from pxr import Gf, UsdGeom

    stage = omni.usd.get_context().get_stage()
    if not stage.GetPrimAtPath(HEAD_CAMERA_LINK):
        raise RuntimeError(
            f"head_camera_link not found at {HEAD_CAMERA_LINK} — has the "
            f"USD been re-rendered with the head D435 macro?"
        )

    if stage.GetPrimAtPath(HEAD_CAMERA_PRIM):
        logger.info("camera prim already exists at %s", HEAD_CAMERA_PRIM)
        return HEAD_CAMERA_PRIM

    cam = UsdGeom.Camera.Define(stage, HEAD_CAMERA_PRIM)
    # D435 RGB intrinsics expressed via the standard pinhole knobs.
    # Pair an 18.8 mm focal length with a 36 mm horizontal aperture to get
    # ~69 deg HFOV (the real D435 RGB).
    cam.GetFocalLengthAttr().Set(18.8)
    cam.GetHorizontalApertureAttr().Set(36.0)
    cam.GetVerticalApertureAttr().Set(27.0)
    cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.05, 100.0))

    # A USD camera looks down its local -Z with up = +Y. The head_camera_link
    # frame has the robot's forward along +X and up along +Z. We need the
    # camera's view direction (-Z_cam) to point along +X_link and the camera
    # up (+Y_cam) along +Z_link. The rotation whose columns are the camera
    # basis expressed in the link frame is
    #     X_cam = (0,-1, 0)   Y_cam = (0, 0, 1)   Z_cam = (-1, 0, 0)
    # which yields the quaternion (w,x,y,z) = (0.5, 0.5, -0.5, -0.5).
    # (head_joint2's tilt is applied above this prim, so commanding tilt
    # pitches the camera down to see the workspace.)
    xform = UsdGeom.Xformable(cam)
    xform.ClearXformOpOrder()
    xform.AddOrientOp().Set(Gf.Quatf(0.5, Gf.Vec3f(0.5, -0.5, -0.5)))
    logger.info("authored Camera at %s (forward = link +X)", HEAD_CAMERA_PRIM)
    return HEAD_CAMERA_PRIM


# ---------------------------------------------------------------------------
# OmniGraph wiring
# ---------------------------------------------------------------------------
ACTION_GRAPH_PATH = "/R2D3_ROS_Graph"


# ---------------------------------------------------------------------------
# Free look-at camera authoring (used by the capture tool for the
# third-person hero shot — a world-space camera framing the whole robot).
# ---------------------------------------------------------------------------
def _quat_wxyz_from_matrix(R):
    """Standard rotation-matrix -> quaternion (w, x, y, z).

    R has COLUMNS equal to the camera-local axes expressed in world, i.e.
    the standard rotation mapping local vectors to world (v_world = R @ v_local).
    USD's xformOp:orient takes exactly this quaternion.
    """
    import numpy as np

    m00, m01, m02 = R[0, 0], R[0, 1], R[0, 2]
    m10, m11, m12 = R[1, 0], R[1, 1], R[1, 2]
    m20, m21, m22 = R[2, 0], R[2, 1], R[2, 2]
    tr = m00 + m11 + m22
    if tr > 0.0:
        s = np.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (m21 - m12) / s
        y = (m02 - m20) / s
        z = (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = np.sqrt(1.0 + m00 - m11 - m22) * 2.0
        w = (m21 - m12) / s
        x = 0.25 * s
        y = (m01 + m10) / s
        z = (m02 + m20) / s
    elif m11 > m22:
        s = np.sqrt(1.0 + m11 - m00 - m22) * 2.0
        w = (m02 - m20) / s
        x = (m01 + m10) / s
        y = 0.25 * s
        z = (m12 + m21) / s
    else:
        s = np.sqrt(1.0 + m22 - m00 - m11) * 2.0
        w = (m10 - m01) / s
        x = (m02 + m20) / s
        y = (m12 + m21) / s
        z = 0.25 * s
    return float(w), float(x), float(y), float(z)


def author_lookat_camera(
    prim_path: str,
    eye,
    target,
    up=(0.0, 0.0, 1.0),
    focal_length: float = 24.0,
    h_aperture: float = 36.0,
    v_aperture: float = 20.25,   # 36 * 720/1280 -> square pixels at 16:9
) -> str:
    """Author a world-space UsdGeom.Camera at ``eye`` looking at ``target``.

    Default focal/aperture give ~73 deg HFOV / ~45 deg VFOV — wide enough
    to frame the full robot at a modest distance without much distortion.
    """
    import numpy as np
    import omni.usd
    from pxr import Gf, UsdGeom

    eye = np.asarray(eye, dtype=float)
    target = np.asarray(target, dtype=float)
    up = np.asarray(up, dtype=float)

    f = target - eye
    f /= np.linalg.norm(f)
    Z = -f                                   # camera looks down -Z
    X = np.cross(up, Z); X /= np.linalg.norm(X)
    Y = np.cross(Z, X)
    R = np.column_stack([X, Y, Z])
    w, x, y, z = _quat_wxyz_from_matrix(R)

    stage = omni.usd.get_context().get_stage()
    cam = UsdGeom.Camera.Define(stage, prim_path)
    cam.CreateFocalLengthAttr(float(focal_length))
    cam.CreateHorizontalApertureAttr(float(h_aperture))
    cam.CreateVerticalApertureAttr(float(v_aperture))
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 1000.0))

    xf = UsdGeom.Xformable(cam)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(float(eye[0]), float(eye[1]), float(eye[2])))
    xf.AddOrientOp().Set(Gf.Quatf(w, Gf.Vec3f(x, y, z)))
    logger.info("authored look-at camera %s at %s -> %s", prim_path, eye, target)
    return prim_path


def camera_world_pose(prim_path: str):
    """Return (position np[3], forward np[3]) of a camera prim in world.

    Forward is the camera's view direction (-Z of the camera, in world).
    """
    import numpy as np
    import omni.usd
    from pxr import UsdGeom

    from pxr import Gf

    stage = omni.usd.get_context().get_stage()
    cache = UsdGeom.XformCache()
    m = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(prim_path))
    # Bulletproof against row/column convention: transform two points and
    # subtract. A USD camera's view direction is its local -Z.
    p0 = m.Transform(Gf.Vec3d(0.0, 0.0, 0.0))
    p1 = m.Transform(Gf.Vec3d(0.0, 0.0, -1.0))
    pos = np.asarray([p0[0], p0[1], p0[2]], dtype=float)
    forward = np.asarray([p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]], dtype=float)
    forward /= np.linalg.norm(forward)
    return pos, forward


def build_action_graph(
    *,
    publish_clock: bool = True,
    publish_tf: bool = True,
    publish_camera: bool = True,
    camera_resolution: tuple[int, int] = (D435_RGB_WIDTH, D435_RGB_HEIGHT),
) -> None:
    """Author the OmniGraph action graph that publishes sim-side topics.

    Must be called *after* world.reset() and *before* the sim loop's first
    ``world.step()``. Idempotent: rebuilds the graph on every call.
    """
    import omni.graph.core as og

    camera_prim_path: Optional[str] = None
    render_product_path: Optional[str] = None

    if publish_camera:
        camera_prim_path = _ensure_camera_prim()
        # Render product = the per-camera viewport pipeline that produces
        # RGB / depth annotators. omni.replicator.core owns these.
        import omni.replicator.core as rep
        render_product = rep.create.render_product(
            camera_prim_path, camera_resolution
        )
        # rep.create.render_product returns a Replicator product object;
        # the OmniGraph nodes want its USD prim path.
        render_product_path = render_product.path
        logger.info("camera render product: %s @ %s", render_product_path,
                    camera_resolution)

    create_nodes = [
        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
        ("ReadSimTime",    "isaacsim.core.nodes.IsaacReadSimulationTime"),
        ("Ros2Context",    "isaacsim.ros2.bridge.ROS2Context"),
    ]
    connections = []
    set_values = []

    if publish_clock:
        create_nodes.append(
            ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock")
        )
        connections += [
            ("OnPlaybackTick.outputs:tick",        "PublishClock.inputs:execIn"),
            ("Ros2Context.outputs:context",        "PublishClock.inputs:context"),
            ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
        ]
        set_values.append(("PublishClock.inputs:topicName", t.CLOCK))

    if publish_tf:
        import usdrt.Sdf as sdf
        create_nodes.append(
            ("PublishTf", "isaacsim.ros2.bridge.ROS2PublishTransformTree")
        )
        connections += [
            ("OnPlaybackTick.outputs:tick",        "PublishTf.inputs:execIn"),
            ("Ros2Context.outputs:context",        "PublishTf.inputs:context"),
            ("ReadSimTime.outputs:simulationTime", "PublishTf.inputs:timeStamp"),
        ]
        set_values += [
            ("PublishTf.inputs:topicName",   t.TF),
            ("PublishTf.inputs:targetPrims", [sdf.Path(ROBOT_PRIM)]),
        ]

    if publish_camera and render_product_path is not None:
        create_nodes += [
            ("CameraRgb",  "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ("CameraDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ("CameraInfoColor", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ("CameraInfoDepth", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
        ]
        connections += [
            ("OnPlaybackTick.outputs:tick", "CameraRgb.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "CameraDepth.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "CameraInfoColor.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "CameraInfoDepth.inputs:execIn"),
            ("Ros2Context.outputs:context", "CameraRgb.inputs:context"),
            ("Ros2Context.outputs:context", "CameraDepth.inputs:context"),
            ("Ros2Context.outputs:context", "CameraInfoColor.inputs:context"),
            ("Ros2Context.outputs:context", "CameraInfoDepth.inputs:context"),
        ]
        set_values += [
            # RGB
            ("CameraRgb.inputs:renderProductPath", render_product_path),
            ("CameraRgb.inputs:topicName",         t.CAMERA_COLOR_IMAGE),
            ("CameraRgb.inputs:type",              "rgb"),
            ("CameraRgb.inputs:frameId",           t.CAMERA_COLOR_OPT_FRAME),
            # Depth
            ("CameraDepth.inputs:renderProductPath", render_product_path),
            ("CameraDepth.inputs:topicName",         t.CAMERA_DEPTH_IMAGE),
            ("CameraDepth.inputs:type",              "depth"),
            ("CameraDepth.inputs:frameId",           t.CAMERA_DEPTH_OPT_FRAME),
            # CameraInfo — color
            ("CameraInfoColor.inputs:renderProductPath", render_product_path),
            ("CameraInfoColor.inputs:topicName",         t.CAMERA_COLOR_INFO),
            ("CameraInfoColor.inputs:frameId",           t.CAMERA_COLOR_OPT_FRAME),
            # CameraInfo — depth (same intrinsics for V1; the D435 aligns
            # depth to color, so the depth optical frame shares the K matrix)
            ("CameraInfoDepth.inputs:renderProductPath", render_product_path),
            ("CameraInfoDepth.inputs:topicName",         t.CAMERA_DEPTH_INFO),
            ("CameraInfoDepth.inputs:frameId",           t.CAMERA_DEPTH_OPT_FRAME),
        ]

    og.Controller.edit(
        {"graph_path": ACTION_GRAPH_PATH, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: create_nodes,
            og.Controller.Keys.CONNECT:      connections,
            og.Controller.Keys.SET_VALUES:   set_values,
        },
    )
    logger.info(
        "built OmniGraph at %s: clock=%s tf=%s camera=%s",
        ACTION_GRAPH_PATH, publish_clock, publish_tf, publish_camera,
    )
