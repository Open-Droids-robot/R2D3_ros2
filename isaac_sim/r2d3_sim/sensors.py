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

# Forward offset (m) of the head camera LENS only (invisible UsdGeom.Camera),
# NOT the camera body. The head camera is now co-located with the upstream
# `camera_link` D435, which sits recessed ~0.08 m inside the head_link2 shroud —
# a lens there images the inside of the housing (black). 0.10 m pushes the lens
# just past the head's front face so it sees forward, while the visible camera
# body (camera_link mesh) stays at its real recessed mount. No visible float
# (the offset prim has no geometry).
HEAD_CAMERA_FWD_OFFSET_M = 0.10

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
# All cameras: head D435 (at the upstream camera_link, lens pushed forward to
# clear the head shroud) + a wrist D435 per arm. Each entry drives both the
# Camera-prim authoring and the OmniGraph ROS publishers. Wrist cameras keep
# their body box visible (it IS the visible wrist camera); only the head hides
# its box (the visible head camera is camera_link's mesh).
CAMERAS = (
    dict(name="head", link="head_camera_link", fwd=HEAD_CAMERA_FWD_OFFSET_M, hide_box=True,
         color=t.CAMERA_COLOR_IMAGE, depth=t.CAMERA_DEPTH_IMAGE,
         color_info=t.CAMERA_COLOR_INFO, depth_info=t.CAMERA_DEPTH_INFO,
         color_frame=t.CAMERA_COLOR_OPT_FRAME, depth_frame=t.CAMERA_DEPTH_OPT_FRAME),
    dict(name="l_wrist", link="l_wrist_camera_link", fwd=0.0, hide_box=False,
         color=t.L_WRIST_COLOR_IMAGE, depth=t.L_WRIST_DEPTH_IMAGE,
         color_info=t.L_WRIST_COLOR_INFO, depth_info=t.L_WRIST_DEPTH_INFO,
         color_frame=t.L_WRIST_COLOR_OPT_FRAME, depth_frame=t.L_WRIST_DEPTH_OPT_FRAME),
    dict(name="r_wrist", link="r_wrist_camera_link", fwd=0.0, hide_box=False,
         color=t.R_WRIST_COLOR_IMAGE, depth=t.R_WRIST_DEPTH_IMAGE,
         color_info=t.R_WRIST_COLOR_INFO, depth_info=t.R_WRIST_DEPTH_INFO,
         color_frame=t.R_WRIST_COLOR_OPT_FRAME, depth_frame=t.R_WRIST_DEPTH_OPT_FRAME),
)


def _find_link_path(name: str):
    import omni.usd
    stage = omni.usd.get_context().get_stage()
    for p in stage.Traverse():
        if p.GetName() == name and p.GetTypeName() == "Xform":
            return p.GetPath().pathString
    return None


def _make_camera_prim(link_path: str, fwd: float = 0.0, hide_box: bool = False) -> str:
    """Create a D435 UsdGeom.Camera under ``link_path`` (looks down link +X).

    The converter writes camera links as plain Xforms with cube visuals — ROS 2
    image publishers need an actual Camera prim to render through. ``fwd`` pushes
    the (invisible) lens forward along link +X (used for the head, whose
    camera_link is recessed inside the shroud). ``hide_box`` makes the D435 body
    box non-rendering (head only — its visible body is camera_link's own mesh).
    """
    import omni.usd
    from pxr import Gf, UsdGeom

    stage = omni.usd.get_context().get_stage()
    if hide_box:
        box = stage.GetPrimAtPath(f"{link_path}/box")
        if box:
            UsdGeom.Imageable(box).MakeInvisible()
    prim_path = f"{link_path}/Camera"
    if stage.GetPrimAtPath(prim_path):
        return prim_path

    cam = UsdGeom.Camera.Define(stage, prim_path)
    # D435 RGB intrinsics: focal 26 mm + 36 mm h-aperture -> 69.4 deg HFOV (the
    # real D435 *color* FOV); 27 mm v-aperture keeps 4:3 square pixels.
    cam.GetFocalLengthAttr().Set(26.0)
    cam.GetHorizontalApertureAttr().Set(36.0)
    cam.GetVerticalApertureAttr().Set(27.0)
    cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.02, 100.0))
    # USD camera looks down -Z (up +Y); aim its view (-Z_cam) along link +X (the
    # D435 forward, set per-camera by the mount rpy) with up along link +Z:
    #   X_cam=(0,-1,0) Y_cam=(0,0,1) Z_cam=(-1,0,0) -> quat (0.5,0.5,-0.5,-0.5).
    xform = UsdGeom.Xformable(cam)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(fwd, 0.0, 0.0))
    xform.AddOrientOp().Set(Gf.Quatf(0.5, Gf.Vec3f(0.5, -0.5, -0.5)))
    logger.info("authored Camera at %s (lens +%.2f m along link +X)", prim_path, fwd)
    return prim_path


def ensure_camera_prims() -> dict:
    """Author Camera prims for every entry in CAMERAS. Returns {name: prim_path}
    for the cameras whose link exists in the stage (skips missing ones)."""
    out = {}
    for c in CAMERAS:
        lp = _find_link_path(c["link"])
        if lp is None:
            logger.warning("camera link %s not in stage — skipping %s", c["link"], c["name"])
            continue
        out[c["name"]] = _make_camera_prim(lp, c["fwd"], c["hide_box"])
    return out


def _ensure_camera_prim() -> str:
    """Back-compat: author + return the HEAD camera prim only."""
    return _make_camera_prim(HEAD_CAMERA_LINK, HEAD_CAMERA_FWD_OFFSET_M, hide_box=True)


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

    # One render product per camera (head + l/r wrist).
    camera_products = []  # list of (camera_dict, render_product_path)
    if publish_camera:
        import omni.replicator.core as rep
        prims = ensure_camera_prims()
        for c in CAMERAS:
            ppath = prims.get(c["name"])
            if not ppath:
                continue
            rprod = rep.create.render_product(ppath, camera_resolution)
            camera_products.append((c, rprod.path))
        logger.info("camera render products: %s @ %s",
                    [c["name"] for c, _ in camera_products], camera_resolution)

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

    for c, rpath in camera_products:
        nm = c["name"]
        rgb, dep = f"Rgb_{nm}", f"Depth_{nm}"
        ic, idp = f"InfoColor_{nm}", f"InfoDepth_{nm}"
        create_nodes += [
            (rgb, "isaacsim.ros2.bridge.ROS2CameraHelper"),
            (dep, "isaacsim.ros2.bridge.ROS2CameraHelper"),
            (ic,  "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            (idp, "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
        ]
        for node in (rgb, dep, ic, idp):
            connections += [
                ("OnPlaybackTick.outputs:tick", f"{node}.inputs:execIn"),
                ("Ros2Context.outputs:context", f"{node}.inputs:context"),
            ]
        set_values += [
            (f"{rgb}.inputs:renderProductPath", rpath),
            (f"{rgb}.inputs:topicName",         c["color"]),
            (f"{rgb}.inputs:type",              "rgb"),
            (f"{rgb}.inputs:frameId",           c["color_frame"]),
            (f"{dep}.inputs:renderProductPath", rpath),
            (f"{dep}.inputs:topicName",         c["depth"]),
            (f"{dep}.inputs:type",              "depth"),
            (f"{dep}.inputs:frameId",           c["depth_frame"]),
            (f"{ic}.inputs:renderProductPath",  rpath),
            (f"{ic}.inputs:topicName",          c["color_info"]),
            (f"{ic}.inputs:frameId",            c["color_frame"]),
            # depth shares the color K matrix (D435 aligns depth to color in V1)
            (f"{idp}.inputs:renderProductPath", rpath),
            (f"{idp}.inputs:topicName",         c["depth_info"]),
            (f"{idp}.inputs:frameId",           c["depth_frame"]),
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
