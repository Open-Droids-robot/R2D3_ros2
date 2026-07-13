#!/usr/bin/env python3
"""
Cached URDF->MJCF conversion for the R2D3 MuJoCo simulation.

Hashes the xacro-generated URDF plus the world file. On a cache hit the
previously generated MJCF is published directly (skipping the expensive mesh
conversion); on a miss the upstream converter is invoked (--publish_topic pointed at
a throwaway internal topic -- see build_converter_cmd() -- so its own publish is
harmless) to save the MJCF and assets into the cache directory, this script raises the
lidar scan plane above the chassis so the rangefinder rays clear the robot body (see
raise_lidar_scan_plane()), and only then publishes the patched model on the real
--topic itself -- this script is always the one true publisher on that topic, so a
cache-miss run never lets a subscriber see the unpatched converter output.

The MJCF is published on --topic with RELIABLE + TRANSIENT_LOCAL QoS, matching
the subscriber inside mujoco_ros2_control.
"""

import argparse
import hashlib
import math
import os
import re
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
from xml.dom import minidom
from xml.parsers.expat import ExpatError

# The converter subprocess is launched with --publish_topic (see build_converter_cmd()),
# which per (see git history: git show aaae018:r2d3_mujoco/SPIKE_NOTES.md) makes it spin
# forever (rclpy.spin()) after writing its output files instead of exiting -- so this
# script cannot wait() on it for completion. Instead it polls the output MJCF for having
# stopped changing size, the same file-stability signal (see git history:
# git show aaae018:r2d3_mujoco/SPIKE_NOTES.md) used manually, then kills the whole
# process group.
CONVERT_POLL_INTERVAL_S = 0.5
CONVERT_STABLE_FOR_S = 2.0
CONVERT_TIMEOUT_S = 300.0

MJCF_FILENAME = "mujoco_description_formatted.xml"
CHECKSUM_FILENAME = "checksum"
URDF_FILENAME = "robot_input.urdf"
# Bump when converter flags in build_converter_cmd change, or when
# raise_lidar_scan_plane()'s patch logic changes, to invalidate caches.
CONVERTER_ARGS_VERSION = "v6:save_only,add_free_joint,scene,lidar_scan_raise,wheel_primitives,base_inertial"

# --- Lidar scan-height fix ----------------------------------------------------------
#
# The lidar is mounted low, inside the chassis body envelope: the rangefinder site sits
# at lidar_link (world ~0.24, 0, 0.233), and the base_link_underpan / body_base_link
# meshes surround it, so every ray leaves the site and immediately re-intersects the
# chassis a few cm out -- every /scan range comes back below range_min and is reported
# as -1.0. (MuJoCo's rangefinder only excludes the site's OWN body from ray casting,
# and the converter puts the rays' site in a dedicated geometry-less
# "<site>_lidar_body", so that exclusion never covers the chassis; geomgroup filtering
# does not apply to rangefinders either -- they call mj_ray with geomgroup=NULL.)
#
# An earlier fix hid the chassis meshes from ray casting by zeroing their rgba alpha
# (the one filter ray casting honors -- ray_eliminate() in engine_ray.c). That worked
# but also hid the chassis from the renderer (alpha is a rendering property too), so the
# robot looked gutted. Instead we now RAISE the horizontal scan plane above the chassis:
# the converter's rangefinder body (lidar_link_lidar_body) carries the sensor pos in its
# `pos` attribute and the ray orientation in its `quat`, so bumping its Z lifts the whole
# scan plane while keeping the rays horizontal. Verified in standalone MuJoCo: at the
# nominal height every ray self-hits the body (<0.2 m); raised by 0.15 m all 240 rays
# clear the chassis and read the world walls (~2-7 m) with the full body left visible.
#
# NOTE: this is a temporary workaround for a lidar_link that is currently placed inside
# the chassis material rather than in the chassis's lidar opening. The mount pose is to
# be corrected upstream (measured on the real robot); once lidar_link sits in the
# opening this raise can be reduced to 0.
LIDAR_SCAN_RAISE_M = 0.15
_LIDAR_BODY_NAME = "lidar_link_lidar_body"


def raise_lidar_scan_plane(mjcf_path: Path) -> int:
    """Raise the lidar rangefinder body's Z by LIDAR_SCAN_RAISE_M so the horizontal scan
    plane clears the chassis, leaving the chassis fully visible to the renderer/camera.

    Returns the number of lidar bodies patched (expected: 1). Uses a targeted text
    substitution (not a full XML re-serialization) so the rest of the generated file --
    comments, attribute order, formatting -- is left untouched.
    """
    text = mjcf_path.read_text()
    patched_count = 0

    def _raise_pos(pos_match: "re.Match") -> str:
        x, y, z = pos_match.group(1).split()
        new_z = round(float(z) + LIDAR_SCAN_RAISE_M, 6)
        return f'pos="{x} {y} {new_z}"'

    def _patch_body(body_match: "re.Match") -> str:
        nonlocal patched_count
        tag, n = re.subn(r'pos="([^"]+)"', _raise_pos, body_match.group(0), count=1)
        patched_count += n
        return tag

    pattern = re.compile(r'<body name="' + re.escape(_LIDAR_BODY_NAME) + r'"[^>]*>')
    patched_text = pattern.sub(_patch_body, text)
    if patched_count:
        mjcf_path.write_text(patched_text)
    return patched_count


# Exactly one lidar rangefinder body is generated, so exactly one Z should be raised.
EXPECTED_PATCH_COUNT = 1


def validate_patch_count(patched: int) -> bool:
    """True iff the lidar scan-height patch matched exactly the expected body.

    Anything else means the converter's output no longer matches the patch pattern
    (upstream format change, renamed lidar body, ...) and the model would ship with the
    all -1.0 /scan bug -- the caller must treat it as fatal.
    """
    return patched == EXPECTED_PATCH_COUNT


# --- Wheel collision primitives -----------------------------------------------------
#
# The R2D3 description is visual-only, so the converter synthesizes every collision geom
# as an unnamed convex HULL of the visual mesh. For the wheels this is wrong two ways:
#   1. A hull is a faceted polyhedron, not a smooth surface -- as a caster swivels its
#      ground-contact point jumps between facets, so the contact depth varies with
#      orientation (measured: one caster penetrated 19.5 mm while another floated +2 mm).
#      That inconsistency makes the base rock/wobble and the robot drive jerkily.
#   2. The hulls are not coplanar: the caster hull bottoms sit ~7-12 mm BELOW the drive
#      wheel hull bottoms, so the robot rests on the casters and the drive wheels float
#      off the ground (poor traction, nose-down tip onto the front skirt).
#
# Fix: replace the wheel collision hulls with smooth primitives, coplanar at the base
# frame's z=0 (== the ground plane). This mirrors the Gazebo model, which uses sphere
# collision for the casters (dual_rm_simulation/urdf/gazebo/sim_gazebo.urdf.xacro).
# Each wheel body's origin is its axle, so a primitive of radius == the axle's height
# above base_footprint puts its bottom at z=0; the drive-wheel radius that does this is
# also exactly the diff_drive_controller wheel_radius (0.08), so odometry stays correct.
#
# NOTE: these radii/half-width are measured from the CURRENT mesh geometry. If the wheel
# link poses change upstream (the lidar/wheel geometry is being re-measured on the real
# robot), re-derive them.
_DRIVE_WHEEL_BODIES = ("link_left_wheel", "link_right_wheel")
_CASTER_WHEEL_BODIES = (
    "link_swivel_wheel_1_2", "link_swivel_wheel_2_2",
    "link_swivel_wheel_3_2", "link_swivel_wheel_4_2",
)
# The caster swivel brackets (forks). Their convex-hull collision hangs low enough to
# drag on the ground during motion (measured: 1000+ ground-contact frames each while
# driving), so their collision is disabled entirely -- only the wheels should touch the
# floor. No replacement primitive: the brackets never contact the ground in reality.
_CASTER_BRACKET_BODIES = (
    "link_swivel_wheel_1_1", "link_swivel_wheel_2_1",
    "link_swivel_wheel_3_1", "link_swivel_wheel_4_1",
)
DRIVE_WHEEL_RADIUS = 0.08        # = diff_drive wheel_radius = drive axle height -> bottom z=0
DRIVE_WHEEL_HALF_WIDTH = 0.055   # half the drive wheel's axial extent
CASTER_WHEEL_RADIUS = 0.0253     # = caster axle height -> bottom coplanar with drive at z=0

# 2 drive wheels + 4 casters each get a primitive geom AND their hull disabled (2 patches
# each); the 4 brackets only get their hull disabled (1 patch each).
EXPECTED_WHEEL_PATCH_COUNT = (
    2 * (len(_DRIVE_WHEEL_BODIES) + len(_CASTER_WHEEL_BODIES))
    + len(_CASTER_BRACKET_BODIES)
)


def _add_primitive_and_disable_hull(text: str, body: str, primitive: str) -> "tuple[str, int]":
    """Insert `primitive` as the body's first geom and disable the body's convex-hull
    collision geom (contype=0 -> collides with nothing; conaffinity is already 0 via the
    collision default class). Returns (new_text, patches_applied) where a full success is
    2 (primitive inserted + hull disabled)."""
    patches = 0

    new_text, n_open = re.subn(
        r'(<body name="' + re.escape(body) + r'"[^>]*>)',
        lambda m: m.group(1) + primitive,
        text,
        count=1,
    )
    patches += n_open

    # Disable the hull collision geom for this body (mesh="<body>" + class="collision").
    new_text, n_hull = re.subn(
        r'(<geom type="mesh" mesh="' + re.escape(body) + r'" class="collision")(/>)',
        r'\1 contype="0"\2',
        new_text,
        count=1,
    )
    patches += n_hull
    return new_text, patches


def replace_wheel_collision_with_primitives(mjcf_path: Path) -> int:
    """Swap the wheels' faceted convex-hull collision for smooth, coplanar primitives:
    cylinders for the drive wheels (axis along the spin axis == body-local X) and spheres
    for the casters. Returns the number of patches applied
    (expected: EXPECTED_WHEEL_PATCH_COUNT)."""
    text = mjcf_path.read_text()
    total = 0

    cylinder = (
        f'<geom type="cylinder" '
        f'fromto="{-DRIVE_WHEEL_HALF_WIDTH} 0 0 {DRIVE_WHEEL_HALF_WIDTH} 0 0" '
        f'size="{DRIVE_WHEEL_RADIUS}" class="collision"/>'
    )
    for body in _DRIVE_WHEEL_BODIES:
        text, patches = _add_primitive_and_disable_hull(text, body, cylinder)
        total += patches

    sphere = f'<geom type="sphere" size="{CASTER_WHEEL_RADIUS}" class="collision"/>'
    for body in _CASTER_WHEEL_BODIES:
        text, patches = _add_primitive_and_disable_hull(text, body, sphere)
        total += patches

    for body in _CASTER_BRACKET_BODIES:
        text, n = re.subn(
            r'(<geom type="mesh" mesh="' + re.escape(body) + r'" class="collision")(/>)',
            r'\1 contype="0"\2',
            text,
            count=1,
        )
        total += n

    if total:
        mjcf_path.write_text(text)
    return total


def validate_wheel_patch_count(patched: int) -> bool:
    """True iff every wheel got both its primitive and its hull disabled."""
    return patched == EXPECTED_WHEEL_PATCH_COUNT


# --- base_footprint inertial --------------------------------------------------------
#
# The converter merges every fixed-jointed child (base_link_underpan, body_base_link,
# lidar_link, imu_link, ...) into the root base_footprint body but DROPS their <inertial>
# elements, and base_footprint has none of its own (it is a virtual Nav2 frame). MuJoCo
# then synthesizes the body's mass from its geoms -- i.e. from the bloated chassis convex
# HULL volume at the default 1000 kg/m^3 (water) density -- yielding ~962 kg instead of
# the real ~15 kg. That crushing phantom mass sags the soft contacts, drops the chassis
# onto its skirt, and makes the base tip/wobble and drive jerkily. We recompute the true
# combined inertial from the URDF and inject it so MuJoCo uses it instead of the geoms.
_MERGED_BASE_BODY = "base_footprint"


def compute_merged_base_inertial(urdf_str: str):
    """Combine the inertials of base_footprint and every link rigidly (fixed-joint)
    attached to it, expressed in the base_footprint frame. Returns
    (mass, (cx, cy, cz), (ixx, iyy, izz, ixy, ixz, iyz)) or None if base_footprint has no
    rigidly-attached mass in the URDF."""
    import numpy as np

    dom = minidom.parseString(urdf_str)

    def rpy_to_R(r, p, y):
        cr, sr = math.cos(r), math.sin(r)
        cp, sp = math.cos(p), math.sin(p)
        cy, sy = math.cos(y), math.sin(y)
        return np.array([
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ])

    def origin(el):
        o = el.getElementsByTagName("origin")
        if not o:
            return np.zeros(3), np.eye(3)
        o = o[0]
        xyz = ([float(v) for v in o.getAttribute("xyz").split()]
               if o.getAttribute("xyz").strip() else [0, 0, 0])
        rpy = ([float(v) for v in o.getAttribute("rpy").split()]
               if o.getAttribute("rpy").strip() else [0, 0, 0])
        return np.array(xyz), rpy_to_R(*rpy)

    joints = {}  # child -> (parent, xyz, R, type)
    for j in dom.getElementsByTagName("joint"):
        parent = j.getElementsByTagName("parent")
        child = j.getElementsByTagName("child")
        if not parent or not child:
            continue
        xyz, R = origin(j)
        joints[child[0].getAttribute("link")] = (
            parent[0].getAttribute("link"), xyz, R, j.getAttribute("type"),
        )

    links = {}  # name -> (mass, com_xyz, com_R, I_3x3)
    for link in dom.getElementsByTagName("link"):
        inertial = link.getElementsByTagName("inertial")
        if not inertial:
            continue
        inertial = inertial[0]
        mass = float(inertial.getElementsByTagName("mass")[0].getAttribute("value"))
        xyz, R = origin(inertial)
        I_el = inertial.getElementsByTagName("inertia")[0]
        g = lambda a: float(I_el.getAttribute(a))
        I = np.array([
            [g("ixx"), g("ixy"), g("ixz")],
            [g("ixy"), g("iyy"), g("iyz")],
            [g("ixz"), g("iyz"), g("izz")],
        ])
        links[link.getAttribute("name")] = (mass, xyz, R, I)

    def base_transform(link):
        if link == _MERGED_BASE_BODY or link not in joints:
            return np.zeros(3), np.eye(3)
        parent, xyz, R, _typ = joints[link]
        tp, Rp = base_transform(parent)
        return tp + Rp @ xyz, Rp @ R

    def fixed_members(root):
        members = [root] if root in links else []
        for child, (parent, _xyz, _R, typ) in joints.items():
            if parent == root and typ == "fixed":
                members += fixed_members(child)
        return members

    components = []
    total_mass = 0.0
    weighted_com = np.zeros(3)
    for name in fixed_members(_MERGED_BASE_BODY):
        mass, com_local, com_R, I_local = links[name]
        t_base, R_base = base_transform(name)
        com_base = t_base + R_base @ com_local
        R_full = R_base @ com_R
        I_base = R_full @ I_local @ R_full.T
        components.append((mass, com_base, I_base))
        total_mass += mass
        weighted_com += mass * com_base

    if total_mass <= 0.0:
        return None

    com = weighted_com / total_mass
    I_total = np.zeros((3, 3))
    for mass, com_i, I_i in components:
        d = com_i - com
        I_total += I_i + mass * (float(d @ d) * np.eye(3) - np.outer(d, d))

    return (
        total_mass,
        (float(com[0]), float(com[1]), float(com[2])),
        (
            float(I_total[0, 0]), float(I_total[1, 1]), float(I_total[2, 2]),
            float(I_total[0, 1]), float(I_total[0, 2]), float(I_total[1, 2]),
        ),
    )


def inject_base_footprint_inertial(mjcf_path: Path, urdf_str: str) -> bool:
    """Insert the true merged <inertial> into the base_footprint body (which the converter
    leaves inertial-less, forcing MuJoCo to invent a ~962 kg mass from the hull volume).
    Returns True on success, False if the inertial can't be computed or base_footprint is
    missing / already has an <inertial>."""
    result = compute_merged_base_inertial(urdf_str)
    if result is None:
        return False
    mass, com, I = result

    text = mjcf_path.read_text()
    body_match = re.search(r'<body name="' + re.escape(_MERGED_BASE_BODY) + r'"[^>]*>', text)
    if not body_match:
        return False
    # Guard against double-counting: only inject if the body has no inertial before its
    # first nested child body.
    body_start = body_match.end()
    next_child = text.find('<body name="', body_start)
    scope = text[body_start:next_child if next_child != -1 else len(text)]
    if "<inertial" in scope:
        return False

    inertial = (
        f'<inertial pos="{com[0]:.6f} {com[1]:.6f} {com[2]:.6f}" mass="{mass:.6f}" '
        f'fullinertia="{I[0]:.6f} {I[1]:.6f} {I[2]:.6f} {I[3]:.6f} {I[4]:.6f} {I[5]:.6f}"/>'
    )
    new_text, n = re.subn(
        r'(<body name="' + re.escape(_MERGED_BASE_BODY) + r'"[^>]*>)',
        lambda m: m.group(1) + inertial,
        text,
        count=1,
    )
    if n != 1:
        return False
    mjcf_path.write_text(new_text)
    return True


def mjcf_parses_as_xml(mjcf_path: Path) -> bool:
    """True iff mjcf_path exists and is well-formed XML (guards against a truncated
    or partially written converter output being accepted, patched, and cached)."""
    try:
        minidom.parse(str(mjcf_path))
    except (OSError, ExpatError, ValueError):
        return False
    return True


def converter_exit_acceptable(returncode: int, mjcf_path: Path) -> bool:
    """Whether an *early-exited* converter's output may be accepted.

    The converter normally spins forever once --publish_topic is set, so exiting at
    all is unexpected; only trust the output if the exit was clean (code 0) AND the
    file is non-empty, well-formed XML.
    """
    return (
        returncode == 0
        and mjcf_path.is_file()
        and mjcf_path.stat().st_size > 0
        and mjcf_parses_as_xml(mjcf_path)
    )


def finalize_conversion(mjcf_path: Path, cache_dir: Path, checksum: str, urdf_str: str) -> bool:
    """Validate, patch, and mark the freshly converted MJCF as cached.

    Applies (in order) the lidar scan-height raise, the wheel-collision primitives, and
    the base_footprint inertial injection. Returns True only if the file is well-formed
    XML and every patch matched its expected count; only then is the checksum written
    (making the cache entry valid). Any failure leaves the cache entry invalid (no
    checksum) so the next launch reconverts, and the caller must NOT publish the model.
    """
    if not mjcf_parses_as_xml(mjcf_path):
        print(
            f"[ensure_mjcf] ERROR: converter output {mjcf_path} is missing or not "
            f"well-formed XML; refusing to cache or publish it",
            flush=True,
        )
        return False

    patched = raise_lidar_scan_plane(mjcf_path)
    if not validate_patch_count(patched):
        print(
            f"[ensure_mjcf] ERROR: lidar scan-height patch matched {patched} bodies, "
            f"expected {EXPECTED_PATCH_COUNT}. The converter output no longer matches "
            f"the patch pattern in raise_lidar_scan_plane() (upstream converter change? "
            f"renamed lidar body?). Publishing this model would silently resurrect the "
            f"all -1.0 /scan bug, so refusing to cache or publish it -- inspect "
            f"{mjcf_path} and update the pattern.",
            flush=True,
        )
        return False

    print(f"[ensure_mjcf] raised {patched} lidar scan plane by {LIDAR_SCAN_RAISE_M} m (chassis left visible)", flush=True)

    wheel_patched = replace_wheel_collision_with_primitives(mjcf_path)
    if not validate_wheel_patch_count(wheel_patched):
        print(
            f"[ensure_mjcf] ERROR: wheel-collision patch applied {wheel_patched} changes, "
            f"expected {EXPECTED_WHEEL_PATCH_COUNT}. The converter output no longer matches "
            f"the patterns in replace_wheel_collision_with_primitives() (upstream converter "
            f"change? renamed wheel bodies?). The faceted convex-hull wheels cause base "
            f"wobble/jerky drive, so refusing to cache or publish it -- inspect {mjcf_path} "
            f"and update the patterns.",
            flush=True,
        )
        return False
    if not mjcf_parses_as_xml(mjcf_path):
        print(
            f"[ensure_mjcf] ERROR: wheel-collision patch produced malformed XML at "
            f"{mjcf_path}; refusing to cache or publish it",
            flush=True,
        )
        return False
    print(f"[ensure_mjcf] replaced wheel collision hulls with smooth primitives ({wheel_patched} patches)", flush=True)

    if not inject_base_footprint_inertial(mjcf_path, urdf_str):
        print(
            f"[ensure_mjcf] ERROR: could not inject base_footprint inertial into {mjcf_path} "
            f"(base_footprint missing, already has an <inertial>, or no rigidly-attached mass "
            f"found in the URDF). Without it MuJoCo invents a ~962 kg base from the hull "
            f"volume, which sags the contacts and makes the base wobble -- refusing to cache "
            f"or publish it.",
            flush=True,
        )
        return False
    if not mjcf_parses_as_xml(mjcf_path):
        print(
            f"[ensure_mjcf] ERROR: base_footprint inertial injection produced malformed XML "
            f"at {mjcf_path}; refusing to cache or publish it",
            flush=True,
        )
        return False
    print("[ensure_mjcf] injected true base_footprint inertial (fixes the ~962 kg phantom mass)", flush=True)

    (cache_dir / CHECKSUM_FILENAME).write_text(checksum + "\n")
    return True


def compute_checksum(urdf: str, world_content: str, extra: str) -> str:
    h = hashlib.sha256()
    h.update(urdf.encode())
    h.update(world_content.encode())
    h.update(extra.encode())
    return h.hexdigest()


def cache_valid(cache_dir: Path, checksum: str) -> bool:
    mjcf = cache_dir / MJCF_FILENAME
    stored = cache_dir / CHECKSUM_FILENAME
    if not (mjcf.is_file() and stored.is_file()):
        return False
    return stored.read_text().strip() == checksum


def internal_convert_topic(topic: str) -> str:
    """A throwaway topic name for the converter subprocess's own --publish_topic.

    See build_converter_cmd() for why the converter is still given --publish_topic
    (it changes <compiler meshdir=...> from a relative to an absolute path -- required
    for mujoco_ros2_control to resolve mesh files when it loads the MJCF from a ROS
    string message instead of a file path) even though nothing subscribes to this
    particular topic; this script republishes the patched file on the real `topic`.
    """
    return topic.rstrip("/") + "/_ensure_mjcf_unpatched"


def build_converter_cmd(urdf_file: Path, world_file: Path, cache_dir: Path, topic: str) -> list:
    """Build the upstream converter command.

    Passes --publish_topic (to a throwaway internal topic, see internal_convert_topic())
    rather than omitting it: mujoco_ros2_control/urdf_to_mujoco_utils.add_mujoco_info()
    only emits an ABSOLUTE <compiler meshdir=...> when --publish_topic is set (relative
    "assets/" otherwise, which only resolves correctly when MuJoCo loads the MJCF from a
    file path with a matching cwd). The live sim always loads the MJCF from the ROS
    string message published on `--topic`, which has no filesystem location of its own,
    so the absolute-path form is required. Passing the real topic here would let the
    converter's own internal publish race this script's patch_lidar_housing_visibility()
    step -- any subscriber on the real topic could see the unpatched (self-occluding
    lidar) model if it happened to receive that message first. This script is always the
    one that publishes on the real topic, after patching the file on disk.
    """
    return [
        "ros2", "run", "mujoco_ros2_control", "robot_description_to_mjcf.sh",
        "--save_only",
        "--add_free_joint",
        "-u", str(urdf_file),
        "--scene", str(world_file),
        "-o", str(cache_dir),
        "--publish_topic", internal_convert_topic(topic),
    ]


def default_cache_root() -> Path:
    ros_home = os.environ.get("ROS_HOME", os.path.join(os.path.expanduser("~"), ".ros"))
    return Path(ros_home) / "r2d3_mujoco"


def publish_cached(mjcf_path: Path, topic: str) -> None:
    """Latch the cached MJCF on the topic and spin until shutdown."""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
    from std_msgs.msg import String

    rclpy.init()
    node = Node("mjcf_publisher")
    qos = QoSProfile(
        depth=1,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )
    pub = node.create_publisher(String, topic.lstrip("/"), qos)
    msg = String()
    msg.data = mjcf_path.read_text()
    pub.publish(msg)
    node.get_logger().info(f"Published cached MJCF from {mjcf_path} ({len(msg.data)} bytes)")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def _wait_for_stable_output(child: subprocess.Popen, mjcf_path: Path, abort: threading.Event) -> bool:
    """Poll mjcf_path until its size stops changing, the child exits, we are told to
    abort, or we time out.

    Returns True if the file appeared and its size held steady for
    CONVERT_STABLE_FOR_S seconds (the converter has finished writing it).
    """
    deadline = time.monotonic() + CONVERT_TIMEOUT_S
    last_size = -1
    stable_since = None
    while time.monotonic() < deadline:
        if abort.is_set():
            return False
        if mjcf_path.is_file():
            size = mjcf_path.stat().st_size
            if size > 0 and size == last_size:
                if stable_since is None:
                    stable_since = time.monotonic()
                elif time.monotonic() - stable_since >= CONVERT_STABLE_FOR_S:
                    return True
            else:
                stable_since = None
            last_size = size
        if child.poll() is not None:
            # Converter process exited. It should spin forever once --publish_topic
            # is set, so exiting at all is unexpected -- only accept the output on a
            # clean exit (code 0) with non-empty, well-formed XML on disk.
            if converter_exit_acceptable(child.returncode, mjcf_path):
                return True
            print(
                f"[ensure_mjcf] ERROR: converter exited early (code {child.returncode}) "
                f"without valid output at {mjcf_path}",
                flush=True,
            )
            return False
        time.sleep(CONVERT_POLL_INTERVAL_S)
    return False


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-description", required=True, help="URDF XML string (xacro output)")
    parser.add_argument("--world", required=True, help="Path to the MuJoCo scene XML")
    parser.add_argument("--model", required=True, help="Robot model variant (65b|75b), names the cache dir")
    parser.add_argument("--topic", default="/mujoco_robot_description")
    parser.add_argument("--force", action="store_true", help="Recompile even if the cache is valid")
    parser.add_argument("--cache-root", default=None)
    # Tolerate ROS-injected args when run as a launch Node
    args, _unknown = parser.parse_known_args()

    cache_root = Path(args.cache_root) if args.cache_root else default_cache_root()
    cache_dir = cache_root / args.model
    cache_dir.mkdir(parents=True, exist_ok=True)

    world_path = Path(args.world)
    checksum = compute_checksum(args.robot_description, world_path.read_text(), CONVERTER_ARGS_VERSION)

    if not args.force and cache_valid(cache_dir, checksum):
        print(f"[ensure_mjcf] cache hit ({cache_dir}); skipping conversion", flush=True)
        publish_cached(cache_dir / MJCF_FILENAME, args.topic)
        return 0

    print(f"[ensure_mjcf] cache miss; converting URDF -> MJCF into {cache_dir}", flush=True)
    # Invalidate stale checksum before converting so an interrupted run stays invalid
    (cache_dir / CHECKSUM_FILENAME).unlink(missing_ok=True)
    urdf_file = cache_dir / URDF_FILENAME
    urdf_file.write_text(args.robot_description)
    mjcf_path = cache_dir / MJCF_FILENAME
    mjcf_path.unlink(missing_ok=True)

    # New process group so we can reliably kill the whole `ros2 run ... .sh -> python`
    # descendant chain (see git history: git show aaae018:r2d3_mujoco/SPIKE_NOTES.md: killing only the immediate child PID doesn't
    # always reach the underlying python process).
    child = subprocess.Popen(
        build_converter_cmd(urdf_file, world_path, cache_dir, args.topic),
        preexec_fn=os.setsid,
    )

    abort = threading.Event()

    def _kill_child_group():
        try:
            os.killpg(os.getpgid(child.pid), signal.SIGTERM)
        except ProcessLookupError:
            pass

    def _forward_signal(_signum, _frame):
        # Mark the run as aborted BEFORE killing the child: main() must not go on
        # to patch/checksum/publish a possibly truncated file (and then sit in
        # rclpy.spin() ignoring the shutdown request).
        abort.set()
        _kill_child_group()

    signal.signal(signal.SIGTERM, _forward_signal)
    signal.signal(signal.SIGINT, _forward_signal)

    converted = _wait_for_stable_output(child, mjcf_path, abort)
    _kill_child_group()
    try:
        child.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(child.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
        child.wait(timeout=5.0)

    if abort.is_set():
        print("[ensure_mjcf] interrupted; aborting without caching or publishing", flush=True)
        return 1

    if not converted:
        print(f"[ensure_mjcf] conversion timed out or failed; {mjcf_path} not usable", flush=True)
        return 1

    if not finalize_conversion(mjcf_path, cache_dir, checksum, args.robot_description):
        return 1

    # Conversion is done; restore default signal handling so Ctrl-C/SIGTERM can
    # interrupt the rclpy.spin() inside publish_cached() instead of being forwarded
    # to the (already-exited) child process group and swallowed.
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    signal.signal(signal.SIGTERM, signal.SIG_DFL)

    publish_cached(mjcf_path, args.topic)
    return 0


if __name__ == "__main__":
    sys.exit(main())
