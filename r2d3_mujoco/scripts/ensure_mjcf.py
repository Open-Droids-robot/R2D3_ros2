#!/usr/bin/env python3
"""
Cached URDF->MJCF conversion for the R2D3 MuJoCo simulation.

Hashes the xacro-generated URDF plus the world file. On a cache hit the
previously generated MJCF is published directly (skipping the expensive mesh
conversion); on a miss the upstream converter is invoked (--publish_topic pointed at
a throwaway internal topic -- see build_converter_cmd() -- so its own publish is
harmless) to save the MJCF and assets into the cache directory, this script patches
the lidar/chassis geoms that self-occlude the rangefinder rays for ray-cast
visibility (see patch_lidar_housing_visibility()), and only then publishes the
patched model on the real --topic itself -- this script is always the one true
publisher on that topic, so a cache-miss run never lets a subscriber see the
unpatched converter output.

The MJCF is published on --topic with RELIABLE + TRANSIENT_LOCAL QoS, matching
the subscriber inside mujoco_ros2_control.
"""

import argparse
import hashlib
import os
import re
import signal
import subprocess
import sys
import time
from pathlib import Path

# The converter subprocess is launched with --publish_topic (see build_converter_cmd()),
# which per SPIKE_NOTES.md makes it spin forever (rclpy.spin()) after writing its output
# files instead of exiting -- so this script cannot wait() on it for completion. Instead
# it polls the output MJCF for having stopped changing size, the same file-stability
# signal SPIKE_NOTES.md used manually, then kills the whole process group.
CONVERT_POLL_INTERVAL_S = 0.5
CONVERT_STABLE_FOR_S = 2.0
CONVERT_TIMEOUT_S = 300.0

MJCF_FILENAME = "mujoco_description_formatted.xml"
CHECKSUM_FILENAME = "checksum"
URDF_FILENAME = "robot_input.urdf"
# Bump when converter flags in build_converter_cmd change, or when
# patch_lidar_housing_visibility()'s matching logic changes, to invalidate caches.
CONVERTER_ARGS_VERSION = "v3:save_only,add_free_joint,scene,lidar_and_chassis_ray_fix"

# --- Lidar self-occlusion fix -------------------------------------------------------
#
# Root cause (see SPIKE_NOTES.md "Lidar self-occlusion" for the full investigation):
# every ray leaving the lidar's rangefinder site immediately re-intersects the robot's
# own chassis a few centimeters out, so every /scan range comes back below range_min
# (0.55 m) and gets reported as -1.0 by rangefinder_lidar_plugin.cpp. MuJoCo's
# rangefinder always excludes exactly one body from ray casting -- the body owning its
# own site (engine_sensor.c: `bodyexclude = m->site_bodyid[objid]`) -- but
# mujoco_ros2_control's converter unconditionally creates the replicated lidar rays'
# sites in their own dedicated, geometry-less "<site>_lidar_body"
# (urdf_to_mujoco_utils.add_lidar_from_sites(), hard-coded, not configurable from
# r2d3_mujoco/), so that exclusion never reaches the chassis geoms doing the occluding.
# geomgroup-based filtering doesn't help either: rangefinder sensors always call
# mj_ray/mj_multiRay with geomgroup=NULL (confirmed against engine_sensor.c), so a
# geom's `group` has no effect on ray casting regardless of default-class settings.
#
# The one filter ray casting DOES honor unconditionally is alpha: MuJoCo's
# ray_eliminate() (engine_ray.c) drops any geom whose rgba alpha (or material alpha)
# is exactly 0. Every robot geom is an unnamed clone of a URDF visual mesh/primitive
# (mujoco_ros2_control synthesizes an unnamed collision+visual pair from every
# visual-only URDF <visual>, see Task 4's SPIKE_NOTES finding: 0/68 robot geoms carry
# a `name` attribute), so `modify_element` can never target an individual occluding
# geom by name. Two things CAN still uniquely identify a specific geom in the
# generated text, though: mesh geoms keep a `mesh="<stl-name>"` attribute, and the
# lidar housing is the only primitive geom of its exact size. Both are matched here
# and patched to alpha=0, in order of what was found occluding rays in this
# investigation (each verified independently with a standalone mj_ray script against
# the cached MJCF -- see task-7-report.md):
#
#   1. The lidar housing itself: modeled in
#      dual_rm_simulation/urdf/sensors/lidar.urdf.xacro as a
#      <cylinder radius="0.03" length="0.05"/> visual primitive on lidar_link, whose
#      origin coincides exactly with the rangefinder site -- the sensor's own optical
#      center sits at the geometric center of its own housing, so every ray starts
#      inside solid geometry and immediately exits through the housing wall (~0.03 m,
#      the housing radius). This is the ONLY geom pair in the generated MJCF of this
#      exact size/type.
#   2. base_link_underpan and body_base_link (the chassis pan geometry the lidar is
#      mounted to): even with the housing cleared, ALL 240 rays still self-hit these
#      two meshes' true (non-convex) surface at <0.17 m in every direction -- the
#      mount sits flush against a raised boss on the chassis, not just inside the
#      collision hull's padding. Hiding these two meshes from ray casting also hides
#      them from the RGB/depth camera's rendering (alpha is a rendering property, not
#      a ray-casting-only one), but the camera is head-mounted looking outward/forward
#      (see the `camera` site's pose in the generated MJCF), so the underpan is not
#      normally in frame; this was judged an acceptable trade-off against a /scan that
#      never reports any wall (see task-7-report.md fix report for the accepted
#      trade-off discussion).
#
# Collision-class geoms already get rgba="... 0" unconditionally via the shared
# "collision" default class in mujoco_inputs.urdf.xacro (defense in depth: the cruder
# convex-hull collision copies are hidden from ray casting robot-wide, not just for
# these three meshes) -- the matchers below additionally cover the VISUAL copies,
# which carry their own inline rgba (baked from URDF material colors) that overrides
# the class default and must be patched explicitly.
_RAY_OCCLUDING_GEOM_SIZE = "0.03 0.025"  # lidar housing cylinder (radius, half-length)
_RAY_OCCLUDING_MESH_NAMES = ("base_link_underpan", "body_base_link")


def _is_ray_occluding_geom_tag(tag: str) -> bool:
    if 'type="cylinder"' in tag and f'size="{_RAY_OCCLUDING_GEOM_SIZE}"' in tag:
        return True
    return any(f'mesh="{name}"' in tag for name in _RAY_OCCLUDING_MESH_NAMES)


def patch_lidar_housing_visibility(mjcf_path: Path) -> int:
    """Make the lidar-self-occluding geoms (see module docstring above) invisible to
    ray casting by zeroing their rgba alpha, while leaving contact physics untouched
    (rgba is a purely visual/ray-casting property).

    Returns the number of <geom> elements patched (expected: 6 -- the lidar housing's
    collision+visual pair, plus the collision+visual pair for each of the two
    occluding chassis meshes). Uses a targeted text substitution (not a full XML
    re-serialization) so the rest of the generated file -- comments, attribute order,
    formatting -- is left untouched.
    """
    text = mjcf_path.read_text()
    pattern = re.compile(r"<geom\b[^>]*/>")
    patched_count = 0

    def _patch_tag(match: "re.Match") -> str:
        nonlocal patched_count
        tag = match.group(0)
        if not _is_ray_occluding_geom_tag(tag):
            return tag
        patched_count += 1
        if "rgba=" in tag:
            tag = re.sub(r'rgba="([^"]*)"', lambda m: f'rgba="{_zero_alpha(m.group(1))}"', tag)
        else:
            tag = tag.replace("<geom ", '<geom rgba="1 1 1 0" ', 1)
        return tag

    patched_text = pattern.sub(_patch_tag, text)
    if patched_count:
        mjcf_path.write_text(patched_text)
    return patched_count


def _zero_alpha(rgba: str) -> str:
    parts = rgba.split()
    if len(parts) == 4:
        parts[3] = "0"
    return " ".join(parts)


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


def _wait_for_stable_output(child: subprocess.Popen, mjcf_path: Path) -> bool:
    """Poll mjcf_path until its size stops changing, the child exits, or we time out.

    Returns True if the file appeared and its size held steady for
    CONVERT_STABLE_FOR_S seconds (the converter has finished writing it).
    """
    deadline = time.monotonic() + CONVERT_TIMEOUT_S
    last_size = -1
    stable_since = None
    while time.monotonic() < deadline:
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
            # Converter process exited (crash, or -- since it should spin forever
            # with --publish_topic -- something unexpected). Give the file one more
            # look in case it exited immediately after the last write.
            return mjcf_path.is_file() and mjcf_path.stat().st_size > 0
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
    # descendant chain (SPIKE_NOTES.md: killing only the immediate child PID doesn't
    # always reach the underlying python process).
    child = subprocess.Popen(
        build_converter_cmd(urdf_file, world_path, cache_dir, args.topic),
        preexec_fn=os.setsid,
    )

    def _kill_child_group():
        try:
            os.killpg(os.getpgid(child.pid), signal.SIGTERM)
        except ProcessLookupError:
            pass

    def _forward_signal(_signum, _frame):
        _kill_child_group()

    signal.signal(signal.SIGTERM, _forward_signal)
    signal.signal(signal.SIGINT, _forward_signal)

    converted = _wait_for_stable_output(child, mjcf_path)
    _kill_child_group()
    try:
        child.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(child.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
        child.wait(timeout=5.0)

    if not converted:
        print(f"[ensure_mjcf] conversion timed out or failed; {mjcf_path} not produced", flush=True)
        return 1

    patched = patch_lidar_housing_visibility(mjcf_path)
    expected = 2 * (1 + len(_RAY_OCCLUDING_MESH_NAMES))
    if patched != expected:
        print(
            f"[ensure_mjcf] WARNING: expected to patch {expected} ray-occluding geoms "
            f"for lidar self-occlusion, patched {patched} (the fix may not have fully "
            f"applied -- check {mjcf_path} for the lidar housing / chassis mesh geoms)",
            flush=True,
        )
    else:
        print(f"[ensure_mjcf] patched {patched} ray-occluding geoms for lidar self-occlusion fix", flush=True)

    (cache_dir / CHECKSUM_FILENAME).write_text(checksum + "\n")
    publish_cached(mjcf_path, args.topic)
    return 0


if __name__ == "__main__":
    sys.exit(main())
