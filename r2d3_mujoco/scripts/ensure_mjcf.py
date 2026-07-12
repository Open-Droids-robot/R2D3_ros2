#!/usr/bin/env python3
"""
Cached URDF->MJCF conversion for the R2D3 MuJoCo simulation.

Hashes the xacro-generated URDF plus the world file. On a cache hit the
previously generated MJCF is published directly (skipping the expensive mesh
conversion); on a miss the upstream converter is invoked, which saves the MJCF
and assets into the cache directory and publishes the model itself.

The MJCF is published on --topic with RELIABLE + TRANSIENT_LOCAL QoS, matching
the subscriber inside mujoco_ros2_control.
"""

import argparse
import hashlib
import os
import signal
import subprocess
import sys
import threading
from pathlib import Path

MJCF_FILENAME = "mujoco_description_formatted.xml"
CHECKSUM_FILENAME = "checksum"
URDF_FILENAME = "robot_input.urdf"
# Bump when converter flags in build_converter_cmd change, to invalidate caches
CONVERTER_ARGS_VERSION = "v1:save_only,add_free_joint,scene"


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


def build_converter_cmd(urdf_file: Path, world_file: Path, cache_dir: Path, topic: str) -> list:
    return [
        "ros2", "run", "mujoco_ros2_control", "robot_description_to_mjcf.sh",
        "--save_only",
        "--add_free_joint",
        "-u", str(urdf_file),
        "--scene", str(world_file),
        "-o", str(cache_dir),
        "--publish_topic", topic,
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


def watch_and_write_checksum(topic: str, cache_dir: Path, checksum: str) -> threading.Thread:
    """Subscribe to the MJCF topic; when the converter publishes, persist the checksum."""

    def _watch():
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
        from std_msgs.msg import String

        rclpy.init()
        node = Node("mjcf_checksum_writer")
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        done = threading.Event()

        def _cb(_msg):
            (cache_dir / CHECKSUM_FILENAME).write_text(checksum + "\n")
            node.get_logger().info(f"Conversion complete; checksum stored in {cache_dir}")
            done.set()

        node.create_subscription(String, topic.lstrip("/"), _cb, qos)
        while rclpy.ok() and not done.is_set():
            rclpy.spin_once(node, timeout_sec=1.0)
        node.destroy_node()
        rclpy.try_shutdown()

    t = threading.Thread(target=_watch, daemon=True)
    t.start()
    return t


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

    watch_and_write_checksum(args.topic, cache_dir, checksum)
    child = subprocess.Popen(build_converter_cmd(urdf_file, world_path, cache_dir, args.topic))

    def _forward_sigterm(_signum, _frame):
        child.terminate()

    signal.signal(signal.SIGTERM, _forward_sigterm)
    signal.signal(signal.SIGINT, _forward_sigterm)
    return child.wait()


if __name__ == "__main__":
    sys.exit(main())
