#!/usr/bin/env python3
"""Readiness gate for the MuJoCo bringup.

The MuJoCo sim takes a variable, sometimes long, time to come up (URDF->MJCF
conversion on a cold cache, GUI + MoveIt + RViz competing for CPU). The old
bringup started Nav2/SLAM on a blind ``TimerAction(period=10.0)``; on a slow
start the sim's TF tree and /scan were not ready yet, so SLAM never built a
map -> odom transform and RViz showed no ``map`` frame.

This node blocks until the sim is genuinely ready, then exits 0. The bringup
fires Nav2/SLAM/MoveIt off that exit (OnProcessExit) instead of a fixed timer,
so the stack starts as soon as -- and no sooner than -- the sim can feed it.

Ready means all of:
  * at least one /scan message received (lidar sensor pipeline alive), and
  * TF ``odom`` -> ``base_footprint`` available (diff-drive odom TF flowing), and
  * TF ``base_footprint`` -> ``laser_link`` available (robot_state_publisher up).

A ``--timeout`` fallback guarantees the gate always exits (0) even if a signal
never arrives, so nav still starts -- never worse than the old fixed timer.
"""

import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image, LaserScan
from tf2_ros import Buffer, TransformListener


def signals_ready(got_scan, got_camera, odom_tf_ok, laser_tf_ok):
    """True only when every readiness signal is present: /scan proves the
    lidar pipeline, the camera image proves the ZED sim camera pipeline,
    odom->base_footprint proves diff-drive odometry, and
    base_footprint->laser_link proves robot_state_publisher. SLAM needs all
    of them connected to build a map -> odom transform."""
    return bool(got_scan and got_camera and odom_tf_ok and laser_tf_ok)


def missing_signal_names(got_scan, got_camera, odom_tf_ok, laser_tf_ok,
                         odom_frame, base_frame, laser_frame, camera_topic):
    """Human-readable list of the signals still missing (for the timeout log)."""
    missing = []
    if not got_scan:
        missing.append("/scan")
    if not got_camera:
        missing.append(camera_topic)
    if not odom_tf_ok:
        missing.append(f"TF {odom_frame}->{base_frame}")
    if not laser_tf_ok:
        missing.append(f"TF {base_frame}->{laser_frame}")
    return missing


class SimReadyGate(Node):
    def __init__(self, scan_topic, camera_topic, odom_frame, base_frame,
                 laser_frame, timeout):
        super().__init__("wait_for_sim_ready")
        # This gate reasons about wall-clock elapsed time for its timeout and
        # only asks TF for the latest available transform, so it deliberately
        # does NOT use sim time (which may not be published yet at startup).
        self._odom_frame = odom_frame
        self._base_frame = base_frame
        self._laser_frame = laser_frame
        self._camera_topic = camera_topic
        self._timeout = timeout

        self._got_scan = False
        self._got_camera = camera_topic == ""
        self._start = self.get_clock().now()

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._scan_sub = self.create_subscription(
            LaserScan, scan_topic, self._on_scan, 1)
        if camera_topic:
            self._camera_sub = self.create_subscription(
                Image, camera_topic, self._on_camera, 1)
        self._timer = self.create_timer(0.5, self._check)

        camera_msg = f" + camera {camera_topic}" if camera_topic else ""
        self.get_logger().info(
            f"Waiting for sim readiness: /scan{camera_msg} + "
            f"TF {odom_frame}->{base_frame} "
            f"+ TF {base_frame}->{laser_frame} (timeout {timeout:.0f}s)")

    def _on_scan(self, _msg):
        if not self._got_scan:
            self.get_logger().info("First /scan received.")
        self._got_scan = True

    def _on_camera(self, _msg):
        if not self._got_camera:
            self.get_logger().info("First camera image received.")
        self._got_camera = True

    def _tf_ready(self, target, source):
        return self._tf_buffer.can_transform(target, source, Time())

    def _elapsed(self):
        return (self.get_clock().now() - self._start).nanoseconds / 1e9

    def _check(self):
        odom_ok = self._tf_ready(self._odom_frame, self._base_frame)
        laser_ok = self._tf_ready(self._base_frame, self._laser_frame)
        if signals_ready(self._got_scan, self._got_camera, odom_ok, laser_ok):
            self.get_logger().info(
                f"Sim ready after {self._elapsed():.1f}s -- releasing Nav2/SLAM.")
            self._done(0)
            return
        if self._elapsed() >= self._timeout:
            missing = missing_signal_names(
                self._got_scan, self._got_camera, odom_ok, laser_ok,
                self._odom_frame, self._base_frame, self._laser_frame,
                camera_topic=self._camera_topic)
            self.get_logger().warn(
                f"Readiness timeout after {self._timeout:.0f}s; still missing: "
                f"{', '.join(missing)}. Releasing Nav2/SLAM anyway.")
            self._done(0)

    def _done(self, code):
        self._timer.cancel()
        # Stash exit code and stop spinning; rclpy.shutdown happens in main.
        self._exit_code = code
        raise SystemExit(code)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scan-topic", default="/scan")
    parser.add_argument("--camera-topic",
                        default="/zed/zed_node/left/image_rect_color",
                        help="camera image readiness topic ('' disables)")
    parser.add_argument("--odom-frame", default="odom")
    parser.add_argument("--base-frame", default="base_footprint")
    parser.add_argument("--laser-frame", default="laser_link")
    parser.add_argument("--timeout", type=float, default=90.0,
                        help="Fallback: exit 0 after this many seconds regardless.")
    # ros2 launch passes --ros-args ...; ignore anything argparse doesn't know.
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = SimReadyGate(args.scan_topic, args.camera_topic, args.odom_frame,
                        args.base_frame, args.laser_frame, args.timeout)
    code = 0
    try:
        rclpy.spin(node)
    except SystemExit as exc:
        code = int(exc.code) if exc.code is not None else 0
    except KeyboardInterrupt:
        code = 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
