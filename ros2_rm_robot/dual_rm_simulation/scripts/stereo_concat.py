#!/usr/bin/env python3
"""Sim-only ZED topic shim: side-by-side stereo image + rgb alias.

The real zed-ros2-wrapper (v5.x) natively publishes
  /zed/zed_node/stereo/color/rect/image  (left|right side-by-side), and
  /zed/zed_node/rgb/color/rect/image(+camera_info)  (alias of the left eye).
Neither Gz Sim nor MuJoCo has a side-by-side stereo sensor, so this node
synthesizes both from the simulated left/right streams. It must NOT run on
the real robot -- zed_node already publishes these topics there.

Sync + QoS (2026-07-16 debug round — do not "simplify" either):
- ApproximateTimeSynchronizer, NOT exact: the sims do NOT stamp both eyes
  identically. mujoco_ros2_control stamps each camera with its own now()
  call, so a sim-clock tick lands between the eyes ~2/3 of the time
  (measured 2 ms skew; exact sync starved to ~1/3 rate, and to ~0 under
  transport loss). Slop is well under one frame period (66 ms at 15 Hz),
  so pairing across frames is impossible. Stall-safety is preserved: if
  one eye stops, no pairs form and nothing is published.
- RELIABLE subscriptions, NOT best-effort: the sim publishes ~13 MB of
  images per render cycle; with best-effort the tail of the burst (the
  right eye) is dropped almost entirely (measured 10/137 msgs vs 148/149
  reliable). Publishers are reliable too — compatible with both reliable
  and best-effort consumers.
"""
import numpy as np
import rclpy
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

# Absorbs the measured 2 ms inter-eye stamp skew with margin while staying
# far below the 66 ms frame period. Guarded by test_stereo_concat.py.
SYNC_SLOP_S = 0.034
_QOS = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                  history=HistoryPolicy.KEEP_LAST)

_BYTES_PER_PIXEL = {
    "rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1,
}


def hconcat_images(left: Image, right: Image) -> Image:
    """Concatenate two Images horizontally (left | right). Header from left."""
    if left.height != right.height:
        raise ValueError(
            f"height mismatch: left={left.height} right={right.height}")
    if left.encoding != right.encoding:
        raise ValueError(
            f"encoding mismatch: left={left.encoding} right={right.encoding}")
    if left.encoding not in _BYTES_PER_PIXEL:
        raise ValueError(f"unsupported encoding: {left.encoding}")
    bpp = _BYTES_PER_PIXEL[left.encoding]

    l_rows = np.frombuffer(left.data, np.uint8).reshape(left.height, left.step)
    r_rows = np.frombuffer(right.data, np.uint8).reshape(right.height, right.step)
    # Drop any row padding beyond width*bpp before concatenating.
    l_rows = l_rows[:, : left.width * bpp]
    r_rows = r_rows[:, : right.width * bpp]

    out = Image()
    out.header = left.header
    out.height = left.height
    out.width = left.width + right.width
    out.encoding = left.encoding
    out.is_bigendian = left.is_bigendian
    out.step = out.width * bpp
    out.data = np.hstack((l_rows, r_rows)).tobytes()
    return out


class StereoConcat(Node):
    def __init__(self):
        super().__init__("stereo_concat")
        self._stereo_pub = self.create_publisher(
            Image, "/zed/zed_node/stereo/color/rect/image", _QOS)
        self._rgb_pub = self.create_publisher(
            Image, "/zed/zed_node/rgb/color/rect/image", _QOS)
        self._rgb_info_pub = self.create_publisher(
            CameraInfo, "/zed/zed_node/rgb/color/rect/camera_info", _QOS)

        left_sub = Subscriber(
            self, Image, "/zed/zed_node/left/color/rect/image",
            qos_profile=_QOS)
        right_sub = Subscriber(
            self, Image, "/zed/zed_node/right/color/rect/image",
            qos_profile=_QOS)
        self._sync = ApproximateTimeSynchronizer(
            [left_sub, right_sub], 5, SYNC_SLOP_S)
        self._sync.registerCallback(self._on_pair)

        self._info_sub = self.create_subscription(
            CameraInfo, "/zed/zed_node/left/color/rect/camera_info",
            self._on_left_info, _QOS)

    def _on_pair(self, left: Image, right: Image):
        try:
            self._stereo_pub.publish(hconcat_images(left, right))
        except ValueError as e:
            self.get_logger().warn(f"skipping stereo pair: {e}",
                                   throttle_duration_sec=5.0)
        self._rgb_pub.publish(left)

    def _on_left_info(self, info: CameraInfo):
        self._rgb_info_pub.publish(info)


def main():
    rclpy.init()
    node = StereoConcat()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
