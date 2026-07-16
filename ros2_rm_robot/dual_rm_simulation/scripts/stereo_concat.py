#!/usr/bin/env python3
"""Sim-only ZED topic shim: side-by-side stereo image + rgb alias.

The real zed-ros2-wrapper natively publishes
  /zed/zed_node/stereo/image_rect_color  (left|right side-by-side), and
  /zed/zed_node/rgb/image_rect_color(+camera_info)  (alias of the left eye).
Neither Gz Sim nor MuJoCo has a side-by-side stereo sensor, so this node
synthesizes both from the simulated left/right streams. It must NOT run on
the real robot -- zed_node already publishes these topics there.

Exact-time sync is deliberate: both sim eyes stamp identical sim time. If
either eye stalls, nothing is published (no stale pairs).
"""
import numpy as np
import rclpy
from message_filters import Subscriber, TimeSynchronizer
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

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
            Image, "/zed/zed_node/stereo/image_rect_color",
            qos_profile_sensor_data)
        self._rgb_pub = self.create_publisher(
            Image, "/zed/zed_node/rgb/image_rect_color",
            qos_profile_sensor_data)
        self._rgb_info_pub = self.create_publisher(
            CameraInfo, "/zed/zed_node/rgb/camera_info",
            qos_profile_sensor_data)

        left_sub = Subscriber(
            self, Image, "/zed/zed_node/left/image_rect_color",
            qos_profile=qos_profile_sensor_data)
        right_sub = Subscriber(
            self, Image, "/zed/zed_node/right/image_rect_color",
            qos_profile=qos_profile_sensor_data)
        self._sync = TimeSynchronizer([left_sub, right_sub], 5)
        self._sync.registerCallback(self._on_pair)

        self._info_sub = self.create_subscription(
            CameraInfo, "/zed/zed_node/left/camera_info",
            self._on_left_info, qos_profile_sensor_data)

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
