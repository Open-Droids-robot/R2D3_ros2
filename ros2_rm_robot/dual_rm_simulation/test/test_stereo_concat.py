"""Unit tests for the sim-only stereo_concat node: the pure concat function
and the sync/QoS wiring.

The wiring tests are regression guards from the 2026-07-16 debug round:
- mujoco_ros2_control stamps its two cameras with SEPARATE now() calls, so a
  sim-clock tick lands between them ~2/3 of the time (measured: 2 ms skew,
  51/150 equal stamps). An exact TimeSynchronizer therefore starves; the node
  must use ApproximateTimeSynchronizer with slop >= that skew.
- The sim's camera publishers are RELIABLE and each render cycle bursts
  ~13 MB; BEST_EFFORT subscribers lose the tail of the burst (measured: right
  eye 10/137 msgs vs 148/149 with a RELIABLE subscriber). The node's
  subscriptions must be RELIABLE.
"""
import sys
import unittest
from pathlib import Path

import numpy as np
import rclpy
from message_filters import ApproximateTimeSynchronizer
from rclpy.duration import Duration
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import Image

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "scripts"))
import stereo_concat  # noqa: E402


def _img(width, height, value, encoding="rgb8", frame="zed_left_camera_frame_optical"):
    bpp = {"rgb8": 3, "mono8": 1}[encoding]
    msg = Image()
    msg.header.frame_id = frame
    msg.header.stamp.sec = 42
    msg.height = height
    msg.width = width
    msg.encoding = encoding
    msg.is_bigendian = 0
    msg.step = width * bpp
    msg.data = bytes([value]) * (height * msg.step)
    return msg


class TestHconcatImages(unittest.TestCase):
    def test_double_width_output(self):
        out = stereo_concat.hconcat_images(_img(4, 2, 10), _img(4, 2, 20))
        self.assertEqual(out.width, 8)
        self.assertEqual(out.height, 2)
        self.assertEqual(out.step, 8 * 3)
        self.assertEqual(len(out.data), 2 * 8 * 3)

    def test_row_layout_left_then_right(self):
        out = stereo_concat.hconcat_images(_img(2, 1, 10), _img(2, 1, 20))
        row = np.frombuffer(out.data, np.uint8)
        # left pixels (2 px * rgb) then right pixels, in the same row
        self.assertEqual(row[:6].tolist(), [10] * 6)
        self.assertEqual(row[6:].tolist(), [20] * 6)

    def test_header_taken_from_left(self):
        left = _img(2, 1, 10)
        right = _img(2, 1, 20, frame="zed_right_camera_frame_optical")
        out = stereo_concat.hconcat_images(left, right)
        self.assertEqual(out.header.frame_id, "zed_left_camera_frame_optical")
        self.assertEqual(out.header.stamp.sec, 42)
        self.assertEqual(out.encoding, "rgb8")

    def test_mismatched_height_raises(self):
        with self.assertRaises(ValueError):
            stereo_concat.hconcat_images(_img(2, 1, 10), _img(2, 2, 20))

    def test_mismatched_encoding_raises(self):
        with self.assertRaises(ValueError):
            stereo_concat.hconcat_images(_img(2, 1, 10), _img(2, 1, 20, encoding="mono8"))


class TestNodeWiring(unittest.TestCase):
    """Sync strategy + QoS are load-bearing (see module docstring)."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = stereo_concat.StereoConcat()

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_uses_approximate_time_sync_with_sufficient_slop(self):
        self.assertIsInstance(self.node._sync, ApproximateTimeSynchronizer)
        # Must absorb the measured 2 ms inter-eye stamp skew with margin,
        # but stay under one frame period (66 ms at 15 Hz) to never pair
        # across frames.
        self.assertGreaterEqual(self.node._sync.slop, Duration(seconds=0.002))
        self.assertLess(self.node._sync.slop, Duration(seconds=0.066))

    def test_all_subscriptions_reliable(self):
        zed_subs = [s for s in self.node.subscriptions
                    if "/zed/zed_node/" in s.topic_name]
        self.assertEqual(len(zed_subs), 3)  # left img, right img, left info
        for s in zed_subs:
            self.assertEqual(
                s.qos_profile.reliability, ReliabilityPolicy.RELIABLE,
                f"{s.topic_name} must subscribe RELIABLE (best-effort loses "
                f"the tail of each render burst)")

    def test_all_publishers_reliable(self):
        zed_pubs = [p for p in self.node.publishers
                    if "/zed/zed_node/" in p.topic_name]
        self.assertEqual(len(zed_pubs), 3)  # stereo, rgb img, rgb info
        for p in zed_pubs:
            self.assertEqual(
                p.qos_profile.reliability, ReliabilityPolicy.RELIABLE,
                f"{p.topic_name} should publish RELIABLE (compatible with "
                f"both reliable and best-effort consumers)")


if __name__ == "__main__":
    unittest.main()
