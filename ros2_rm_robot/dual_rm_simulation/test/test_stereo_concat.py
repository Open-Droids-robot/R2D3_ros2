"""Unit tests for the sim-only stereo_concat node's pure concat function."""
import sys
import unittest
from pathlib import Path

import numpy as np
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


if __name__ == "__main__":
    unittest.main()
