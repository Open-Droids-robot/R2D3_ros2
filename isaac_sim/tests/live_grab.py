"""Grab the LIVE head-D435 frames from ROS (run in ros_humble env).

Subscribes to /camera/color/image_raw + /camera/depth/image_raw published by
Isaac's OmniGraph ROS2 camera (correct tonemapping, unlike the offline
replicator path), collects a few seconds, and saves PNGs. Expects
`bring_up.py --demo-workspace` running.
"""
import sys
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

OUT = Path("/usr1/home/semathew/r2d3_isaac/isaac_sim/tests/captures")


class G(Node):
    def __init__(self):
        super().__init__("live_grab")
        self.color = None
        self.depth = None
        self.create_subscription(Image, "/camera/color/image_raw",
                                 lambda m: setattr(self, "color", m), qos_profile_sensor_data)
        self.create_subscription(Image, "/camera/depth/image_raw",
                                 lambda m: setattr(self, "depth", m), qos_profile_sensor_data)


def main():
    rclpy.init()
    n = G()
    start = n.get_clock().now()
    while rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.1)
        if (n.get_clock().now() - start).nanoseconds / 1e9 > 6.0:
            break

    from PIL import Image as PImage
    if n.color is not None and n.color.width:
        c = n.color
        ch = 4 if c.encoding == "rgba8" else 3
        arr = np.frombuffer(bytes(c.data), np.uint8).reshape(c.height, c.width, ch)[:, :, :3]
        PImage.fromarray(arr, "RGB").save(OUT / "cam_live_rgb.png")
        print(f"[live] color {c.width}x{c.height} enc={c.encoding} "
              f"min={arr.min()} max={arr.max()} mean={arr.mean():.0f} -> cam_live_rgb.png", flush=True)
    else:
        print("[live] no color frame", flush=True)

    if n.depth is not None and n.depth.width:
        d = n.depth
        dd = np.frombuffer(bytes(d.data), np.float32).reshape(d.height, d.width)
        fin = dd[np.isfinite(dd)]
        if fin.size:
            print(f"[live] depth {d.width}x{d.height} range {fin.min():.3f}..{fin.max():.3f} m", flush=True)
        cl = np.clip(np.nan_to_num(dd, nan=2.5, posinf=2.5), 0.3, 2.5)
        norm = ((cl - 0.3) / 2.2 * 255).astype(np.uint8)
        try:
            import matplotlib.cm as cm
            PImage.fromarray((cm.get_cmap("turbo")(norm / 255.0) * 255).astype(np.uint8)[:, :, :3],
                             "RGB").save(OUT / "cam_live_depth_color.png")
            print("[live] -> cam_live_depth_color.png", flush=True)
        except Exception:
            pass
    else:
        print("[live] no depth frame", flush=True)

    print("[live] DONE", flush=True)
    n.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
