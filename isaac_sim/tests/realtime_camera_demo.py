"""Real-time head-D435 demo (run in ros_humble env).

Drives the FULL loop over ROS: publishes head-pan commands to the sim's command
topic, the robot pans, and the LIVE D435 view sweeps across the workspace. Saves
a frame at each pan step into a filmstrip and measures the live publish rate.

Expects `bring_up.py --demo-workspace` running (camera live, table+cube in view).
"""
import sys, time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from sensor_msgs.msg import Image, JointState

OUT = Path("/usr1/home/semathew/r2d3_isaac/isaac_sim/tests/captures")
PANS = [-0.35, -0.21, -0.07, 0.07, 0.21, 0.35]   # rad, head_joint1 sweep
TILT = -0.45                                       # keep looking down at the table
DWELL_S = 1.3                                      # settle time per pan step


class Demo(Node):
    def __init__(self):
        super().__init__("realtime_camera_demo")
        self.latest = None
        self.count = 0
        self.create_subscription(Image, "/camera/color/image_raw",
                                 self._on_img, qos_profile_sensor_data)
        self.pub = self.create_publisher(JointState, "/r2d3/sim/cmd/head", QoSProfile(depth=10))

    def _on_img(self, m):
        self.latest = m
        self.count += 1

    def cmd_head(self, pan, tilt):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ["head_joint1", "head_joint2"]
        js.position = [float(pan), float(tilt)]
        self.pub.publish(js)

    def frame(self):
        m = self.latest
        if m is None or not m.width:
            return None
        ch = 4 if m.encoding == "rgba8" else 3
        return np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, ch)[:, :, :3].copy()


def main():
    rclpy.init()
    n = Demo()
    # wait for the first frame
    t0 = time.time()
    while rclpy.ok() and n.latest is None and time.time() - t0 < 15:
        rclpy.spin_once(n, timeout_sec=0.1)
    if n.latest is None:
        print("[rt] no camera frames — is bring_up --demo-workspace running?", flush=True)
        return

    from PIL import Image as PImage, ImageDraw
    shots = []
    n.count = 0
    rate_t0 = time.time()
    for pan in PANS:
        # stream the command for DWELL_S so the position drive reaches it
        t1 = time.time()
        while rclpy.ok() and time.time() - t1 < DWELL_S:
            n.cmd_head(pan, TILT)
            rclpy.spin_once(n, timeout_sec=0.05)
        fr = n.frame()
        shots.append((pan, fr))
        bright = "-" if fr is None else f"mean={fr.mean():.0f}"
        print(f"[rt] pan={pan:+.2f} rad -> captured frame ({bright})", flush=True)
    elapsed = time.time() - rate_t0
    hz = n.count / elapsed if elapsed > 0 else 0
    print(f"[rt] live /camera/color: {n.count} frames in {elapsed:.1f}s = {hz:.1f} Hz", flush=True)

    # filmstrip: 2 rows x 3 cols, downscaled, labeled with the commanded pan
    cols, rows, tw, th = 3, 2, 320, 240
    sheet = PImage.new("RGB", (cols * tw, rows * th), (20, 20, 20))
    dr = ImageDraw.Draw(sheet)
    for i, (pan, fr) in enumerate(shots):
        if fr is None:
            continue
        im = PImage.fromarray(fr, "RGB").resize((tw, th))
        x, y = (i % cols) * tw, (i // cols) * th
        sheet.paste(im, (x, y))
        dr.text((x + 6, y + 6), f"head pan = {pan:+.2f} rad", fill=(255, 255, 0))
    sheet.save(OUT / "cam_realtime_sweep.png")
    print(f"[rt] wrote {OUT/'cam_realtime_sweep.png'} (live D435 sweeping the workspace)", flush=True)
    print("[rt] DONE", flush=True)
    n.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
