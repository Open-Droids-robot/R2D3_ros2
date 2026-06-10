"""Verify the R2D3 sensor suite end-to-end (run in the ros_humble env).

Subscribes to every sensor stream + the aggregated Observation for a few
seconds, then reports rates + key fields, and saves the live head-camera
color frame to a PNG. Expects Isaac bring_up + r2d3_humble_bridge running.
"""
import sys
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo, JointState
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Bool
from r2d3_model_interfaces.msg import Observation

OUT = Path("/usr1/home/semathew/r2d3_isaac/isaac_sim/tests/captures")


class V(Node):
    def __init__(self):
        super().__init__("verify_sensors")
        self.count = {}
        self.last = {}
        def sub(typ, topic, qos=10):
            self.create_subscription(typ, topic, lambda m, k=topic: self._rx(k, m), qos)
        sub(Image, "/camera/color/image_raw", qos_profile_sensor_data)
        sub(Image, "/camera/depth/image_raw", qos_profile_sensor_data)
        sub(CameraInfo, "/camera/color/camera_info", qos_profile_sensor_data)
        sub(CameraInfo, "/camera/depth/camera_info", qos_profile_sensor_data)
        sub(WrenchStamped, "/r2d3/sim/wrench/left")
        sub(WrenchStamped, "/r2d3/sim/wrench/right")
        sub(JointState, "/r2d3/sim/joint_states")
        sub(Observation, "/r2d3/observations")

    def _rx(self, k, m):
        self.count[k] = self.count.get(k, 0) + 1
        self.last[k] = m


def main():
    rclpy.init()
    n = V()
    import time as _t
    start = n.get_clock().now()
    # collect ~6 s
    while rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.1)
        if (n.get_clock().now() - start).nanoseconds / 1e9 > 6.0:
            break

    print("\n===== SENSOR VERIFICATION =====")
    for k in ["/camera/color/image_raw", "/camera/depth/image_raw",
              "/camera/color/camera_info", "/camera/depth/camera_info",
              "/r2d3/sim/wrench/left", "/r2d3/sim/wrench/right",
              "/r2d3/sim/joint_states", "/r2d3/observations"]:
        c = n.count.get(k, 0)
        print(f"  {k:34s} msgs={c:4d}  (~{c/6.0:.0f} Hz)")

    ci = n.last.get("/camera/color/camera_info")
    if ci:
        print(f"\n  color camera_info: {ci.width}x{ci.height} frame={ci.header.frame_id} "
              f"fx={ci.k[0]:.1f} fy={ci.k[4]:.1f} cx={ci.k[2]:.1f} cy={ci.k[5]:.1f}")

    wl = n.last.get("/r2d3/sim/wrench/left")
    wr = n.last.get("/r2d3/sim/wrench/right")
    if wl:
        f = wl.wrench.force; t_ = wl.wrench.torque
        print(f"  left  wrench: F=({f.x:.2f},{f.y:.2f},{f.z:.2f}) "
              f"T=({t_.x:.3f},{t_.y:.3f},{t_.z:.3f}) frame={wl.header.frame_id}")
    if wr:
        f = wr.wrench.force; t_ = wr.wrench.torque
        print(f"  right wrench: F=({f.x:.2f},{f.y:.2f},{f.z:.2f}) "
              f"T=({t_.x:.3f},{t_.y:.3f},{t_.z:.3f}) frame={wr.header.frame_id}")

    obs = n.last.get("/r2d3/observations")
    if obs:
        print(f"\n  Observation.head_color:       {obs.head_color.width}x{obs.head_color.height} "
              f"enc={obs.head_color.encoding!r}")
        print(f"  Observation.head_depth:       {obs.head_depth.width}x{obs.head_depth.height} "
              f"enc={obs.head_depth.encoding!r}")
        print(f"  Observation.head_camera_info: {obs.head_camera_info.width}x{obs.head_camera_info.height} "
              f"fx={obs.head_camera_info.k[0]:.1f}")
        lf = obs.left_wrench.wrench.force; rf = obs.right_wrench.wrench.force
        print(f"  Observation.left_wrench.F=({lf.x:.2f},{lf.y:.2f},{lf.z:.2f}) "
              f"right_wrench.F=({rf.x:.2f},{rf.y:.2f},{rf.z:.2f})")
        print(f"  Observation.joint_states: {len(obs.joint_states.name)} joints; "
              f"lift_state.height={obs.lift_state.height} mm")

    # Save live color frame
    col = n.last.get("/camera/color/image_raw")
    if col and col.width and col.encoding in ("rgb8", "rgba8"):
        ch = 4 if col.encoding == "rgba8" else 3
        arr = np.frombuffer(bytes(col.data), np.uint8).reshape(col.height, col.width, ch)[:, :, :3]
        try:
            from PIL import Image as PILImage
            PILImage.fromarray(arr, "RGB").save(OUT / "live_head_rgb.png")
        except Exception:
            import matplotlib.pyplot as plt
            plt.imsave(str(OUT / "live_head_rgb.png"), arr)
        print(f"\n  saved live head frame -> {OUT/'live_head_rgb.png'} "
              f"(min={arr.min()} max={arr.max()})")
    else:
        print(f"\n  live color frame not saved (enc={col.encoding if col else 'none'})")

    print("===== DONE =====", flush=True)
    n.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
