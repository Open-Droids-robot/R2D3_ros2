"""Capture clean D435 sensor feeds in a populated scene, for the showcase
'sensors' slide. The stock 02_cameras.py grabs the cameras in the empty
bring-up scene (so the head just sees white void); here we load the KITCHEN,
pose the robot at the island exactly like the perception demo, and grab RGB +
colourised depth for each of the three D435s:

  * cam_head_rgb.png     / cam_head_depth.png     head D435
  * cam_l_wrist_rgb.png  / cam_l_wrist_depth.png  left-wrist D435
  * cam_r_wrist_rgb.png  / cam_r_wrist_depth.png  right-wrist D435

(depth is colourised near->far)

    scripts/isaacsim_ros2.sh isaac_sim/tests/sensor_views.py
"""
from pathlib import Path

import numpy as np
from PIL import Image

from isaac_sim.r2d3_sim import R2D3
from isaac_sim.r2d3_sim import helpers as h
from isaac_sim.r2d3_sim import scenes

OUT = Path(__file__).resolve().parents[1] / "tests" / "captures"
BASE_Z = 0.25                                    # wheels on the floor (matches demo 09)


def colorize_depth(depth: np.ndarray) -> np.ndarray:
    """Map a metric depth image to an 8-bit RGB 'turbo'-style near->far ramp.
    Invalid (<=0) pixels render dark grey so they read as 'no return'."""
    d = np.asarray(depth, dtype=np.float32)
    valid = d > 0
    if not valid.any():
        return np.zeros((*d.shape, 3), np.uint8)
    lo, hi = np.percentile(d[valid], 2), np.percentile(d[valid], 98)
    t = np.clip((d - lo) / max(1e-6, hi - lo), 0.0, 1.0)
    # 5-stop perceptual ramp: navy -> blue -> cyan -> yellow -> red (near=navy)
    stops = np.array([[0.10, 0.10, 0.45], [0.13, 0.36, 0.92], [0.16, 0.82, 0.78],
                      [0.95, 0.86, 0.18], [0.78, 0.16, 0.13]], np.float32)
    xs = np.linspace(0, 1, len(stops))
    rgb = np.stack([np.interp(t, xs, stops[:, c]) for c in range(3)], axis=-1)
    rgb[~valid] = np.array([0.18, 0.18, 0.20], np.float32)
    return (rgb * 255).astype(np.uint8)


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    man = {}
    sim = R2D3(end_effector="dexterous", mobile=True, headless=True,
               enable_cameras=True, camera_resolution=(1280, 960),
               setup=lambda w: man.update(scenes.load("kitchen", w)))
    try:
        sim.reset()
        sim.go_home()                            # arms at home -> the wrist D435s frame the hands
        sim.set_lift(man.get("lift", 0.90))
        sx, sy, yaw = man["spawn"]
        sim.set_base_pose(np.array([sx + 0.35, sy, BASE_Z]), h.yaw_quat(yaw))
        sim.set_head(0.0, 0.0)
        sim.step(n=60)
        sim.cameras.warmup(sim.world, steps=16)

        for cam in sim.cameras.names:            # 'head', 'l_wrist', 'r_wrist'
            rgb, depth = sim.get_image(cam, depth=True)
            Image.fromarray(rgb).save(OUT / f"cam_{cam}_rgb.png")
            Image.fromarray(colorize_depth(depth)).save(OUT / f"cam_{cam}_depth.png")
            dmin = float(depth[depth > 0].min()) if (depth > 0).any() else 0.0
            print(f"[sensors] {cam}: rgb {rgb.shape} mean={rgb.mean():.0f}  "
                  f"depth min={dmin:.2f} m", flush=True)
        print("[sensors] DONE", flush=True)
        return 0
    finally:
        sim.close()


if __name__ == "__main__":
    raise SystemExit(main())
