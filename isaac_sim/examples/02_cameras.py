"""Grab the head + wrist cameras as numpy (RGB + depth) and save PNGs.

    scripts/isaacsim_ros2.sh isaac_sim/examples/02_cameras.py
"""
from pathlib import Path

import numpy as np
from PIL import Image
from isaac_sim.r2d3_sim import R2D3

OUT = Path(__file__).resolve().parents[1] / "tests" / "captures"


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    with R2D3(end_effector="dexterous", headless=True, enable_cameras=True) as sim:
        sim.reset()
        sim.set_head(0.0, -0.3)      # look down at the workspace
        sim.step(n=20)

        for cam in sim.cameras.names:          # 'head', 'l_wrist', 'r_wrist'
            rgb, depth = sim.get_image(cam, depth=True)
            Image.fromarray(rgb).save(OUT / f"cam_{cam}_rgb.png")
            # normalize depth (metres) to a viewable 8-bit image
            d = np.clip(depth, 0, 3.0); d = (255 * d / 3.0).astype(np.uint8)
            Image.fromarray(d).save(OUT / f"cam_{cam}_depth.png")
            print(f"[cameras] {cam}: rgb {rgb.shape} mean={rgb.mean():.0f}  "
                  f"depth {depth.shape} min={depth[depth>0].min() if (depth>0).any() else 0:.2f} m")
    print(f"[cameras] wrote PNGs to {OUT}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
