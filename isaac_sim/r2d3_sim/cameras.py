"""In-process camera access for the R2D3 SDK — RGB + depth as numpy arrays,
without needing ROS.

Wraps ``sensors.ensure_camera_prims`` with persistent omni.replicator render
products + annotators (the pattern the diag/capture scripts copy-paste). Bakes
in the recurring gotchas: RTX needs warm-up frames before the first read,
annotator RGB is RGBA (strip alpha), and depth (``distance_to_camera``) is
float32 metres with inf for no-hit.
"""
from __future__ import annotations

import numpy as np

from . import sensors as _sensors
from . import helpers as h


class CameraRig:
    """Render products + rgb/depth annotators for the head + wrist D435s."""

    def __init__(self, resolution: tuple[int, int] = (640, 480), depth: bool = True):
        self._res = resolution
        self._want_depth = depth
        self._prims: dict[str, str] = {}
        self._rgb: dict[str, object] = {}
        self._depth: dict[str, object] = {}
        self._products = []
        self._warmed = False

    def attach(self) -> dict:
        """Author the Camera prims (head + l_wrist + r_wrist) and create one
        render product + annotators per camera. Idempotent."""
        import omni.replicator.core as rep
        if self._prims:
            return self._prims
        self._prims = _sensors.ensure_camera_prims()
        for name, prim in self._prims.items():
            rp = rep.create.render_product(prim, self._res)
            ra = rep.AnnotatorRegistry.get_annotator("rgb")
            ra.attach(rp)
            self._rgb[name] = ra
            if self._want_depth:
                da = rep.AnnotatorRegistry.get_annotator("distance_to_camera")
                da.attach(rp)
                self._depth[name] = da
            self._products.append(rp)
        return self._prims

    @property
    def names(self):
        return list(self._prims.keys())

    @property
    def warmed(self) -> bool:
        return self._warmed

    def warmup(self, world, steps: int = 20) -> None:
        """Step the world (rendering) so RTX populates the render products —
        the first frames are otherwise black/stale."""
        for _ in range(steps):
            world.step(render=True)
        self._warmed = True

    def get_rgb(self, name: str = "head") -> np.ndarray:
        """RGB frame as uint8 HxWx3."""
        return h.rgba_to_rgb(self._rgb[name].get_data(do_array_copy=True))

    def get_depth(self, name: str = "head") -> np.ndarray:
        """Depth frame as float32 HxW in metres (0 where no geometry)."""
        d = np.asarray(self._depth[name].get_data(do_array_copy=True), dtype=np.float32)
        if d.ndim == 3 and d.shape[2] == 1:
            d = d[:, :, 0]
        return np.nan_to_num(d, posinf=0.0, neginf=0.0)

    def detach(self) -> None:
        for ann in list(self._rgb.values()) + list(self._depth.values()):
            try:
                ann.detach()
            except Exception:  # noqa: BLE001
                pass
        for rp in self._products:
            try:
                rp.destroy()
            except Exception:  # noqa: BLE001
                pass
        self._rgb.clear()
        self._depth.clear()
        self._products.clear()
        self._prims.clear()
