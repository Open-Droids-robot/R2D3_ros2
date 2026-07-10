"""Shared helpers for the R2D3 SDK + examples.

Dedups the boilerplate that was copy-pasted across ``isaac_sim/tests/*``
(prim lookup, world poses, quaternion conversions, RGBA handling, lighting,
GIF writing). Pure-Python where possible; omni/pxr/PIL imports are deferred
into the functions so this module is importable *before* SimulationApp boots.
"""
from __future__ import annotations

import math
from typing import Optional

import numpy as np

# ---------------------------------------------------------------------------
# Quaternions  (convention: w, x, y, z unless noted)
# ---------------------------------------------------------------------------
TOP_DOWN_R = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], dtype=float)


def mat_to_quat(R) -> np.ndarray:
    """3x3 rotation matrix -> quaternion (w, x, y, z)."""
    R = np.asarray(R, dtype=float)
    tr = float(np.trace(R))
    if tr > 0:
        s = 0.5 / math.sqrt(tr + 1.0)
        return np.array([0.25 / s, (R[2, 1] - R[1, 2]) * s,
                         (R[0, 2] - R[2, 0]) * s, (R[1, 0] - R[0, 1]) * s])
    i = int(np.argmax(np.diag(R)))
    if i == 0:
        s = 2 * math.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2])
        return np.array([(R[2, 1] - R[1, 2]) / s, 0.25 * s,
                         (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s])
    if i == 1:
        s = 2 * math.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2])
        return np.array([(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s,
                         0.25 * s, (R[1, 2] + R[2, 1]) / s])
    s = 2 * math.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1])
    return np.array([(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s,
                     (R[1, 2] + R[2, 1]) / s, 0.25 * s])


def quat_wxyz_to_xyzw(q) -> np.ndarray:
    q = np.asarray(q, dtype=float)
    return np.array([q[1], q[2], q[3], q[0]])


def quat_xyzw_to_wxyz(q) -> np.ndarray:
    q = np.asarray(q, dtype=float)
    return np.array([q[3], q[0], q[1], q[2]])


def yaw_quat(deg: float) -> np.ndarray:
    """Quaternion (wxyz) for a yaw of ``deg`` degrees about +Z."""
    h = math.radians(deg) / 2.0
    return np.array([math.cos(h), 0.0, 0.0, math.sin(h)])


def top_down_quat() -> np.ndarray:
    """Orientation (wxyz) for a top-down grasp: the arm-hand link's +X points
    down (world -Z). Matches the orientation used in the grasp examples."""
    return mat_to_quat(TOP_DOWN_R)


# ---------------------------------------------------------------------------
# Stage lookups
# ---------------------------------------------------------------------------
def _stage():
    import omni.usd
    return omni.usd.get_context().get_stage()


def find_prim(name: str, typ: Optional[str] = "Xform"):
    """First stage prim whose name == ``name`` (and type == ``typ`` if given)."""
    for p in _stage().Traverse():
        if p.GetName() == name and (typ is None or p.GetTypeName() == typ):
            return p
    return None


def prim_path(name: str, typ: Optional[str] = "Xform") -> Optional[str]:
    p = find_prim(name, typ)
    return p.GetPath().pathString if p is not None else None


def world_pose(name_or_prim):
    """World pose of a prim (given by name or prim) -> (xyz np[3], quat_wxyz np[4])."""
    from pxr import UsdGeom, Gf
    prim = name_or_prim if hasattr(name_or_prim, "GetPath") else find_prim(name_or_prim, None)
    if prim is None:
        return np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0])
    m = UsdGeom.XformCache().GetLocalToWorldTransform(prim)
    o = m.Transform(Gf.Vec3d(0, 0, 0))
    q = m.ExtractRotationQuat()
    return (np.array([o[0], o[1], o[2]], dtype=float),
            np.array([q.GetReal(), *q.GetImaginary()], dtype=float))


def world_position(name_or_prim) -> np.ndarray:
    return world_pose(name_or_prim)[0]


# ---------------------------------------------------------------------------
# Images / lighting / GIF
# ---------------------------------------------------------------------------
def rgba_to_rgb(a) -> np.ndarray:
    """Strip the alpha channel if present; return uint8 HxWx3."""
    a = np.asarray(a)
    if a.ndim == 3 and a.shape[2] == 4:
        a = a[:, :, :3]
    return a.astype(np.uint8)


def set_lighting(dome: Optional[float] = None, key: Optional[float] = None,
                 fill: Optional[float] = None) -> None:
    """Override the intensities of the lights authored by scene._add_lighting.
    Pass only the ones you want to change (others left as-is)."""
    from pxr import UsdLux
    stage = _stage()
    for path, val in (("/DomeLight", dome), ("/KeyLight", key), ("/FillLight", fill)):
        if val is None:
            continue
        lp = stage.GetPrimAtPath(path)
        if lp:
            UsdLux.LightAPI(lp).GetIntensityAttr().Set(float(val))


class GifWriter:
    """Collect frames (resized) and save an animated GIF."""

    def __init__(self, size: tuple[int, int] = (480, 270)):
        self._size = size
        self._frames = []

    def add(self, rgb: np.ndarray) -> None:
        from PIL import Image
        self._frames.append(Image.fromarray(rgba_to_rgb(rgb)).resize(self._size))

    def __len__(self) -> int:
        return len(self._frames)

    def save(self, path, duration: int = 80) -> None:
        if not self._frames:
            return
        self._frames[0].save(str(path), save_all=True,
                             append_images=self._frames[1:], duration=duration, loop=0)


class Mp4Writer:
    """Stream frames straight to an H.264 MP4 via the system ffmpeg.

    Mirrors :class:`GifWriter` (``add`` / ``len`` / ``save``) but encodes a
    clean, high-resolution video instead of a low-res GIF. Frames are piped to
    ffmpeg as they arrive (rawvideo rgb24 -> libx264 yuv420p) so memory stays
    flat even for long reels — nothing is buffered in Python. ``yuv420p`` +
    ``+faststart`` make the file play everywhere (browsers, slides, QuickTime).

    No extra Python deps: we shell out to whatever ffmpeg is on PATH.
    """

    def __init__(self, path, size: tuple[int, int] = (1280, 720),
                 fps: int = 30, crf: int = 18):
        # libx264 needs even dimensions; round down defensively.
        self._w = int(size[0]) - (int(size[0]) % 2)
        self._h = int(size[1]) - (int(size[1]) % 2)
        self._path = str(path)
        self._fps = int(fps)
        self._crf = int(crf)
        self._proc = None
        self._n = 0

    def _open(self) -> None:
        import shutil
        import subprocess
        ff = shutil.which("ffmpeg") or "/usr/bin/ffmpeg"
        cmd = [
            ff, "-y", "-loglevel", "error",
            "-f", "rawvideo", "-pix_fmt", "rgb24",
            "-s", f"{self._w}x{self._h}", "-r", str(self._fps), "-i", "-",
            "-an", "-c:v", "libx264", "-pix_fmt", "yuv420p",
            "-crf", str(self._crf), "-preset", "slow",
            "-movflags", "+faststart", self._path,
        ]
        self._proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)

    def add(self, rgb: np.ndarray) -> None:
        from PIL import Image
        img = Image.fromarray(rgba_to_rgb(rgb))
        if img.size != (self._w, self._h):
            img = img.resize((self._w, self._h), Image.LANCZOS)
        if self._proc is None:
            self._open()
        self._proc.stdin.write(img.tobytes())
        self._n += 1

    def __len__(self) -> int:
        return self._n

    def save(self, *args, **kwargs) -> None:
        """Flush and finalize the MP4. Args accepted for GifWriter parity."""
        if self._proc is None:
            return
        self._proc.stdin.close()
        self._proc.wait()
        self._proc = None
