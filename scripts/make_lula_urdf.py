#!/usr/bin/env python3
"""Make a portable, mesh-free URDF for Lula IK.

The rendered flat URDFs reference meshes by absolute ``file:///.../install/...``
paths (machine-specific, and the meshes live in gitignored build output). Lula's
kinematics solver only needs the joint tree + the ``*_lula.yaml`` collision
spheres — NOT the URDF meshes — so we strip every ``<mesh .../>`` down to a tiny
``<box/>``. The result has zero external dependencies, is a few KB, and is what
``r2d3_sim.ik.ArmIK`` loads, so the SDK's IK works on any clone.

    scripts/make_lula_urdf.py in.urdf out.urdf
"""
import re
import sys
from pathlib import Path

_BOX = '<box size="0.001 0.001 0.001"/>'


def strip_meshes(src: str) -> str:
    src = re.sub(r"<mesh\b[^>]*?/>", _BOX, src)                  # self-closing
    src = re.sub(r"<mesh\b[^>]*?>.*?</mesh>", _BOX, src, flags=re.S)  # paired
    return src


def main() -> int:
    if len(sys.argv) != 3:
        print("usage: make_lula_urdf.py <in.urdf> <out.urdf>", file=sys.stderr)
        return 1
    inp, outp = Path(sys.argv[1]), Path(sys.argv[2])
    out = strip_meshes(inp.read_text())
    outp.write_text(out)
    print(f"wrote {outp} ({len(out)} bytes, meshes stripped)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
