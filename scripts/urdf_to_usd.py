#!/usr/bin/env python3
"""Convert the R2D3 75-B URDF into a USD asset for Isaac Sim 5.1.

MUST be run inside the Isaac Sim container:

    docker compose -f Docker/isaac/compose.isaac.yaml run --rm isaac-sim \\
        /isaac-sim/python.sh /workspace/r2d3_isaac/scripts/urdf_to_usd.py

Cannot run on the host: `isaacsim.*` / `omni.*` modules only exist inside
the Isaac Sim Python environment.

See docs/urdf_to_usd.md for the full configuration rationale.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_URDF = (
    REPO_ROOT
    / "ros2_rm_robot/dual_rm_description/dual_rm_75b_description"
    / "urdf/dual_rm_75b_description.urdf"
)
DEFAULT_USD = REPO_ROOT / "isaac_sim/usd/r2d3.usd"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--urdf",
        type=Path,
        default=DEFAULT_URDF,
        help="Path to the source URDF (default: dual_rm_75b_description.urdf).",
    )
    parser.add_argument(
        "--usd",
        type=Path,
        default=DEFAULT_USD,
        help="Path to write the output USD (default: isaac_sim/usd/r2d3.usd).",
    )
    parser.add_argument(
        "--merge-fixed-joints",
        action="store_true",
        help="Collapse links connected by fixed joints (initially OFF for visibility).",
    )
    parser.add_argument(
        "--convex-decomp",
        action="store_true",
        help="Decompose convex meshes; enable only if grasping shows mesh artifacts.",
    )
    args = parser.parse_args()

    if not args.urdf.is_file():
        print(f"error: URDF not found: {args.urdf}", file=sys.stderr)
        return 1
    args.usd.parent.mkdir(parents=True, exist_ok=True)
    if args.usd.exists():
        print(f"warn: {args.usd} exists; deleting (importer is non-idempotent).")
        args.usd.unlink()

    # Isaac Sim bootstrap — keep imports inside main so module-level import
    # doesn't fail when someone runs this on the host by accident.
    from isaacsim import SimulationApp

    sim_app = SimulationApp({"headless": True})

    import omni.kit.commands  # noqa: E402

    status, cfg = omni.kit.commands.execute("URDFCreateImportConfig")
    if not status:
        print("error: URDFCreateImportConfig failed", file=sys.stderr)
        sim_app.close()
        return 2

    # See docs/urdf_to_usd.md for rationale per field.
    cfg.fix_base = False
    cfg.merge_fixed_joints = args.merge_fixed_joints
    cfg.convex_decomp = args.convex_decomp
    cfg.import_inertia_tensor = True
    cfg.self_collision = False
    cfg.density = 0.0
    cfg.default_drive_type = 1  # POSITION
    cfg.default_drive_strength = 1e7
    cfg.default_position_drive_damping = 1e5
    cfg.distance_scale = 1.0  # URDF is in meters
    cfg.up_vector = (0.0, 0.0, 1.0)
    cfg.create_physics_scene = True
    cfg.make_default_prim = True
    cfg.collision_from_visuals = False  # URDF has explicit collision meshes
    cfg.replace_cylinders_with_capsules = False
    cfg.parse_mimic = True  # required for gripper finger mimic (V2 once gripper joints are authored)
    cfg.override_joint_dynamics = False

    print(f"importing: {args.urdf}")
    print(f"      to: {args.usd}")
    status, prim_path = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=str(args.urdf),
        import_config=cfg,
        dest_path=str(args.usd),
        get_articulation_root=True,
    )
    if not status:
        print("error: URDFParseAndImportFile failed", file=sys.stderr)
        sim_app.close()
        return 3

    print(f"imported robot at prim: {prim_path}")
    print(f"USD written:           {args.usd}")
    sim_app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
