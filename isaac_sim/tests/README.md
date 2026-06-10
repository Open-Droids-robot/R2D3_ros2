# tests / verification

Launch through the wrapper: `scripts/isaacsim_ros2.sh isaac_sim/tests/<script>.py`.

| Script | Purpose |
|---|---|
| `smoke_sdk.py` | end-to-end SDK smoke — boot, control, cameras, sensing |
| `diag_all_joints.py` | command **every DOF** + read back motion (`--ee dexterous\|gripper`, `--mobile`); prints PASS/FAIL, exits non-zero on failure |
| `diag_motion_gif.py` | third-person **motion GIF** for visual verification (`--ee`) → `tests/captures/` |
| `grasp_lift_ik.py` | full IK grasp-and-lift demo with GIF (`--ee`) — predates the SDK facade |
| `move_task.py` | mobile-base drive + 90° rotate demo with GIF |

`diagnostics/` holds ~27 archived one-off probes from initial bring-up — kept for
reference, not maintained. `captures/` (gitignored) is where GIFs/PNGs land.

For clean, SDK-based usage demos see [`../examples/`](../examples/).
