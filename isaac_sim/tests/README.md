# tests / verification

Launch through the wrapper: `scripts/isaacsim_ros2.sh isaac_sim/tests/<script>.py`.

| Script | Purpose |
|---|---|
| `smoke_sdk.py` | end-to-end SDK smoke — boot, control, cameras, sensing |
| `diag_all_joints.py` | command **every DOF** + read back motion (`--ee dexterous\|gripper`, `--mobile`); prints PASS/FAIL, exits non-zero on failure |
| `diag_motion_gif.py` | third-person **motion GIF** for visual verification (`--ee`) → `tests/captures/` |
| `grasp_lift_ik.py` | full IK grasp-and-lift demo with GIF + 720p MP4 (`--ee`) — predates the SDK facade |
| `move_task.py` | mobile-base drive + 90° rotate demo with GIF + MP4 |

Showcase capture scripts (render clean 720p MP4s + stills into `captures/`):

| Script | Purpose |
|---|---|
| `motion_reel.py` | eased full-body motion reel — every subsystem in one smooth take (`--ee`) |
| `gripper_pickup.py` | single-arm parallel-gripper pickup — jaws physically close on the cup (verified contact) |
| `pick_place_drive.py` | mobile pick-place-drive — grasp at the island, carry across, place on a second table (`--ee`) |
| `sensor_views.py` | clean RGB + colourised-depth feeds from all three D435s in the kitchen scene |

`diagnostics/` holds ~27 archived one-off probes from initial bring-up — kept for
reference, not maintained. `captures/` (gitignored) is where GIFs/PNGs/MP4s land.

For clean, SDK-based usage demos see [`../examples/`](../examples/).
