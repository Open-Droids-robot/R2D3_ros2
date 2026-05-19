# `isaac_sim/`

Isaac-Sim-side code for the R2D3 hackathon platform.

| Subdir | Contents |
|---|---|
| `usd/` | R2D3 USD asset (converted from `../ros2_rm_robot/dual_rm_description/` URDFs) |
| `scenes/` | One scene script per eval task (Pick & Place, Stacking, Bimanual Handoff, Vision-Guided Grasp) |
| `r2d3_sim/` | Python package wrapping scene / robot / sensor APIs used by `tasks/` and `r2d3_eval/` |
| `tests/` | pytest suite for the Python wrappers |

All code here is **container-only** — it imports `omni.*` / `isaacsim.*` modules that exist inside the Isaac Sim container (`Docker/isaac/`). Don't try to run it on the host.
