# `tasks/`

Task definitions for the V1 evaluation suite. Each task is a YAML descriptor (initial conditions, success criteria, time limit, scoring rubric) + an optional Python hook for any task-specific setup that can't be expressed declaratively.

Planned tasks:

| Slug | Description |
|---|---|
| `pick_and_place` | Single-arm pick of a 4 cm cube, transport to target zone |
| `stacking` | Single-arm stacking of 3 distinguishable cubes |
| `bimanual_handoff` | Pass an object from left arm to right arm without dropping |
| `vision_guided_grasp` | Top-down grasp of an object placed at a random pose within the workspace, using D435 RGB-D only |

Each task is consumed by `r2d3_eval` (see `../r2d3_eval/`) for deterministic scoring.
