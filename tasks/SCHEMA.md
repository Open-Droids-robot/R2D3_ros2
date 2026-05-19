# Task YAML Schema (V1)

> Authoritative reference for files in `tasks/<slug>.yaml`. Consumed by `r2d3_eval` and the sim-side `r2d3_engine`.

Files in this directory describe one task instance each. They are **declarative** — the engine reads them and assembles the scene, sets up scoring, and dispatches the `RunTask` action. No Python is required to add a new task (though tricky goal predicates may need a custom evaluator hook).

## Top-level keys

| Key | Type | Required | Description |
|---|---|---|---|
| `id` | string | ✓ | Slug; must match the YAML filename without extension. Sent to participant via `RunTask.goal.task_id`. |
| `display_name` | string | ✓ | Human-readable name shown in reports and the leaderboard. |
| `description` | string | | One-paragraph rationale (what the task probes). |
| `version` | string | ✓ | Semver. Bump when changing scoring or goal semantics so old policy results stay comparable to themselves. |
| `time_limit_seconds` | int | ✓ | Wall-clock budget per trial. Engine cancels the action at this limit. |
| `robot` | object | ✓ | See **Robot section**. |
| `scene` | object | ✓ | See **Scene section**. |
| `goal` | object | ✓ | See **Goal section**. |
| `scoring` | object | ✓ | See **Scoring section**. |
| `randomization` | object | | Optional: extra envelope for domain randomization (textures, lighting). V1 uses object-spawn randomization in the scene section. |

## Robot section

Controls the starting state of R2D3 before the action goal is sent.

```yaml
robot:
  arms_used: [right_arm]            # subset of {left_arm, right_arm}
  home:
    body_lift_mm: 1000              # 0–2600
    left_arm_joints:                # rad; 7 entries (l_joint1..l_joint7)
      - 0.0
      - -0.5
      - 0.0
      - -1.2
      - 0.0
      - 0.7
      - 0.0
    right_arm_joints:               # rad; 7 entries (r_joint1..r_joint7)
      - 0.0
      - -0.5
      - 0.0
      - -1.2
      - 0.0
      - 0.7
      - 0.0
    head:                           # optional; defaults to (0, 0)
      pan_rad: 0.0                  # joint range ±1.256
      tilt_rad: 0.0                 # joint range ±0.419
  start_gripper_position:           # optional; default 1000 (fully open ~70 mm)
    left: 1000
    right: 1000
```

**Notes**
- `arms_used` is **advisory**, not enforced. A unimanual task can still receive bimanual policies; scoring only credits goal satisfaction.
- `home` is restored deterministically by the engine via `/reset_joints` service before each trial.
- Joint limits are validated against the URDF at engine startup; out-of-range values fail loudly.

## Scene section

Declarative scene description. Assets are spawned by the engine at trial start and removed at trial end.

```yaml
scene:
  ground_plane: true                # always; default true
  table:                            # optional; many tasks need a workspace
    name: workbench
    pose: { x: 0.5, y: 0.0, z: 0.7, roll: 0, pitch: 0, yaw: 0 }
    size: { x: 1.0, y: 0.6, z: 0.05 }    # axis-aligned bounding box
    material: wood_oak              # asset library key
  objects:                          # list of manipulable bodies
    - name: cube_red
      type: primitive_cube          # primitive_cube | primitive_cylinder | primitive_sphere | asset
      size: 0.04                    # m, for cubes
      mass: 0.05                    # kg
      friction: 0.7                 # μ
      restitution: 0.0
      color: red                    # named or "#RRGGBB"
      spawn:
        type: uniform               # uniform | fixed
        seed_source: task_seed      # always task_seed for V1
        x: [0.4, 0.6]               # range or single value
        y: [-0.2, 0.2]
        z: 0.74                     # often fixed (rest on table)
        yaw: [-1.57, 1.57]          # rotation about world Z
    # ...
  distractors:                      # optional non-task objects in the scene
    - name: cube_blue
      type: primitive_cube
      ...
```

**Pose conventions**
- World frame: Z up, X forward (matches Isaac Sim / RViz defaults).
- All poses are in the **world** frame unless `frame` is specified.
- Angles in radians.

**Asset types**
- `primitive_cube`, `primitive_cylinder`, `primitive_sphere` — generated procedurally.
- `asset` (V1 supports a small library: see `isaac_sim/r2d3_sim/assets.py`). For V2 we'll allow `usd_path: ".../<asset>.usd"` referencing arbitrary USD.

## Goal section

A **deterministic predicate** evaluated against simulation ground truth (NOT against participant-reported state). The engine signals task completion via the action result.

Currently supported predicate types:

### `object_in_zone`

```yaml
goal:
  type: object_in_zone
  object: cube_red                  # must match a scene.objects[].name
  zone:
    pose: { x: 0.7, y: 0.0, z: 0.74 }
    tolerance_xy: 0.05              # m, planar
    tolerance_z: 0.02               # m, vertical
  hold_seconds: 1.0                 # must remain inside for this long
```

### `stacked`

```yaml
goal:
  type: stacked
  order: [cube_red, cube_green, cube_blue]   # top-of-list = bottom of stack
  alignment_tolerance: 0.02         # m; horizontal offset between consecutive blocks
  hold_seconds: 1.0
```

### `held_by_arm`

```yaml
goal:
  type: held_by_arm
  object: cube_red
  arm: right_arm                    # left_arm | right_arm | either
  hold_seconds: 2.0
```

### `bimanual_handoff`

```yaml
goal:
  type: bimanual_handoff
  object: cube_red
  start_arm: right_arm
  end_arm: left_arm
  intermediate_hold_seconds: 0.5    # object must be held by start_arm first
  end_hold_seconds: 1.0             # then by end_arm
```

### `reached_pose`

```yaml
goal:
  type: reached_pose
  arm: right_arm
  target: { x: 0.6, y: 0.1, z: 0.9, roll: 0, pitch: -1.57, yaw: 0 }
  tolerance_xyz: 0.02
  tolerance_rot: 0.1                # rad
  hold_seconds: 0.5
```

Custom predicates go in `isaac_sim/r2d3_sim/goal_predicates.py` and register via decorator (V1 has the five above; V2 will add `cable_inserted`, `tool_used`, etc.).

## Scoring section

Two tiers, evaluated **independently**. Tier 2 only contributes if Tier 1 passes — but Tier 1 failure still produces a (low) tier1_score in the report.

### Tier 1 — liveness / submission validity

Cheap monitors that verify the participant is participating.

```yaml
scoring:
  tier1:
    min_observation_rate_hz: 10     # Observation msg subscribers must keep up
    min_command_rate_hz: 5          # at least one of the controller topics must publish
    max_observation_gap_seconds: 1  # no single gap longer than this
    activate_within_seconds: 5      # lifecycle node must transition unconfigured→active
```

A trial **fails Tier 1** if any of these thresholds is breached. `tier1_score` is `1.0 - (1 - passing_topic_fraction)`.

### Tier 2 — task quality

Only evaluated if Tier 1 passes. Weighted sum of components:

```yaml
scoring:
  tier2:
    success_bonus: 100              # awarded if goal predicate is satisfied within time_limit
    penalties:
      completion_time:
        weight: -1.0                # subtract 1.0 × seconds taken
        unit: s
      path_length:
        weight: -0.5
        unit: m
        target_arm: right_arm       # which TCP to measure
      max_grip_force:
        weight: -0.1
        unit: N
        cap: 50                     # only penalize forces above this
      off_limit_contact:
        weight: -25.0
        pairs:                      # robot-to-X contacts that incur penalty
          - [robot, workbench]
          - [robot, cube_blue]      # specifically: don't bump the distractor
```

`tier2_score = success_bonus_if_satisfied + Σ penalty_weight × measured_value`.

### Total score

`total_score = tier2_score` *if* Tier 1 passes, *else* `tier1_score × 10`. Reported in `RunTask.result.total_score`.

## Determinism

Every randomized element draws from `task_seed`, sent via `RunTask.goal.task_seed`. The same `(task_id, task_seed, trial_index)` triple **must** produce the same scene and the same goal outcome (given a deterministic policy). The engine asserts this via a CRC check of the spawned scene transform tree at trial start.

## Versioning

Bump `version:` when:
- A goal predicate changes (different success criterion)
- Scoring weights change (tier2 penalty tweaks)
- Scene topology changes (new distractors, table size)

Cosmetic changes (descriptions, formatting) don't require a version bump.

## Validation

`r2d3_eval validate tasks/<slug>.yaml` runs schema validation and dry-runs scene spawn against the URDF. Run this in CI before merging task changes.
