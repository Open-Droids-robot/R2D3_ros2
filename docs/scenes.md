# Training environments (scenes)

R2D3 can be dropped into furnished, **interactable** environments — each with
manipulable objects already on its surfaces — for training and evaluation.
`isaac_sim/r2d3_sim/scenes.py` loads a scene; pass it through the `R2D3` setup hook:

```python
from isaac_sim.r2d3_sim import R2D3, scenes

man = {}
sim = R2D3(mobile=True, setup=lambda w: man.update(scenes.load("kitchen", w)))
sim.reset()
# scenes.load(...) returns a manifest:
#   man["objects"]   -> {"mug": "/World/objs/mug", ...}   manipulable rigid bodies
#   man["surface_z"] -> {"mug": 0.85, ...}                the surface each rests on
#   man["spawn"]/["look"]/["eye"]/["lift"]                robot placement + camera hints
```

Render a screenshot, or verify the objects are interactable (numeric, no render):

```bash
scripts/isaacsim_ros2.sh isaac_sim/tests/diag_scenes.py --scene kitchen           # screenshot
scripts/isaacsim_ros2.sh isaac_sim/tests/diag_scenes.py --scene kitchen --check --no-render
# screenshots -> isaac_sim/tests/captures/scene_<name>.png
# --check verifies every object settled on its surface + the robot is stable (exit 1 on fail)
```

To grasp an object, see **`isaac_sim/examples/08_kitchen_manipulation.py`** (picks the
mug off the island). For an **ML-perception-driven** pick-and-place — an
open-vocabulary detector (OWL-ViT) finds the mug in the head camera, the pixel is
unprojected to 3D, and the robot picks + places it — see
**`isaac_sim/examples/09_kitchen_clear_island.py`** (needs `pip install transformers`).

## The three scenes

| Scene | Built from | Objects on it |
|---|---|---|
| **warehouse** | NVIDIA prebuilt `warehouse.usd` (shelving, floor markings) | cardboard boxes + a tote on the floor |
| **living_room** | Office props (`SM_Sofa`/`SM_Armchair`/`SM_Plant`) + rug + a collidable coffee table | banana, mug, sugar box on the table |
| **kitchen** | collidable cabinetry (counter run, island, fridge, cabinets, stovetop) | mug, bowl, cracker box, soup can, mustard on the worktops |

Assets stream from NVIDIA's **cloud** asset server (`get_assets_root_path()`), so the
machine needs internet — no local Nucleus required.

## Manipulable objects

Every scene seeds rigid-body objects (Isaac YCB props + cardboard boxes) on its
surfaces. `scenes.load(...)` returns them in the manifest:

```python
man = scenes.load("kitchen", world)
man["objects"]     # {"mug": "/World/objs/mug", "bowl": ..., "soup_can": ...}
man["surface_z"]   # {"mug": 0.85, ...}  (None = rests on the scene floor)
```

Objects live under `/World/objs/<name>`, are made dynamic rigid bodies
(`scenes.add_object` applies a collider + mass if the asset lacks one), and settle
onto **collidable** surfaces (`scene.add_fixed_box` counters/island/table, or the
warehouse floor). Grasp one with the SDK's IK + a fixed-joint weld — see
`isaac_sim/examples/08_kitchen_manipulation.py`. Add your own with
`scenes.add_object(manifest, name, usd, pos, surface_z=..., mass=...)`.

> **Surfaces vs floors:** counters/island/coffee-table are collidable
> (`add_fixed_box`); the composed-room floors/walls are visual-only (the kinematic
> base is re-pinned each step, and a collidable ground would perturb the
> articulation). Objects rest on the fixed surfaces, not the room floor.

> **Warehouse note:** uses the single-shelf `warehouse.usd` (open floor at the
> origin) rather than `full_warehouse.usd` — the full warehouse is densely racked,
> which made headless robot placement finicky. Both render the same way; swap the
> USD in `scenes.load_warehouse` if you want the larger layout (and tune the spawn).

### Asset note (kitchen)

The kitchen cabinetry is built from **collidable primitives** (`scene.add_fixed_box`)
rather than the `SimReady/Residential/Kitchen` USDs. Those assets carry varied
internal z-offsets and didn't render reliably headless (the prims load but the
visual meshes don't appear). The Office furniture props (living room) and the
prebuilt warehouse render correctly. To swap the photoreal kitchen assets back in
once the SimReady rendering is resolved, replace the `add_fixed_box` calls in
`scenes.load_kitchen` with `scenes.add_object(...)`/`_ref(...)` to the SimReady USDs
(paths are in git history). The island is intentionally low (worktop 0.85 m) so the
robot's resting arms clear it at the close grasp spawn.

## Recommended additional scenes

**Prebuilt — drop the robot straight in (like warehouse), high reliability:**
- **Office** (`Environments/Office/office.usd`) — desks, meeting rooms, open plan: mobile manipulation + navigation.
- **Hospital** (`Environments/Hospital/hospital.usd`) — corridors + rooms: delivery, door traversal, long-horizon navigation.
- **Outdoor / Rivermark** (`Environments/Outdoor/Rivermark/rivermark.usd`) — outdoor navigation, lighting variation.
- **Warehouse variants** (`warehouse_with_forklifts.usd`, `warehouse_multiple_shelves.usd`) — logistics + dynamic-obstacle avoidance.

**Composed (build like kitchen / living room):**
- **Retail / grocery aisle** — shelving + products: pick-from-shelf, restocking.
- **Dining room** — table + chairs + tableware: bimanual table setting/clearing (plays to R2D3's dual arms).
- **Bedroom** — bed, nightstand, wardrobe: tidying, fetch-and-place.
- **Garage / workshop** — workbench, tools, shelving: tool use, sorting.
- **Loading dock** — pallets, boxes, conveyor: the composite-lift use case.
- **Lab bench** — precise bimanual manipulation.

**Training-specific:**
- **Cluttered tabletop** — denser piles of the YCB props already used here (`Props/YCB/`): the RL grasping workhorse.
- **Domain-randomized variants** of every scene — randomize lighting, textures, and object poses (Isaac Replicator) for sim-to-real robustness.
- **Multi-room apartment** — kitchen + living room + bedroom + hallway: long-horizon mobile manipulation.

Add a scene by writing a `load_<name>(world)` in `scenes.py` (return `spawn` +
optional `look`/`eye` camera hints) and registering it in `_SCENES`.
