# Training environments (scenes)

R2D3 can be dropped into furnished environments for training and evaluation.
`isaac_sim/r2d3_sim/scenes.py` loads a scene; pass it through the `R2D3` setup hook:

```python
from isaac_sim.r2d3_sim import R2D3, scenes

sim = R2D3(mobile=True, setup=lambda w: scenes.load("kitchen", w))
sim.reset()
# scenes.load(...) returns {"spawn": (x, y, yaw), "look": ..., "eye": ...}
```

Render a screenshot of any scene with the robot in it:

```bash
scripts/isaacsim_ros2.sh isaac_sim/tests/diag_scenes.py --scene warehouse
scripts/isaacsim_ros2.sh isaac_sim/tests/diag_scenes.py --scene kitchen
scripts/isaacsim_ros2.sh isaac_sim/tests/diag_scenes.py --scene living_room
# -> isaac_sim/tests/captures/scene_<name>.png
```

## The three scenes

| Scene | Built from | Notes |
|---|---|---|
| **warehouse** | NVIDIA prebuilt `full_warehouse.usd` | shelving, pallets, forklifts, floor markings — a complete real scene |
| **living_room** | Isaac **Office** furniture props (`SM_Sofa`, `SM_Armchair`, `SM_Plant`, table) + rug | real SimReady-quality props |
| **kitchen** | primitive cabinetry (counter run, island, fridge, upper cabinets, stovetop) + a live plant | see the asset note below |

Assets are streamed from NVIDIA's **cloud** asset server (`get_assets_root_path()`),
so the machine needs internet — no local Nucleus required. First load of the
warehouse takes a minute (it pulls many sub-assets).

> **Robot framing in the warehouse:** the prebuilt warehouse is large, densely
> racked, and its floor isn't at z=0, which makes a *headless, blind* third-person
> screenshot of the robot inside it finicky (the robot settles on the floor fine,
> but origin is occluded by racks and good camera/spawn coordinates are best picked
> interactively in the GUI). The kitchen + living-room screenshots show the robot
> clearly; for the warehouse, tune `scenes.load_warehouse`'s `spawn`/`look`/`eye`
> (or view it in the Kit UI with `headless=False`) to a clear aisle.

### Asset note (kitchen)

The kitchen cabinetry is built from primitives rather than the
`SimReady/Residential/Kitchen` USDs. Those assets carry varied internal
z-offsets and didn't render reliably headless (the prims load but the visual
meshes don't appear). The Office furniture props (used for the living room) and
the prebuilt warehouse render correctly. To swap the photoreal kitchen assets
back in once the SimReady rendering is resolved, replace the `add_visual_box`
calls in `scenes.load_kitchen` with `_ref(...)` to the SimReady USDs (the paths
are in git history) and let the per-asset floor-snap in `diag_scenes.py` place them.

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
- **Cluttered tabletop** — dense graspable objects (Isaac `Props/YCB/`): the RL grasping workhorse.
- **Domain-randomized variants** of every scene — randomize lighting, textures, and object poses (Isaac Replicator) for sim-to-real robustness.
- **Multi-room apartment** — kitchen + living room + bedroom + hallway: long-horizon mobile manipulation.

Add a scene by writing a `load_<name>(world)` in `scenes.py` (return `spawn` +
optional `look`/`eye` camera hints) and registering it in `_SCENES`.
