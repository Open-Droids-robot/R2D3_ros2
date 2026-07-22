# The container

The canonical guide to running the R2D3 simulation in a container. This
replaces the old `Docker/` tree entirely.

## 1. What this is, and the one command

```bash
./droid up
```

That is the whole workflow. It detects your platform, starts a single
container, rebuilds the simulation subset, launches Gazebo with RViz, and
prints a URL. Open it in a browser:

```
http://localhost:6080/vnc.html?autoconnect=1&resize=scale
```

and you will see the robot. Prerequisites are **bash and Docker**, nothing
else — no ROS 2 install, no GPU drivers on the host, no X11 setup. The same
command and the same URL work on an amd64 Linux desktop, an Apple Silicon
Mac, a Jetson, or a headless cloud instance, because the GUI is delivered
over noVNC rather than through the host's display stack.

## 2. The two tiers, and their different ambitions

`./droid up` resolves to one of two rendering tiers:

- **`nvidia`** — hardware-accelerated rendering via the NVIDIA Container
  Toolkit. This is the **recommended tier and where real work happens**.
- **`cpu`** — software rendering (llvmpipe). This is the default whenever no
  usable NVIDIA GPU is detected, and it is deliberately universal: it is what
  runs on a Mac, a machine with no GPU at all, or a machine where GPU
  passthrough isn't set up.

The software tier's goal is **to be alive, not fast**. The simulation drives
four RGBD cameras and a GPU lidar; under software rendering that is slow, and
slowness there is expected behaviour, not a defect. Its success criterion is
simply: the simulation runs, the clock advances, and the robot is visible.
Do not judge the `cpu` tier by frame rate — judge the `nvidia` tier by that.

## 3. Commands

| Command | What it does | What it preserves / destroys |
|---|---|---|
| `./droid up [--mujoco] [--gpu <tier>] [--recreate]` | Ensures the image, starts the container, rebuilds the simulation subset, launches the simulation, prints the noVNC URL. `--mujoco` selects the MuJoCo backend instead of the default Gazebo. `--gpu <tier>` overrides platform detection (`cpu` or `nvidia`). `--recreate` consents to recreating the container when its configuration has drifted. | `--recreate` destroys container-local installs; otherwise nothing. |
| `./droid shell` | Opens a shell in the running container. | Nothing. |
| `./droid doctor` | Re-runs the platform probe and prints raw + resolved values. | Nothing — read-only, though it does attempt a real `docker run --gpus all` (see below). |
| `./droid resolve` | Prints the resolved configuration as `key=value` lines. Pure — no side effects. | Nothing. |
| `./droid down` | Stops the container. | Preserves anything installed inside it (e.g. `sudo apt install`), shell history, and scratch files. |
| `./droid nuke` | Destroys the container **and its volumes**, requires typing `nuke` to confirm. | Destroys container-local installs, build artifacts, the MuJoCo cache, and any downloaded model weights. |

## 4. Platform detection

Detection is a **probe, not an inspection**. `./droid doctor` and `./droid up`
don't just read `nvidia-smi` and guess — they actually attempt
`docker run --gpus all` and act on whether that succeeds. This matters: an
NVIDIA GPU can be visible to the host while still unreachable from Docker.

If an NVIDIA GPU is detected but Docker cannot acquire it, the tool
**hard-fails** rather than silently falling back to software rendering (which
would otherwise mean days of unexplained slowness). The failure prints
remediation:

```
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo nvidia-ctk cdi generate --output=/etc/cdi/nvidia.yaml
sudo systemctl restart docker
```

followed by re-running `./droid doctor`. If you'd rather proceed on software
rendering without fixing anything, there's an explicit escape hatch:

```bash
./droid up --gpu cpu
```

Machines with no GPU at all — including every macOS host, since Docker
Desktop has no GPU passthrough to Linux containers — resolve to `cpu` with no
error.

`./droid resolve` answers "why did it pick this?" — it prints the resolved
tier and every input that fed the decision, with no side effects, so you can
diagnose a surprising choice without running anything destructive.

## 5. Editing code

The working tree is bind-mounted into the container. Edits you make on the
host take effect immediately inside it — there is no image rebuild step for
source changes. Build artifacts (`build/`, `install/`, `log/`) live on
Docker volumes, kept strictly separate from any native build you may also
have on the host, so the two never collide. Files you create from inside the
container are owned by you on the host, whatever your host uid — the
container remaps to match.

## 6. The rebuild guarantee

Every path through `./droid up` rebuilds the simulation subset **first**, so
a YAML or xacro edit takes effect on the very next launch — there is no
separate "remember to rebuild" step to forget.

The container deliberately keeps the host's **non-symlink** install
semantics: `install/` inside the container holds plain copies too, exactly
as it does on the host. This is on purpose, so there is one mental model
everywhere and the existing rebuild-trap documentation (see the repository's
`CLAUDE.md`) stays true whether you're running natively or in the container.
The rebuild itself is cheap, because the ignored packages (below) are the
bulk of the workspace and the simulation subset is almost entirely data
packages.

## 7. Persistence and destruction

- `./droid down` stops the container but preserves anything you did inside
  it: `sudo apt install`s, shell history, scratch files. Your next
  `./droid up` picks up where you left off.
- `./droid nuke` is the explicit destructive reset. It requires typing
  `nuke` to confirm, and removes the container and its volumes — including
  container-local installs, build artifacts, the MuJoCo cache, and cached
  model weights.
- A configuration change (a different resolved tier, an edited compose file,
  a new image) refuses to act silently — it requires `./droid up --recreate`.
  Recreating destroys container-local installs the same way `nuke` does.

If there's something you install inside the container that you want to keep
permanently, don't rely on it surviving — promote it into `container/Dockerfile`
so your teammates get it too.

## 8. MuJoCo cold start

The image ships a pre-warmed converter venv and a pre-warmed MJCF cache. That
cache is keyed content-addressably over the generated robot description plus
the world file, so it is a cache **hit only for an unmodified checkout at the
image's revision**.

If you're on a feature branch, or you've edited the robot description
locally, the content hash changes and the first `./droid up --mujoco`
launch triggers a full multi-minute reconversion. That is correct behaviour,
not a hang — let it finish; subsequent launches from the same checkout state
are fast again.

## 9. What is built, and what is not

The container builds the **15 simulation packages**. **14 hardware packages**
are excluded via a colcon defaults file baked into the image at
`/etc/colcon/defaults.yaml` (`container/colcon-defaults.yaml` in this repo) —
not via `COLCON_IGNORE` markers in the tree, because the tree is
bind-mounted: runtime-created markers would pollute the developer's git
status, and committed ones would change the *host's* native build too.

The ignored packages are hardware-only and have no simulation dependents:
the ZED wrapper, the RealSense driver and message/description packages, the
AGV robot package, the Woosh message packages, the arm driver, and the
object-detection demo. There's no camera SDK or vendor hardware to run a
simulation against, so these simply don't build inside the container.

Nav2 and MoveIt **are** built into the image and are launchable by hand from
`./droid shell` — they are not wired up as `./droid` subcommands.

## 10. Verification status

Stated honestly:

| Path | Status |
|---|---|
| amd64, software-rendering tier, Gazebo and MuJoCo | Exercised during development on this machine; the formal hand-verification pass has not yet been run at the time of writing |
| amd64 and arm64 image build + workspace compile | Expected to pass in CI; this branch has not yet been pushed, so CI has not actually run it yet |
| NVIDIA accelerated tier | **Not verified** |
| Jetson | **Not verified** |
| arm64 at runtime | **Not booted** — only expected to build once CI runs |
| macOS desktop experience | **Not verified** |
| Windows / WSL2 | Not supported — deliberately out of scope |

Treat every row above the "Not supported" line as provisional until it is
independently re-checked; do not read "exercised during development" as
equivalent to a completed verification pass.

## 11. Optional dev container

A `.devcontainer/` is provided for "Reopen in Container" in VS Code or
Cursor. It is strictly optional — `./droid shell` gets you the identical
environment without an editor integration.

## 12. Credentials and caching

`container/.env` is read if present, is git-ignored, and is **not**
generated for you — copy `container/env.example` if you want one. Nothing
is required in it for weight caching: downloaded model weights are cached
on a volume and persist across `./droid down` / `./droid up` cycles (though
not across `./droid nuke`).

## 13. Naming

Factory AI ships a CLI that is also called `droid`. Running `./droid` from
the repository root is unambiguous regardless of what else is on your
`PATH`. Putting this repo's `droid` on your `PATH` is optional, and if you
do, be aware of that name collision.

## 14. Troubleshooting

**GPU unreachable** (NVIDIA GPU detected but Docker can't acquire it):
run `./droid doctor` to see the diagnosis and remediation commands, or skip
straight to software rendering with `./droid up --gpu cpu`.

**Port 6080 already in use**: something else on the host is bound to 6080.
Stop it, or stop any other `droid`/compose stack that's already running
(`./droid down` in the other checkout).

**"the resolved configuration has changed" / drift refusal**: your platform
probe resolved differently than when the container was created (different
tier, an edited compose file, a new image). Run `./droid resolve` to see
what changed. If you're fine losing container-local installs, re-run with
`./droid up --recreate`; otherwise `./droid shell` keeps using the existing
container unchanged.

**MuJoCo reconversion takes minutes**: expected on a feature branch or with
local description edits — see §8. It is not a hang.

**Software-rendering tier feels slow**: expected — see §2. If you have an
NVIDIA GPU and expected the `nvidia` tier, run `./droid doctor` to see why
it didn't resolve that way.
