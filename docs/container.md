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

## 1a. The desktop

What the browser shows is a single 1920x1080 virtual screen (`DROID_GEOMETRY`
overrides it) running fluxbox, tiled into four fixed, peer quadrants so that
everything is usable at the viewer's default `resize=scale` size:

```
+-------------------------+-------------------------+
| simulator               | RViz                    |
| (MuJoCo viewer / Gazebo)|                         |
+-------------------------+-------------------------+
| rogent agent  (goal>)   | shell                   |
| rogent mode only        | ROS + workspace sourced |
+-------------------------+-------------------------+
```

- **Simulator** and **RViz** are placed by the fluxbox `apps` file
  (`container/fluxbox-apps`, matched on window class, installed into
  `~/.fluxbox/apps` on every container start by `container/gui-start.sh`).
- **Shell** (lower right, both modes) is an `xterm` running an interactive
  bash with `/opt/ros/jazzy` and, once the first launch has built it,
  `/ws/install` sourced — `ros2 topic list`, `ros2 action list`,
  `ros2 topic echo /clock` all work from the browser.
- **Agent** (lower left, rogent mode only) is an `xterm` running
  `container/rogent-run.sh --wait`: it prints "waiting for the simulation
  clock to advance" until the sim is really up, then starts the rogent REPL at
  its `goal>` prompt. Type a goal there exactly as you would in `./droid
  rogent`. `./droid rogent` from the host is the same script without `--wait`
  and still works alongside the pane (a second REPL is fine).

The two terminals are supervised by `gui-start.sh`, which respawns a pane
that is closed (Ctrl-D, or the REPL exiting), so the browser view never loses
its way back in. Click a pane to give it keyboard focus before typing.

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
| `./droid up [--mujoco] [--rogent] [--gpu <tier>] [--recreate]` | Ensures the image, starts the container, rebuilds the simulation subset, launches the simulation, prints the noVNC URL. `--mujoco` selects the MuJoCo backend instead of the default Gazebo. `--rogent` provisions the rogent agent mode (see §13). `--gpu <tier>` overrides platform detection (`cpu` or `nvidia`). `--recreate` consents to recreating the container when its configuration has drifted. | `--recreate` destroys container-local installs; otherwise nothing. |
| `./droid rogent [args…]` | Opens an interactive rogent agent session (the text-goal REPL) in a container provisioned with `up --rogent`, after verifying the simulation clock is advancing. Extra arguments go to rogent's `main.py`. | Nothing. |
| `./droid shell` | Opens a shell in the running container. | Nothing. |
| `./droid doctor` | Re-runs the platform probe and prints raw + resolved values. | Nothing — read-only, though it does attempt a real `docker run --gpus all` (see below). |
| `./droid resolve` | Prints the resolved configuration as `key=value` lines. Pure — no side effects. | Nothing. |
| `./droid render [--rogent] [--gpu <tier>]` | Prints the fully rendered compose configuration `up` would apply (`docker compose config` of the base file plus the resolved tier/mode overlays, host uid/gid interpolated). Runs the same probe as `doctor`; touches no container. Consumed by the dev container (§11). | Nothing. |
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
everywhere: an edit under `src/` does not exist until `colcon build` copies it
into `install/`, whether you're running natively or in the container.
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
| amd64, software-rendering tier, Gazebo | **Hand-verified** — `./droid up` reaches a running sim: `/clock` advancing, five controllers active, Gazebo + RViz rendered in the browser over noVNC |
| amd64, software-rendering tier, MuJoCo | **Hand-verified** — `./droid up --mujoco` reaches a running sim on a warm cache hit: `/clock` advancing, six controllers active, MuJoCo viewer + RViz in the browser |
| amd64 image build + workspace compile | Verified locally (the image builds and the 15-package subset compiles inside it) |
| amd64 and arm64 image build in CI | Expected to pass; this branch has not yet been pushed, so CI has not actually run it yet |
| Rogent mode (`up --rogent --mujoco`, zenoh graph, agent pane, `./droid rogent`) | **Hand-verified** (amd64, software tier) — the derived image builds, the zenoh bringup reaches an advancing `/clock`, the desktop shows MuJoCo, RViz, the agent pane at `goal>` and the shell pane, and a typed goal runs the loop end-to-end against host Ollama |
| Default mode desktop panes (`up --mujoco`) | **Hand-verified** — MuJoCo, RViz and the shell pane; `ros2 topic list` works in the pane |
| NVIDIA accelerated tier | **Not verified** |
| Jetson | **Not verified** |
| arm64 at runtime | **Not booted** — only expected to build once CI runs |
| macOS desktop experience | **Not verified** |
| Windows / WSL2 | Not supported — deliberately out of scope |

The two hand-verified rows were exercised on the reference amd64 machine, whose
GPU is unreachable from Docker, so `--gpu cpu` was used. Treat every row that is
not marked hand-verified as provisional until it is independently checked; in
particular, the accelerated tier, Jetson, arm64 runtime, and macOS are unproven.

## 11. Optional dev container

`.devcontainer/` lets VS Code or Cursor "Reopen in Container" from a clean
clone and land in a working workspace: the same image, the same tier, the
same shared build volumes as `./droid up`, with `ros2` and the workspace
sourced in every integrated terminal. It is strictly optional — `./droid
shell` gets you the same environment without an editor integration.

### What happens on "Reopen in Container"

1. **`initializeCommand`** (host, before compose runs) runs
   `.devcontainer/initialize.sh`, which calls **`./droid render`** and writes
   its output to `.devcontainer/docker-compose.resolved.yml` (git-ignored, a
   per-machine artefact). That is the base compose file plus whichever
   overlays `./droid up` would select on this machine — `nvidia` when the GPU
   probe passes, `rogent` when opted in — interpolated with your uid/gid.
   There is no second copy of the tier decision in JSON: the dev container
   composes what `./droid` resolves, including the hard failure on an NVIDIA
   GPU that Docker cannot acquire (§4). The script also pre-creates the five
   shared volumes with compose's own labels (see below).
2. **Compose** brings up `docker-compose.resolved.yml` +
   `docker-compose.dev.yml`. The dev overlay is where the setup deliberately
   diverges from `./droid up` — see the next subsection.
3. **`postCreateCommand`** (in the container, once per creation, as `droid`)
   runs `.devcontainer/post-create.sh`: appends a block to `~/.bashrc` that
   sources `/opt/ros/jazzy/setup.bash` and `/ws/install/setup.bash`, then
   runs a plain `colcon build` from `/ws` — the simulation subset, **no
   `--symlink-install`**, exactly what every `./droid up` launch path does.
   The build is not repeated on later starts; rebuild yourself
   (`cd /ws && colcon build --packages-select <pkg>`) and remember the
   install-space copy trap applies here as everywhere.
4. Terminals open as `droid` (never root — files you create in the
   bind-mounted tree stay yours), in `/ws/src/R2D3_ros2`, with ROS sourced.
   The Python and C++ extensions are pointed at `/opt/ros/jazzy` and
   `/ws/install`.

`shutdownAction` is `none`: closing the editor leaves the container running,
like `./droid down` never happened; stop it with `docker stop r2d3-dev`.

### How it coexists with `./droid up` — the intentional divergence

The dev container is a **separate compose project and container**
(`r2d3-dev`, container `r2d3-dev`), not the `r2d3-sim` container `./droid`
manages. Two tools running `compose up` against one container would each see
the other's configuration (VS Code's own labels and metadata on one side, the
drift fingerprint on the other) as a change and recreate it — destroying
container-local installs on every switch. Keeping the projects distinct means
neither tool can ever address, recreate or stop the other's container, and
`./droid up`'s drift gate (§7) only ever inspects `r2d3-sim`, as before.

What the two **share** are the five named volumes: `build/`, `install/`,
`log/`, the MuJoCo cache and downloaded weights. A build in the editor is what
`./droid up` launches next, and vice versa. `initialize.sh` creates them with
compose's project labels so the `r2d3` project still owns them: `./droid up`
adopts them silently on a fresh clone, and `./droid nuke` removes them — which
fails while `r2d3-dev` still has them mounted, so `docker rm -f r2d3-dev`
first if you really mean to nuke.

What the two do **not** share:

- **The display.** Both containers run the GUI stack; the dev container's
  noVNC is published on **6081** (`DROID_DEV_NOVNC_PORT` to change), so it
  coexists with `./droid up` on 6080.
- **The ROS graph.** The image pins `ROS_LOCALHOST_ONLY=1`, so a simulation
  launched by `./droid up` is invisible to a terminal in the dev container.
  To work against a running sim from the editor, launch it *in the dev
  container*: `/opt/droid/launch-sim.sh mujoco` (or `gz`) from an integrated
  terminal — it rebuilds first, like `./droid up` — and watch it at
  `http://localhost:6081/vnc.html?autoconnect=1&resize=scale`. Do not run
  both sims at once: they would build into, and launch from, the same
  install volume.
- **The lifecycle.** "Rebuild Container" in the editor is the dev container's
  equivalent of `./droid up --recreate`; the fingerprint label on `r2d3-dev`
  reads `devcontainer` because nothing reads it.

### Opting in and out

`initialize.sh` reads the environment and, because a desktop-launched editor
does not inherit your shell, an optional git-ignored `.devcontainer/local.env`:

```
DROID_GPU_OVERRIDE=cpu        # skip the GPU tier, like ./droid up --gpu cpu
DROID_DEVCONTAINER_ROGENT=1   # compose the rogent overlay (§13)
DROID_ROGENT_SRC=/path/to/rogent-v3
```

`container/.env` (§12) applies too: `./droid render` inlines it into the
rendered file, so after editing it run "Rebuild Container" for the change to
reach the dev container. Changing any of these, or a `git pull` that touches
the compose files, is a configuration change: rebuild the container. Nothing
here alters the image — `container/Dockerfile` is untouched by the dev
container, and `./droid up` still never rebuilds an existing image.

### Verification status

The pieces are covered by the static guards in `container/test/` and were
exercised individually on the reference amd64 machine: `./droid render` for
the cpu, nvidia and rogent selections; the merged dev project through
`docker compose config` and the devcontainer CLI's `read-configuration`;
`post-create.sh` in a throwaway container from the published image (15
packages built, a fresh login shell had `ros2` on PATH with the workspace
sourced, a single-package rebuild worked). The live "Reopen in Container"
path from a clean clone has **not** been hand-verified; the checklist is in
the pull request that introduced it.

## 12. Credentials and caching

`container/.env` is read if present, is git-ignored, and is **not**
generated for you — copy `container/env.example` if you want one. Nothing
is required in it for weight caching: downloaded model weights are cached
on a volume and persist across `./droid down` / `./droid up` cycles (though
not across `./droid nuke`).

## 13. Rogent mode

`./droid up --rogent` provisions an opt-in agent mode: the
[rogent-v3](https://github.com/Open-Droids-robot/rogent-v3) agent running
against this simulation, entirely on local models. It is an overlay in the
same pattern as the nvidia tier — the default mode's composed service is
untouched, and toggling `--rogent` on or off is a configuration change that
requires `--recreate`.

What the overlay changes:

- **Image** — a derived image (`container/Dockerfile.rogent`) adds rogent's
  python dependencies, Kokoro TTS and `paplay` on top of the base image.
- **Transport** — `RMW_IMPLEMENTATION=rmw_zenoh_cpp` container-wide, with one
  supervised `rmw_zenohd` router started at container start; every launch in
  rogent mode gates on the router actually listening on `:7447`.
- **Rogent source** — bind-mounted from `DROID_ROGENT_SRC` (default: a
  `rogent-v3` checkout beside this repo). The paired repo + ref is pinned in
  `rogent.repos`; a fresh machine materialises it once with
  `cd <parent of R2D3_ros2> && vcs import --input R2D3_ros2/rogent.repos .`
  If the checkout is missing, `up --rogent` refuses with that remediation —
  it never clones on its own.
- **Reachability** — `host.docker.internal` maps to the host (Ollama serves
  models there on `:11434`), and the host Pulse socket is mounted so speech is
  audible on the host's speakers. On hosts without a Pulse socket, speech
  degrades and everything else works; rogent mode is Linux-first. Add
  `--mute` to `./droid up --rogent` to route that playback to a null sink
  (`droid_mute`, created on the host on demand) — the speech pipeline and its
  events still run, only the sound is silent. Useful for repeated test runs.

The workflow: `./droid up --rogent --mujoco` provisions and launches the sim,
and the agent pane on the noVNC desktop (§1a) comes up at the rogent `goal>`
prompt once the simulation clock is advancing — the browser view is
self-sufficient. `./droid rogent` from a host terminal opens a second,
identical REPL; both go through `container/rogent-run.sh`, which owns the
sim-ready guard (the clock must be advancing, not merely present) and the
sourced ROS environment rclpy needs. Local models are the no-keys fallback:
with Google API keys present in rogent's environment it uses Gemini, with no
keys it runs fully local.

## 14. Naming

Factory AI ships a CLI that is also called `droid`. Running `./droid` from
the repository root is unambiguous regardless of what else is on your
`PATH`. Putting this repo's `droid` on your `PATH` is optional, and if you
do, be aware of that name collision.

## 15. Troubleshooting

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
