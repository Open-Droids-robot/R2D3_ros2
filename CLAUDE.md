# CLAUDE.md

Guidance for AI agents working in the R2D3_ros2 repository.

## Build model: rebuild before trusting anything

This workspace is built **without** `--symlink-install`, so `install/` contains
plain copies. Everything is consumed from the install space at runtime —
`$(find pkg)` xacro includes, `xacro.load_yaml()` configs, launch files, worlds,
params. **An edit under `src/` does not exist until `colcon build` copies it.**

```bash
colcon build --packages-select <pkg> && source install/setup.bash
```

This bites hardest on plain data files, because it does not look like a build
problem: change a YAML, relaunch, and the old value silently stays in force. The
symptom is "this setting does nothing", which invites debugging the code rather
than the build. Before investigating any config-driven behaviour that appears
inert, rebuild and re-check — it costs a second and rules out the most common
cause.

The same trap invalidates test and launch results, so rebuild before believing
either. See `docs/agents/domain.md` for the wider context.

## The container

`./droid up` runs the whole simulation in a container (ROS 2 Jazzy, GUI over
noVNC at `http://localhost:6080`). The repository is bind-mounted, so it is the
developer's working tree that runs — but **the container's build semantics are
identical to the host's**: no `--symlink-install`, `install/` holds plain copies,
and the same stale-install trap applies. The container neutralises it by
rebuilding the simulation subset on every launch path rather than by changing
build semantics, so the rule above is unchanged wherever code runs. Build
artifacts live on volumes, separate from any native `build/` and `install/` in
the working tree. See `docs/container.md`.

## Agent skills

### Issue tracker

Issues and PRDs are tracked in this repo's GitHub Issues (via the `gh` CLI). See `docs/agents/issue-tracker.md`.

### Triage labels

Default canonical triage vocabulary: `needs-triage`, `needs-info`, `ready-for-agent`, `ready-for-human`, `wontfix`. See `docs/agents/triage-labels.md`.

### Domain docs

Single-context: one `CONTEXT.md` + `docs/adr/` at the repo root. See `docs/agents/domain.md`.
