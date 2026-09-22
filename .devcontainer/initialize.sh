#!/usr/bin/env bash
# devcontainer.json `initializeCommand`: runs on the HOST, before VS Code (or
# the devcontainer CLI) reads the compose files. Bash 3.2-safe, like ./droid.
#
# Two jobs, both delegated to ./droid so the dev container never grows its own
# copy of the tier/overlay decision table:
#
# 1. Write docker-compose.resolved.yml from `./droid render`: the base compose
#    file plus whichever overlays ./droid would select on this machine (nvidia
#    tier when the GPU probe passes; rogent when opted in), interpolated with
#    this host's uid/gid. docker-compose.dev.yml then overlays the parts that
#    make it a separate container (see that file). The output is git-ignored:
#    it is a per-machine artefact, exactly like ./droid's resolved config.
#
# 2. Pre-create the shared named volumes with compose's own project labels, so
#    the dev project can mount them as `external` while the r2d3 project
#    (./droid) still owns them -- whichever of the two runs first on a fresh
#    clone, and with no "created for a different project" warnings from either.
#
# Opt-ins are read from the environment, and from .devcontainer/local.env
# (git-ignored) because a desktop-launched editor does not inherit your shell:
#   DROID_GPU_OVERRIDE=cpu       skip the GPU tier (same as ./droid up --gpu cpu)
#   DROID_DEVCONTAINER_ROGENT=1  compose the rogent overlay (./droid up --rogent)
#   DROID_ROGENT_SRC=<path>      the rogent-v3 checkout for that overlay
set -eu

here="$(cd "$(dirname "$0")" && pwd)"
repo_root="$(cd "$here/.." && pwd)"
out="$here/docker-compose.resolved.yml"

if [ -f "$here/local.env" ]; then
  set -a
  # shellcheck disable=SC1091
  . "$here/local.env"
  set +a
fi

render_args=""
[ "${DROID_DEVCONTAINER_ROGENT:-}" = "1" ] && render_args="--rogent"

# Rendered to a temp file first, so a failed render (unreachable GPU, missing
# rogent checkout -- ./droid prints the remediation) leaves the previous file
# intact rather than a truncated one. $render_args is deliberately unquoted:
# it is either empty or a single flag.
# shellcheck disable=SC2086
"$repo_root/droid" render $render_args > "$out.tmp"
mv "$out.tmp" "$out"

project="$(sed -n 's/^name: //p' "$out" | head -1)"
[ -n "$project" ] || { echo "initialize.sh: rendered compose has no project name" >&2; exit 1; }

# Top-level `volumes:` is the last section `docker compose config` emits; each
# entry is `  <key>:` followed by `    name: <volume name>`.
sed -n '/^volumes:/,$p' "$out" | sed -n 's/^  \([A-Za-z0-9_-]*\):$/\1/p' | while read -r key; do
  name="$(sed -n '/^volumes:/,$p' "$out" | sed -n "/^  $key:\$/,/^  [^ ]/s/^    name: //p" | head -1)"
  [ -n "$name" ] || continue
  if ! docker volume inspect "$name" >/dev/null 2>&1; then
    docker volume create \
      --label "com.docker.compose.project=$project" \
      --label "com.docker.compose.volume=$key" \
      "$name" >/dev/null
    echo "initialize.sh: created shared volume $name (project $project)"
  fi
done

echo "initialize.sh: wrote $out"
