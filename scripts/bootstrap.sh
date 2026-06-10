#!/usr/bin/env bash
# One-command setup for the R2D3 Isaac Sim platform (the in-process SDK path).
#
#   bash scripts/bootstrap.sh
#
# Creates the `isaac` conda env (Python 3.12), installs Isaac Sim 6.0 + the R2D3
# SDK (editable), checks the robot assets, and runs the smoke test. Idempotent —
# safe to re-run; it skips anything already in place.
#
# The ROS bridge env (`ros_humble`, RoboStack) is OPTIONAL and not created here —
# the SDK doesn't need it. See docs/bridge.md if you want the ROS path.
#
# Overrides: R2D3_CONDA_BASE / R2D3_ISAAC_ENV / R2D3_ISAAC_PY (see _conda_env.sh).
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=scripts/_conda_env.sh
source "$REPO/scripts/_conda_env.sh"
r2d3_resolve_envs
ISAAC_ENV="${R2D3_ISAAC_ENV:-isaac}"

# conda/mamba front-end
RUNNER=""
for c in mamba micromamba conda; do command -v "$c" >/dev/null 2>&1 && { RUNNER="$c"; break; }; done
if [[ -z "$RUNNER" ]]; then
  echo "error: need conda / mamba / micromamba on PATH. Install Miniforge:" >&2
  echo "       https://github.com/conda-forge/miniforge" >&2
  exit 2
fi
echo ">> conda front-end: $RUNNER    base: ${R2D3_CONDA_BASE:-<unknown>}"

# 1) isaac env (Python 3.12)
if [[ ! -x "$R2D3_ISAAC_PY" ]]; then
  echo ">> creating conda env '$ISAAC_ENV' (Python 3.12)"
  "$RUNNER" create -n "$ISAAC_ENV" python=3.12 -y
  r2d3_resolve_envs
else
  echo ">> isaac env present: $R2D3_ISAAC_PY"
fi
PY="$R2D3_ISAAC_PY"

# 2) Isaac Sim 6.0 (NVIDIA index; ~5.7 GB incl. extscache). Skip if already there.
if "$PY" -c "import importlib.util, sys; sys.exit(0 if importlib.util.find_spec('isaacsim') else 1)"; then
  echo ">> isaacsim already installed"
else
  echo ">> installing isaacsim[all,extscache]==6.0.0.0 (large, one-time download)"
  "$PY" -m pip install --upgrade pip
  "$PY" -m pip install 'isaacsim[all,extscache]==6.0.0.0' --extra-index-url https://pypi.nvidia.com
fi

# 3) platform deps + editable SDK install
echo ">> installing platform deps + editable package (r2d3-isaac)"
"$PY" -m pip install -r "$REPO/requirements.txt"
"$PY" -m pip install -e "$REPO"

# 4) robot assets (shipped in the repo)
if [[ -f "$REPO/isaac_sim/usd_dexterous/r2d3_v1.usda" ]]; then
  echo ">> robot assets present"
else
  echo ">> robot assets MISSING — regenerate with: bash scripts/build_robot.sh all" >&2
  echo "   (needs the ros_humble env for xacro; see docs/run.md)" >&2
fi

# 5) smoke test (also verifies the launcher + GPU + rendering)
echo ">> running smoke test ..."
bash "$REPO/scripts/isaacsim_ros2.sh" "$REPO/isaac_sim/tests/smoke_sdk.py"

cat <<EOF

✅ bootstrap complete. Try an example:
     scripts/isaacsim_ros2.sh isaac_sim/examples/01_hello_robot.py
     scripts/isaacsim_ros2.sh isaac_sim/examples/07_grasp_cube.py

   RL extras:   ${PY##*/} -m pip install -e .[rl]
   ROS bridge:  optional — create a 'ros_humble' RoboStack env; see docs/bridge.md
EOF
