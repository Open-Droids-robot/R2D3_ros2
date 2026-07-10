#!/usr/bin/env bash
# Build the R2D3 USD for a chosen end-effector / base (or several).
#   scripts/build_robot.sh [dexterous|gripper|mobile|both|all]   (default: both)
#
# Renders isaac_sim/urdf/r2d3_v1.urdf.xacro with end_effector:=<ee> (needs the
# ros_humble env for xacro + the description packages), then converts to
# isaac_sim/usd_<out>/r2d3_v1.usda. scene.py picks the USD by R2D3_EE; the SDK's
# mobile=True loads usd_mobile. The built assets already ship in the repo — you
# only need this to regenerate them.
#
# Env names auto-detect (override R2D3_CONDA_BASE / R2D3_ROS_ENV / R2D3_ISAAC_PY;
# see scripts/_conda_env.sh).
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=scripts/_conda_env.sh
source "$REPO/scripts/_conda_env.sh"
r2d3_resolve_envs
ROS_ENV="${R2D3_ROS_ENV:-ros_humble}"

# A conda/mamba front-end that supports `run -n <env>` (xacro lives in $ROS_ENV).
RUNNER=""
for c in micromamba mamba conda; do command -v "$c" >/dev/null 2>&1 && { RUNNER="$c"; break; }; done
if [[ -z "$RUNNER" ]]; then
  echo "error: need micromamba/mamba/conda on PATH to render the xacro (it uses the '$ROS_ENV' env)." >&2
  exit 2
fi

# build_one <ee> <weld> <out> <urdf-basename>
build_one() {
  local ee="$1" weld="$2" out="$3" urdf="$4"
  echo "============================================================"
  echo "=== building: $out  (end_effector=$ee, weld_wheels=$weld)"
  echo "============================================================"
  # 1) render xacro -> flat URDF (+ post-process) in the ROS env
  "$RUNNER" run -n "$ROS_ENV" bash -c \
    "source '$REPO/install/setup.bash' && bash '$REPO/isaac_sim/urdf/render.sh' $ee $weld"
  # 2) convert URDF -> USD (urdf_to_usd.py drives the isaac python internally)
  "$REPO/scripts/urdf_to_usd.py" \
    --urdf "$REPO/isaac_sim/urdf/$urdf" \
    --usd-dir "$REPO/isaac_sim/usd_${out}" \
    --comment "R2D3 V1 ($out)"
  echo "=== done: isaac_sim/usd_${out}/r2d3_v1.usda"
}

build_dexterous()      { build_one dexterous 1 dexterous      r2d3_v1_dexterous.urdf; }
build_gripper()        { build_one gripper   1 gripper        r2d3_v1_gripper.urdf; }
build_mobile()         { build_one dexterous 0 mobile         r2d3_v1_dexterous_mobile.urdf; }
build_gripper_mobile() { build_one gripper   0 gripper_mobile r2d3_v1_gripper_mobile.urdf; }

case "${1:-both}" in
  dexterous)      build_dexterous ;;
  gripper)        build_gripper ;;
  mobile)         build_mobile ;;
  gripper-mobile) build_gripper_mobile ;;
  both)           build_dexterous; build_gripper ;;
  all)            build_dexterous; build_gripper; build_mobile; build_gripper_mobile ;;
  *) echo "usage: build_robot.sh [dexterous|gripper|mobile|gripper-mobile|both|all]" >&2; exit 1 ;;
esac
