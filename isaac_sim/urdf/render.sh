#!/usr/bin/env bash
# isaac_sim/urdf/render.sh
#
# Render r2d3_v1.urdf.xacro into a flat URDF that scripts/urdf_to_usd.py
# can consume. Requires ROS2 + xacro (so this runs inside a ROS2 Humble
# container — see Docker/docker/Dockerfile.humble — not on the host).
#
# Inside the Humble container:
#     source /opt/ros/humble/setup.bash
#     cd /workspace/r2d3_isaac
#     colcon build --packages-select dual_rm_75b_description realsense2_description
#     source install/setup.bash
#     bash isaac_sim/urdf/render.sh
#
# Output: isaac_sim/urdf/r2d3_v1.urdf  (committed for review; regenerable any time)
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IN="$HERE/r2d3_v1.urdf.xacro"
OUT="$HERE/r2d3_v1.urdf"

if ! command -v xacro >/dev/null; then
  echo "error: xacro not found on PATH. Run this inside a ROS2 environment." >&2
  echo "       (source /opt/ros/humble/setup.bash, etc.)" >&2
  exit 1
fi

echo "rendering: $IN"
echo "        -> $OUT"
xacro "$IN" -o "$OUT"
echo
echo "Done. Verify the rendered URDF parses:"
echo "  python3 -c 'from lxml import etree; etree.parse(\"$OUT\")'"
echo "Verify gripper joints appear:"
echo "  grep -E 'finger_(drive|mimic)' '$OUT' | head"
