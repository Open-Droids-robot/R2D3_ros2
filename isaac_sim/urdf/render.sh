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

# Usage: render.sh [end_effector] [weld_wheels]
#   end_effector = dexterous | gripper   (default dexterous)
#   weld_wheels  = 1 (default, AGV welded for stability) | 0 (mobile: wheels stay
#                  revolute so they can roll; output tagged _mobile)
EE="${1:-dexterous}"
WELD="${2:-1}"
TAG=""; [ "$WELD" = "0" ] && TAG="_mobile"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IN="$HERE/r2d3_v1.urdf.xacro"
OUT="$HERE/r2d3_v1_${EE}${TAG}.urdf"

if ! command -v xacro >/dev/null; then
  echo "error: xacro not found on PATH. Run this inside a ROS2 environment." >&2
  echo "       (source /opt/ros/humble/setup.bash, etc.)" >&2
  exit 1
fi

echo "rendering: $IN  (end_effector=$EE)"
echo "        -> $OUT"
xacro "$IN" "end_effector:=$EE" -o "$OUT"

# Upstream 75b URDF contains 34 <material name=""> tags (empty name). The
# urdf_usd_converter (Isaac Sim 6.0) refuses to handle anonymous materials —
# its NameCache.find_unique_names() gets None and raises. Post-process the
# rendered URDF to give each anonymous material a unique placeholder name.
# This is purely cosmetic from URDF's perspective (anonymous materials are
# rarely referenced by name) but lets the converter author the asset.
#
# Additionally: realsense2_description authors `*_bottom_screw_frame` as an
# empty link (no inertial). PhysX in Isaac Sim 6.0 treats every URDF link
# as a rigid body and assigns mass=-1, inertia={1,1,1} when none are
# specified — that surfaces as `Illegal BroadPhaseUpdateData - non-finite
# bounds` errors at every sim step. Inject a tiny inertial into any link
# that has none.
python3 - "$OUT" "$WELD" <<'PYEOF'
import re, sys
p = sys.argv[1]
weld_wheels = (len(sys.argv) < 3 or sys.argv[2] != "0")
with open(p) as f:
    text = f.read()

counter = {"n": 0}
def repl(m):
    counter["n"] += 1
    return f'<material name="unnamed_{counter["n"]:03d}">'
text = re.sub(r'<material\s+name=""\s*>', repl, text)
print(f"[render] renamed {counter['n']} unnamed materials")

# Find empty links: <link name="X"/>  → expand with tiny inertial. The mass
# is small enough to not affect kinematics; the inertia is finite so PhysX
# can compute bounds.
DUMMY_INERTIAL = (
    '<inertial>'
    '<origin xyz="0 0 0" rpy="0 0 0"/>'
    '<mass value="0.001"/>'
    '<inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>'
    '</inertial>'
)
ic = {"n": 0}
def fill(m):
    ic["n"] += 1
    return f'<link name="{m.group(1)}">{DUMMY_INERTIAL}</link>'
text = re.sub(r'<link\s+name="([^"]+)"\s*/>', fill, text)
print(f"[render] injected dummy inertial into {ic['n']} empty links")

# Weld the 10 AGV wheel joints (continuous) to FIXED. The AGV is stationary
# in V1 (architecture.md: lock the wheels); as free continuous joints with
# zero damping they are the dominant source of articulation NaN (they spin up
# in a single step and destabilize the whole tree). Welding removes 10 DOFs
# and the instability. Reversible for V2: revert to continuous + add a wheel
# controller.
if weld_wheels:
    wc = {"n": 0}
    def weld(m):
        name = m.group(1)
        if "wheel" in name:
            wc["n"] += 1
            return m.group(0).replace('type="continuous"', 'type="fixed"')
        return m.group(0)
    text = re.sub(r'<joint name="([^"]+)"[^>]*type="continuous"[^>]*>', weld, text)
    print(f"[render] welded {wc['n']} AGV wheel joints to fixed")
else:
    print("[render] kept AGV wheels REVOLUTE (mobile build)")

# Inject <dynamics damping="..." friction="..."/> into every joint that
# doesn't already have one. The upstream URDF only damps the finger joints
# we author; the arms, lift, head, and wheels have no damping, so PhysX
# integrates them with positive feedback until joint positions go NaN.
DYNAMICS_TAG = '<dynamics damping="1.0" friction="0.1"/>'
jc = {"n": 0}
def add_dyn(m):
    body = m.group(2)
    if '<dynamics' in body:
        return m.group(0)   # already has one
    jc["n"] += 1
    return f'<joint{m.group(1)}>{body}{DYNAMICS_TAG}</joint>'
text = re.sub(
    r'<joint([^>]*type="(?:revolute|prismatic|continuous)"[^>]*)>(.*?)</joint>',
    add_dyn, text, flags=re.DOTALL,
)
print(f"[render] injected dynamics into {jc['n']} joints")

# Fix the head tilt axis. Upstream authors head_joint2 with axis="-1 0 0"
# (the -X / camera-forward axis), so commanding tilt ROLLS the head camera
# about its view axis instead of pitching it. Re-author it as the lateral
# -Y axis so tilt NODS the head down (negative tilt = look-down, matching
# tasks/*.yaml head.tilt_rad). Verified via isaac_sim/tests/diag_head.py.
hj2 = re.subn(
    r'(<joint name="head_joint2"[^>]*>.*?<axis xyz=")[^"]*(")',
    r'\g<1>0 -1 0\g<2>', text, count=1, flags=re.DOTALL)
text, n_hj2 = hj2
print(f"[render] re-axised head_joint2 tilt to lateral (-Y): {n_hj2} joint")

with open(p, "w") as f:
    f.write(text)
PYEOF

echo
echo "Done. Verify the rendered URDF parses:"
echo "  python3 -c 'from lxml import etree; etree.parse(\"$OUT\")'"
echo "Verify gripper joints appear:"
echo "  grep -E 'finger_(drive|mimic)' '$OUT' | head"
