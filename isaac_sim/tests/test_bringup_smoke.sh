#!/usr/bin/env bash
# End-to-end smoke test for the R2D3 bring-up.
#
# What it checks:
#   1. Isaac boots and publishes /r2d3/sim/ready
#   2. /r2d3/sim/joint_states streams at the configured rate
#   3. r2d3_humble_bridge translates rm_ros_interfaces -> Isaac and back
#   4. Movej moves the left arm
#   5. Liftheight moves the body lift
#   6. Gripperset closes both fingers symmetrically
#   7. /r2d3/observations publishes the aggregated Observation at 10 Hz
#
# Prereqs:
#   - pip install isaacsim[all,extscache]==6.0.0.0 (see project_isaacsim_extscache)
#   - bash scripts/build_packages.sh
set -uo pipefail

REPO=/usr1/home/semathew/r2d3_isaac
MAMBA=/usr1/home/semathew/miniforge3/bin/mamba
LOG_DIR=/tmp/r2d3_smoke_$(date +%s)
mkdir -p "$LOG_DIR"

echo "[smoke] log dir: $LOG_DIR"

ros2_run() {
    "$MAMBA" run -n ros_humble bash -c "
        source $REPO/install/setup.bash 2>/dev/null
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
        $1
    "
}

# Parse one JointState message from `ros2 topic echo --once` (multi-doc YAML).
parse_js() {
    python3 - "$@" <<'PYEOF'
import sys, yaml, math
docs = list(yaml.safe_load_all(sys.stdin.read()))
m = docs[0]
n = sys.argv[1]
i = m["name"].index(n)
v = m["position"][i]
print(f"{v:.5f}" if not math.isnan(v) else "NaN")
PYEOF
}

# --- 1. Start Isaac bring_up -------------------------------------------------
echo "[smoke] launching bring_up..."
nohup bash "$REPO/scripts/isaacsim_ros2.sh" \
    "$REPO/isaac_sim/r2d3_sim/bring_up.py" --headless --max-steps 100000 \
    > "$LOG_DIR/bring_up.log" 2>&1 &
BRINGUP_PID=$!
trap 'kill -9 $BRINGUP_PID $BRIDGE_PID 2>/dev/null; wait 2>/dev/null; exit' EXIT INT TERM

echo "[smoke] waiting for /r2d3/sim/ready..."
for i in $(seq 1 30); do
    if ros2_run 'timeout 3 ros2 topic echo --once /r2d3/sim/ready 2>&1' \
            | grep -q '^data: true'; then
        echo "[smoke]   ready after ${i}*2s"
        break
    fi
    sleep 2
done

# --- 2. Verify joint_states rate ---------------------------------------------
echo "[smoke] joint_states rate..."
hz=$(ros2_run 'timeout 6 ros2 topic hz /r2d3/sim/joint_states 2>&1' \
       | grep 'average rate' | head -1 | awk '{print $3}')
echo "[smoke]   joint_states rate: ${hz} Hz"

# --- 3. Start the bridge -----------------------------------------------------
echo "[smoke] launching r2d3_humble_bridge..."
nohup "$MAMBA" run -n ros_humble bash -c "
    source $REPO/install/setup.bash
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    ros2 run r2d3_humble_bridge r2d3_humble_bridge
" > "$LOG_DIR/bridge.log" 2>&1 &
BRIDGE_PID=$!
sleep 5

# --- 4. Movej moves left arm -------------------------------------------------
echo "[smoke] Movej l_joint1=0.4..."
ros2_run 'timeout 3 ros2 topic pub --once \
  /left_arm_controller/rm_driver/movej_cmd rm_ros_interfaces/msg/Movej \
  "{joint: [0.4, -0.5, 0.0, -1.0, 0.0, 0.6, 0.0], speed: 50, block: false, dof: 7}"' \
  2>&1 | grep publishing | head -1
sleep 2
val=$(ros2_run 'timeout 5 ros2 topic echo --once /r2d3/sim/joint_states' | parse_js l_joint1)
echo "[smoke]   l_joint1 = $val (target 0.4)"

# --- 5. Liftheight moves lift ------------------------------------------------
echo "[smoke] Liftheight 700 mm..."
ros2_run 'timeout 3 ros2 topic pub --once \
  /left_arm_controller/rm_driver/set_lift_height_cmd rm_ros_interfaces/msg/Liftheight \
  "{height: 700, speed: 30, block: false}"' 2>&1 | grep publishing | head -1
sleep 2
val=$(ros2_run 'timeout 5 ros2 topic echo --once /r2d3/sim/joint_states' | parse_js platform_joint)
echo "[smoke]   platform_joint = $val (target 0.700)"

# --- 6. Gripperset closes fingers symmetrically ------------------------------
echo "[smoke] Gripperset position 200..."
ros2_run 'timeout 3 ros2 topic pub --once \
  /left_arm_controller/rm_driver/set_gripper_position_cmd rm_ros_interfaces/msg/Gripperset \
  "{position: 200, block: false, timeout: 100}"' 2>&1 | grep publishing | head -1
sleep 2
drv=$(ros2_run 'timeout 5 ros2 topic echo --once /r2d3/sim/joint_states' | parse_js l_finger_drive)
mim=$(ros2_run 'timeout 5 ros2 topic echo --once /r2d3/sim/joint_states' | parse_js l_finger_mimic)
echo "[smoke]   l_finger_drive=$drv  l_finger_mimic=$mim (target 0.007 m each)"

# --- 7. /r2d3/observations rate ----------------------------------------------
echo "[smoke] /r2d3/observations rate..."
obs_hz=$(ros2_run 'timeout 6 ros2 topic hz /r2d3/observations 2>&1' \
       | grep 'average rate' | head -1 | awk '{print $3}')
echo "[smoke]   observations rate: ${obs_hz} Hz"

# --- 8. OmniGraph fast-path topics -------------------------------------------
echo "[smoke] /clock rate..."
clk_hz=$(ros2_run 'timeout 5 ros2 topic hz /clock 2>&1' \
       | grep 'average rate' | head -1 | awk '{print $3}')
echo "[smoke]   /clock rate: ${clk_hz} Hz"

echo "[smoke] /tf rate..."
tf_hz=$(ros2_run 'timeout 5 ros2 topic hz /tf 2>&1' \
       | grep 'average rate' | head -1 | awk '{print $3}')
echo "[smoke]   /tf rate: ${tf_hz} Hz"

echo "[smoke] /camera/color/image_raw rate..."
rgb_hz=$(ros2_run 'timeout 6 ros2 topic hz /camera/color/image_raw 2>&1' \
       | grep 'average rate' | head -1 | awk '{print $3}')
echo "[smoke]   /camera/color/image_raw rate: ${rgb_hz} Hz"

echo "[smoke] /camera/depth/image_raw rate..."
d_hz=$(ros2_run 'timeout 6 ros2 topic hz /camera/depth/image_raw 2>&1' \
       | grep 'average rate' | head -1 | awk '{print $3}')
echo "[smoke]   /camera/depth/image_raw rate: ${d_hz} Hz"

echo "[smoke] DONE. Logs in $LOG_DIR/"
