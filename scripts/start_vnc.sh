#!/usr/bin/env bash
# scripts/start_vnc.sh — start a TurboVNC session for Isaac Sim GUI work.
#
# Usage:
#   scripts/start_vnc.sh           # start :1 @ 1920x1080
#   scripts/start_vnc.sh :2 2560x1440
#   scripts/start_vnc.sh stop      # kill the :1 session
#   scripts/start_vnc.sh list      # show running sessions
#
# First run will prompt you to set a VNC password (stored at ~/.vnc/passwd).
# After starting, tunnel from your laptop:
#   ssh -L 5901:localhost:5901 semathew@riddle.autonomy.ri.cmu.edu
# then connect TurboVNC Viewer to localhost:5901.
set -euo pipefail

VNC=/opt/TurboVNC/bin/vncserver

case "${1:-}" in
  stop)
    "$VNC" -kill ":${2:-1}" 2>&1 | tail -3
    exit 0
    ;;
  list)
    "$VNC" -list 2>&1
    exit 0
    ;;
esac

DISPLAY_NUM="${1:-:1}"
GEOMETRY="${2:-1920x1080}"

[[ -x "$VNC" ]] || { echo "TurboVNC not found at $VNC" >&2; exit 1; }
mkdir -p "$HOME/.vnc"

echo "Starting TurboVNC on $DISPLAY_NUM @ $GEOMETRY..."
"$VNC" \
  "$DISPLAY_NUM" \
  -geometry "$GEOMETRY" \
  -depth 24 \
  -localhost \
  -nohttpd \
  -SecurityTypes VncAuth

cat <<EOF

VNC session started on $DISPLAY_NUM.

Tunnel from your laptop:
  ssh -L 590${DISPLAY_NUM#:}:localhost:590${DISPLAY_NUM#:} ${USER}@$(hostname -f)

Then connect TurboVNC Viewer to localhost:590${DISPLAY_NUM#:}.

For GPU-accelerated GL inside the session:
  DISPLAY=$DISPLAY_NUM vglrun glxinfo | head -5
  DISPLAY=$DISPLAY_NUM vglrun <gl-app>

To stop:  $0 stop ${DISPLAY_NUM#:}
EOF
