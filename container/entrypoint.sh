#!/usr/bin/env bash
# Runs as root. Remaps the runtime user's numeric id to the host user's so files
# created in the bind mount are owned by the developer on the host, fixes up
# ownership, starts the GUI stack, then drops privileges.
#
# Running the container as an unmapped host uid was rejected: it provides no
# passwd entry, and therefore no sudo -- and developers need to install packages
# mid-session.
set -eu

USER_NAME="droid"
USER_HOME="/home/$USER_NAME"
HOST_UID="${HOST_UID:-1000}"
HOST_GID="${HOST_GID:-1000}"

current_uid="$(id -u "$USER_NAME")"
current_gid="$(id -g "$USER_NAME")"

if [ "$current_gid" != "$HOST_GID" ]; then
  groupmod -o -g "$HOST_GID" "$USER_NAME"
fi
if [ "$current_uid" != "$HOST_UID" ]; then
  usermod -o -u "$HOST_UID" "$USER_NAME"
fi

# Volumes are created root-owned by the daemon; the home directory needs fixing
# whenever the ids moved. The bind-mounted source tree is deliberately NOT
# chowned -- it already belongs to the host user, and recursing into it would
# rewrite the ctime of every file in the developer's checkout on every start.
# That is why /ws itself is chowned shallowly and only the build artefact
# directories below it are chowned recursively.
chown "$HOST_UID:$HOST_GID" "$USER_HOME" /ws /ws/src 2>/dev/null || true
for d in "$USER_HOME/.ros" "$USER_HOME/.cache" /ws/build /ws/install /ws/log; do
  [ -d "$d" ] || mkdir -p "$d"
  chown -R "$HOST_UID:$HOST_GID" "$d" 2>/dev/null || true
done

# setpriv does NOT build a login environment: it leaves HOME at root's and USER
# empty (verified on this base image). Left alone, every ROS tool would resolve
# ~ to /root, so the MuJoCo cache pre-warmed into /home/droid/.ros would miss and
# the converter venv would be rebuilt from scratch on first launch -- the exact
# cost the pre-warm exists to remove. Set the login variables explicitly.
export HOME="$USER_HOME"
export USER="$USER_NAME"
export LOGNAME="$USER_NAME"

exec setpriv --reuid "$HOST_UID" --regid "$HOST_GID" --init-groups \
  /opt/droid/gui-start.sh "$@"
