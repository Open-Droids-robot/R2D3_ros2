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
#
# $USER_HOME is chowned RECURSIVELY, though. `groupmod -g` chowns nothing and
# `usermod -u` only rewrites uids, so a shallow chown left /home/droid/.bashrc and
# the rest of the skeleton owned by the old ids -- an unwritable shell profile for
# the very user who is about to get a shell. The directory is small (dotfiles plus
# .ros and .cache, which the loop below already recurses), so the added cost is
# negligible.
chown -R "$HOST_UID:$HOST_GID" "$USER_HOME" 2>/dev/null || true
chown "$HOST_UID:$HOST_GID" /ws /ws/src 2>/dev/null || true
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

# Readiness signal for the host: `./droid up` runs `compose exec -u droid`
# right after `compose up -d`, and resolving the `droid` account mid-remap (or
# before it) is a race when HOST_UID/HOST_GID differ from the image default of
# 1000. The remap and ownership fixes above are now complete, so drop a marker
# the host can poll for before it execs as -u droid. Written as root, before
# privileges are dropped, so no permission games are needed to create it.
mkdir -p /run/droid
touch /run/droid/ready

exec setpriv --reuid "$HOST_UID" --regid "$HOST_GID" --init-groups \
  /opt/droid/gui-start.sh "$@"
