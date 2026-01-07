#!/usr/bin/env bash
set -euo pipefail

WS="/workspaces/aavDroneSoftware/aav_ws"
SRC="$WS/src"
MARKER="$WS/.ros2_repos_imported"

mkdir -p "$SRC"

# Skip if already imported
if [[ -f "$MARKER" ]]; then
  echo "[postCreate] ros2.repos already imported."
  exit 0
fi

# If src already has content, don't overwrite
if [[ -n "$(ls -A "$SRC" 2>/dev/null || true)" ]]; then
  echo "[postCreate] $SRC is not empty; skipping vcs import."
  touch "$MARKER"
  exit 0
fi

echo "[postCreate] Importing ArduPilot ROS2 repos into $SRC ..."
cd "$WS"

vcs import --recursive \
  --input "https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/ros2/ros2.repos" \
  "$SRC"

touch "$MARKER"
echo "[postCreate] Done."
