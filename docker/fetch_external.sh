#!/bin/bash
# Fetch third-party packages this repo uses but does not contain (they keep
# their own licences). Usage: docker/fetch_external.sh [WORKSPACE_SRC_DIR]
#   default WORKSPACE_SRC_DIR: the directory containing this repository.
#
# - SCHUNK SVH five-finger hand description (GPL-3.0-or-later):
#   https://github.com/SCHUNK-SE-Co-KG/schunk_svh_ros_driver (branch ros2).
#   Only schunk_svh_description is built; the driver packages are ignored.
set -e
SRC="${1:-$(cd "$(dirname "$0")/../.." && pwd)}"
DEST="$SRC/external/schunk_svh_ros_driver"
if [ ! -d "$DEST" ]; then
  git clone --depth 1 -b ros2 https://github.com/SCHUNK-SE-Co-KG/schunk_svh_ros_driver.git "$DEST"
fi
for pkg in schunk_svh_driver schunk_svh_simulation schunk_svh_tests; do
  [ -d "$DEST/$pkg" ] && touch "$DEST/$pkg/COLCON_IGNORE"
done
echo "External packages ready in $SRC/external (SVH: $(git -C "$DEST" log --oneline -1))."
