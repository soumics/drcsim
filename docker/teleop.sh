#!/bin/bash
# Keyboard teleop (interactive). Ctrl-C quits.
NAME="${DRCSIM_CONTAINER:-drcsim}"
docker exec -it "$NAME" /entrypoint.sh drcsim_teleop "$@"
