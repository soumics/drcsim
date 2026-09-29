#!/bin/bash
# Open Gazebo's GUI and RViz from the container on this display.
NAME="${DRCSIM_CONTAINER:-drcsim}"
docker exec "$NAME" /entrypoint.sh drcsim_gui
