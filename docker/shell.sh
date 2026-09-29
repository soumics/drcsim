#!/bin/bash
# Interactive shell in the container, ROS and the workspace sourced.
NAME="${DRCSIM_CONTAINER:-drcsim}"
docker exec -it "$NAME" /entrypoint.sh bash
