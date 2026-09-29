#!/bin/bash
# Stop and remove the drcsim container (only this one).
NAME="${DRCSIM_CONTAINER:-drcsim}"
docker rm -f "$NAME" > /dev/null && echo "Removed container $NAME."
