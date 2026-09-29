#!/bin/bash
# Start (or reuse) the drcsim container in the background with the NVIDIA
# GPU, the host's X display and host networking.
#   docker/run.sh              start container "drcsim"
#   DRCSIM_PRIME_OFFLOAD=0 docker/run.sh   desktop GPU driving the display
# Then: docker/sim.sh, docker/gui.sh, docker/teleop.sh, docker/shell.sh.
set -e
NAME="${DRCSIM_CONTAINER:-drcsim}"
IMAGE="${DRCSIM_IMAGE:-drcsim:jazzy}"
if docker ps --format '{{.Names}}' | grep -qx "$NAME"; then
  echo "Container $NAME already running."; exit 0
fi
docker rm "$NAME" > /dev/null 2>&1 || true
# Let local containers use the X server (only local connections).
xhost +local: > /dev/null 2>&1 || echo "warning: xhost not available; GUI may not open"
docker run -d --name "$NAME" \
  --gpus all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e DRCSIM_PRIME_OFFLOAD="${DRCSIM_PRIME_OFFLOAD:-1}" \
  -e DISPLAY="${DISPLAY:-:0}" \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  ${XAUTHORITY:+-e XAUTHORITY=/tmp/.Xauthority -v "$XAUTHORITY":/tmp/.Xauthority:ro} \
  --net host --ipc host \
  "$IMAGE" sleep infinity > /dev/null
echo "Started container $NAME ($IMAGE)."
