#!/bin/bash
# Start (or reuse) the drcsim container in the background with the NVIDIA
# GPU, the host's X display and host networking.
#   docker/run.sh              start container "drcsim"
#   DRCSIM_PRIME_OFFLOAD=0 docker/run.sh   desktop GPU driving the display
# ROS domain 78 and gz partition drcsim_docker by default (override with
# DRCSIM_ROS_DOMAIN_ID / DRCSIM_GZ_PARTITION): host networking means any
# other simulation on the same domain/partition would mix into this one.
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
  -e ROS_DOMAIN_ID="${DRCSIM_ROS_DOMAIN_ID:-78}" \
  -e GZ_PARTITION="${DRCSIM_GZ_PARTITION:-drcsim_docker}" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  ${XAUTHORITY:+-e XAUTHORITY=/tmp/.Xauthority -v "$XAUTHORITY":/tmp/.Xauthority:ro} \
  --net host --ipc host \
  "$IMAGE" sleep infinity > /dev/null
echo "Started container $NAME ($IMAGE)."
