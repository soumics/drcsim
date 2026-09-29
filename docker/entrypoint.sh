#!/bin/bash
# Source ROS 2 and the drcsim workspace, then run the given command.
set -e
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
# Hybrid-graphics laptops (Intel display + NVIDIA offload): route GL to the
# NVIDIA GPU when asked (docker/run.sh sets DRCSIM_PRIME_OFFLOAD=1 by default).
if [ "${DRCSIM_PRIME_OFFLOAD:-0}" = "1" ]; then
  export __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia
fi
exec "$@"
