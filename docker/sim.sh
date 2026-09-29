#!/bin/bash
# (Re)start the simulation in the container, in the background. Any running
# simulation there is stopped first. Args go to atlas.launch.py.
#   docker/sim.sh                     Atlas, Sandia hands, modern skin (server only;
#                                     open the GUI with docker/gui.sh)
#   docker/sim.sh hands:=robotiq skin:=classic
NAME="${DRCSIM_CONTAINER:-drcsim}"
if docker exec "$NAME" pgrep -f "ros2 launch drcsim_gazebo" > /dev/null; then
  echo "Stopping the running simulation..."
  docker exec "$NAME" pkill -INT -f "ros2 launch drcsim_gazebo"
  sleep 5
  docker exec "$NAME" pkill -9 -f "gz sim -s" 2> /dev/null
  docker exec "$NAME" pkill -9 -f robot_state_publisher 2> /dev/null
  docker exec "$NAME" pkill -9 -f parameter_bridge 2> /dev/null
fi
docker exec -d "$NAME" /entrypoint.sh bash -c "drcsim_sim headless:=true $* > /tmp/sim.log 2>&1"
echo "Simulation starting in $NAME (log: docker exec $NAME tail -f /tmp/sim.log)."
