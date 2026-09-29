# drcsim on ROS 2 Jazzy + Gazebo Harmonic, NVIDIA GPU, CycloneDDS.
#
# Build from the repository root (the build context is the whole repo):
#   docker/build.sh            (wraps: docker build -f docker/Dockerfile -t drcsim:jazzy .)
# Run with docker/run.sh, which passes the GPU, X11 and host networking.
ARG BASE_IMAGE=osrf/ros:jazzy-desktop-full
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# CycloneDDS instead of the default FastDDS: FastDDS crashed the gz server
# (a writer crash while AtlasPlugin created its node) whenever GPU-rendered
# sensors were on, and dropped the first service replies under the hands'
# bigger startup graph (see CLAUDE.md). Xvfb/xauth let tools render a
# virtual display for screenshots and CI; mesa-utils is for
# checking the GL/NVIDIA setup (glxinfo -B); ffmpeg records demo videos.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-rmw-cyclonedds-cpp \
      ros-jazzy-teleop-twist-keyboard \
      python3-colcon-common-extensions \
      python3-pil \
      git ffmpeg \
      xvfb xauth mesa-utils x11-utils \
    && rm -rf /var/lib/apt/lists/*

# The workspace, with every repo dependency resolved by rosdep.
WORKDIR /root/ros2_ws
COPY . src/drcsim
# Third-party packages under their own licences (SCHUNK SVH hand: GPL-3.0),
# fetched next to -- never into -- this repository.
RUN src/drcsim/docker/fetch_external.sh /root/ros2_ws/src
RUN apt-get update \
    && (rosdep update --rosdistro jazzy || true) \
    && rosdep install --from-paths src/drcsim \
       src/external/schunk_svh_ros_driver/schunk_svh_description \
       --ignore-src --rosdistro jazzy -y -r \
    && rm -rf /var/lib/apt/lists/*
RUN source /opt/ros/jazzy/setup.bash \
    && colcon build --event-handlers console_direct- --cmake-args -DBUILD_TESTING=OFF

# NVIDIA: all driver capabilities -- graphics/display for Gazebo's Ogre2 and
# RViz, compute/utility for nvidia-smi. The nvidia-container-toolkit reads
# these when the container is started with --gpus all.
ENV NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=all \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    CYCLONEDDS_URI=file:///root/cyclonedds.xml \
    ROS_DOMAIN_ID=77 \
    GZ_PARTITION=drcsim \
    QT_X11_NO_MITSHM=1

COPY docker/cyclonedds.xml /root/cyclonedds.xml
COPY docker/entrypoint.sh /entrypoint.sh
COPY docker/in_container/ /usr/local/bin/
RUN chmod +x /entrypoint.sh /usr/local/bin/drcsim_*

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
