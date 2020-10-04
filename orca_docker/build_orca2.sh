#!/bin/bash -x

# Build orca2
# TODO(clyde): wait on gtsam
# TODO(clyde): add foxy branch to https://github.com/clydemcqueen/gazebo_ros_pkgs.git
# TODO(clyde): update eloquent version of these scripts (checkout eloquent branches)

docker build --tag orca2_vision:foxy -f- scripts <<EOF

FROM osrf/ros:foxy-desktop
RUN apt-get update && apt-get upgrade -y

# Get OpenCV 4.4
COPY do_build_opencv.sh /scripts/do_build_opencv.sh
RUN /scripts/do_build_opencv.sh

EOF

docker build --tag orca2_gtsam:foxy -f- scripts <<EOF

FROM orca2_vision:foxy

# Get GTSAM
COPY do_build_gtsam.sh /scripts/do_build_gtsam.sh
RUN /scripts/do_build_gtsam.sh

EOF

docker build --tag orca2_tools:foxy -f- scripts <<EOF

FROM orca2_gtsam:foxy

# Get python3 packages
COPY do_get_python.sh /scripts/do_get_python.sh
RUN /scripts/do_get_python.sh

# Install some tools for debugging
COPY do_get_tools.sh /scripts/do_get_tools.sh
RUN /scripts/do_get_tools.sh

EOF

docker build --tag orca2_src:foxy -f- scripts <<EOF

FROM orca2_tools:foxy

# Get orca2
COPY do_get_src.sh /scripts/do_get_src.sh
COPY setup.sh /scripts/setup.sh
RUN /scripts/do_get_src.sh

# Run rosdep
COPY do_rosdep.sh /scripts/do_rosdep.sh
RUN /scripts/do_rosdep.sh

EOF

docker build --tag orca2_build:foxy -f- scripts <<EOF

FROM orca2_src:foxy

# Build
COPY do_colcon.sh /scripts/do_colcon.sh
RUN /scripts/do_colcon.sh

EOF
