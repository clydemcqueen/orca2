#!/bin/bash -x

docker build --tag orca2_vision:eloquent -f- scripts <<EOF

FROM osrf/ros:eloquent-desktop
RUN apt-get update && apt-get upgrade -y

# Get OpenCV 4.4 and GTSAM
COPY do_build_opencv_gtsam.sh /scripts/do_build_opencv_gtsam.sh
RUN /scripts/do_build_opencv_gtsam.sh

EOF

docker build --tag orca2_tools:eloquent -f- scripts <<EOF

FROM orca2_vision:eloquent

# Get python3 packages
COPY do_get_python.sh /scripts/do_get_python.sh
RUN /scripts/do_get_python.sh

# Install some tools for debugging
COPY do_get_tools.sh /scripts/do_get_tools.sh
RUN /scripts/do_get_tools.sh

EOF

docker build --tag orca2_src:eloquent -f- scripts <<EOF

FROM orca2_tools:eloquent

# Get orca2
COPY do_get_src.sh /scripts/do_get_src.sh
COPY setup.sh /scripts/setup.sh
RUN /scripts/do_get_src.sh

# Run rosdep
COPY do_rosdep.sh /scripts/do_rosdep.sh
RUN /scripts/do_rosdep.sh

EOF

docker build --tag orca2_build:eloquent -f- scripts <<EOF

FROM orca2_src:eloquent

# Build
COPY do_colcon.sh /scripts/do_colcon.sh
RUN /scripts/do_colcon.sh

EOF
