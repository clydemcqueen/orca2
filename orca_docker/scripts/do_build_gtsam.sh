#!/bin/bash -x

# Get GTSAM prerequisites
apt-get install -y build-essential libboost-all-dev libtbb-dev

# Get GTSAM source
mkdir -p ./lib/gtsam/build \
  && mkdir -p ./lib/gtsam/install \
  && cd ./lib/gtsam \
  && git clone https://github.com/borglab/gtsam.git \
  && cd gtsam \
  && git checkout 4.1rc \
  && cd ../../../

# Build GTSAM
cd ./lib/gtsam/build \
  && cmake ../gtsam -DCMAKE_INSTALL_PREFIX=../install \
    -DCMAKE_BUILD_TYPE=Release \
    -DGTSAM_WITH_EIGEN_MKL=OFF \
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
    -DGTSAM_BUILD_TIMING_ALWAYS=OFF \
    -DGTSAM_BUILD_TESTS=OFF \
    -DGTSAM_ALLOW_DEPRECATED_SINCE_V4=OFF \
    -DGTSAM_BUILD_WRAP=OFF \
    -DGTSAM_INSTALL_CPPUNITLITE=OFF \
    -DGTSAM_WRAP_SERIALIZATION=OFF \
  && make -j$((`nproc`+2)) \
  && make install \
  && cd ../../../
