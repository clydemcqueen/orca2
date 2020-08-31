#!/bin/bash -x

# Get opencv source
mkdir -p ./lib/opencv/build \
  && mkdir -p ./lib/opencv/install \
  && cd ./lib/opencv \
  && git clone https://github.com/opencv/opencv.git \
  && git clone https://github.com/opencv/opencv_contrib.git \
  && cd opencv \
  && git checkout 4.4.0 \
  && cd ../opencv_contrib \
  && git checkout 4.4.0 \
  && cd ../../../

# Build opencv
cd ./lib/opencv/build \
  && cmake ../opencv -DCMAKE_INSTALL_PREFIX=../install -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib/modules -DCMAKE_BUILD_TYPE=RELEASE \
    -DBUILD_JAVA=0 -DBUILD_PERF_TESTS=0 -DBUILD_TESTS=0 -DBUILD_DOCS=OFF \
    -DINSTALL_C_EXAMPLES=ON  -DINSTALL_PYTHON_EXAMPLES=ON -DBUILD_EXAMPLES=ON \
    -DOPENCV_GENERATE_PKGCONFIG=ON -DWITH_GTK_2_X=ON \
  && make -j$((`nproc`+2)) \
  && make install \
  && cd ../../../

# Get GTSAM prerequisites
apt-get install -y libboost-all-dev libtbb-dev

# Get GTSAM source
mkdir -p ./lib/gtsam/build \
  && mkdir -p ./lib/gtsam/install \
  && cd ./lib/gtsam \
  && git clone https://github.com/borglab/gtsam.git \
  && cd gtsam \
  && git checkout 4.0.3 \
  && cd ../../../

# Build GTSAM
cd ./lib/gtsam/build \
  && cmake ../gtsam -DCMAKE_INSTALL_PREFIX=../install \
    -DGTSAM_BUILD_TESTS=0 -DGTSAM_BUILD_EXAMPLES_ALWAYS=0 -DGTSAM_ALLOW_DEPRICATED_SINCE_V4=0 -DGTSAM_BUILD_WRAP=0 \
    -DGTSAM_INSTALL_CPPUNITLITE=0 -DTSAM_WRAP_SERIALIZATION=0 \
  && make -j$((`nproc`+2)) \
  && make install \
  && cd ../../../
