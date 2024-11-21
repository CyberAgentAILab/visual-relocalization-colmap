FROM ros:humble-ros-core

RUN apt-get update && apt-get install --no-install-recommends -y \
    clang \
    make \
    python3-pip \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

RUN apt update && apt-get install --no-install-recommends -y \
    git \
    cmake \
    ninja-build \
    build-essential \
    libboost-program-options-dev \
    libboost-filesystem-dev \
    libboost-graph-dev \
    libboost-system-dev \
    libeigen3-dev \
    libflann-dev \
    libfreeimage-dev \
    libmetis-dev \
    libgoogle-glog-dev \
    libgtest-dev \
    libgmock-dev \
    libsqlite3-dev \
    libglew-dev \
    qtbase5-dev \
    libqt5opengl5-dev \
    libcgal-dev \
    libceres-dev \
    && rm -rf /var/lib/apt/lists/*

RUN pip install -U colcon-common-extensions setuptools

RUN mkdir -p ros2_ws/src/visual_localization

COPY colmap          /ros2_ws/src/visual_localization/colmap

# Independently install colmap, not as a ros pacckage but as a binary
RUN cd ros2_ws/src/visual_localization/colmap \
  && mkdir -p build \
  && cd build \
  && cmake .. -GNinja \
  && ninja \
  && ninja install

COPY package.xml /ros2_ws/src/visual_localization/package.xml

WORKDIR /ros2_ws

RUN apt-get update \
  && rosdep init \
  && rosdep update -y \
  && rosdep install --from-paths src --ignore-src -y \
  && rm -rf /var/lib/apt/lists/*
