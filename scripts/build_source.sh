#!/bin/bash

docker run --rm \
    -v $(pwd)/docker_mount/build_cache:/ros2_ws/build \
    -v $(pwd)/docker_mount/install:/ros2_ws/install \
    -v $(pwd)/apps:/ros2_ws/src/visual_localization/apps \
    -v $(pwd)/CMakeLists.txt:/ros2_ws/src/visual_localization/CMakeLists.txt \
    -v $(pwd)/config:/ros2_ws/src/visual_localization/config \
    -v $(pwd)/include:/ros2_ws/src/visual_localization/include \
    -v $(pwd)/launch:/ros2_ws/src/visual_localization/launch \
    -v $(pwd)/src:/ros2_ws/src/visual_localization/src \
    -v $(pwd)/tests:/ros2_ws/src/visual_localization/tests \
    relocalization  \
    /bin/bash -c "source /opt/ros/humble/setup.bash \
     && cd /ros2_ws/ \
     && colcon build \
       --parallel-workers 32 \
       --packages-select visual_localization \
       --cmake-args \
       -DCMAKE_BUILD_TYPE=Release"
