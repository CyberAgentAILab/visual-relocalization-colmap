#!/bin/bash

if [ ! -f vocab_tree_flickr100K_words256K.bin ]; then
    wget -c https://demuc.de/colmap/vocab_tree_flickr100K_words256K.bin
fi

ply_path=$(pwd)/docker_mount/pointcloud.ply

if [ ! -f $ply_path ]; then
    echo "Run ./scripts/make_ply.sh"
    exit
fi

docker run --rm  \
    -v $(pwd)/apps:/ros2_ws/src/visual_localization/apps \
    -v $(pwd)/vocab_tree_flickr100K_words256K.bin:/vocab_tree_flickr100K_words256K.bin \
    -v $(pwd)/dataset/model:/reconstruction \
    -v $(pwd)/dataset/test:/input_images \
    -v $(pwd)/dataset/intrinsic.xml:/intrinsic.xml \
    -v $ply_path:/pointcloud.ply \
    -v $(pwd)/docker_mount/build_cache:/ros2_ws/build \
    -v $(pwd)/docker_mount/install:/ros2_ws/install \
    -v $(pwd)/CMakeLists.txt:/ros2_ws/src/visual_localization/CMakeLists.txt \
    -v $(pwd)/config:/ros2_ws/src/visual_localization/config \
    -v $(pwd)/include:/ros2_ws/src/visual_localization/include \
    -v $(pwd)/launch:/ros2_ws/src/visual_localization/launch \
    -v $(pwd)/src:/ros2_ws/src/visual_localization/src \
    -v $(pwd)/tests:/ros2_ws/src/visual_localization/tests \
    --net=bridge \
    relocalization \
    /bin/bash -c "source /opt/ros/humble/setup.bash \
     && cd /ros2_ws/ \
     && source install/setup.bash \
     && ros2 launch visual_localization relocalization_launch.py"
