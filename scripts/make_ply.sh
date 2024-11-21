#!/bin/bash

docker run \
    --rm \
    -v $(pwd)/dataset/model:/model \
    -v $(pwd)/docker_mount:/docker_mount \
    relocalization \
    colmap model_converter \
    --input_path /model \
    --output_path /docker_mount/pointcloud.ply \
    --output_type PLY
