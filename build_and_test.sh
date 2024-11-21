#!/bin/bash

script_dir=$(dirname $(readlink -f $0))

mkdir -p $script_dir/build
cd $script_dir/build

pwd

cmake -DCMAKE_BUILD_TYPE=Release -G Ninja .. \
    && ninja -v \
    && ctest --output-on-failure

cd $script_dir
