#!/bin/bash

docker buildx build --output=type=docker -f Dockerfile -t relocalization .
docker build --build-arg UID=$(id -u) --build-arg GID=$(id -g) -f Dockerfile.rviz -t relocalization-rviz .
