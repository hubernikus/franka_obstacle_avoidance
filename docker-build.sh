#!/bin/bash

# if [[ "$OSTYPE" != "darwin"* ]]; then
# 	BUILD_FLAGS+=(--ssh default="${SSH_AUTH_SOCK}")
# else
# BUILD_FLAGS+=(--ssh default="$HOME/.ssh/id_ed25519")
# fi

# docker build -t ros2_franka_avoidance 
# DOCKER_BUILDKIT=1 docker build "${BUILD_FLAGS[@]}" . || exit 1
# DOCKER_BUILDKIT=1 docker build "${BUILD_FLAGS[@]}" . || exit 1
# install ros package
DOCKER_BUILDKIT=1 docker build -t ros2_franka_avoidance --ssh default="$HOME/.ssh/id_ed25519" .

# TO run:
# 
# $ docker run -it --rm ros_with_rviz
# 
