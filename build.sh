#!/usr/bin/env bash

source /opt/ros/$ROS_DISTRO/setup.bash

colcon build \
    --symlink-install \
    --base-paths /home/trickfire/jake-urc-2023/urc-2023\
    --cmake-args \
        -DCMAKE_BUILD_TYPE=Release
