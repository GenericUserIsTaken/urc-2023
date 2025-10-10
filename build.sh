#!/usr/bin/env bash

# Run ZED dependency setup.
./setup_zed_dependencies.sh

# Make sure ROS packages are accessable.
source /opt/ros/$ROS_DISTRO/setup.bash

# Make sure we have rosdep dependencies (please update apt and rosdep beforehand).
rosdep install --from-paths src --ignore-src -r -y

# Build the ZED components.
colcon build \
    --symlink-install \
    --base-paths /home/trickfire/urc-2023 \
    --packages-select zed_components \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo

colcon build \
    --symlink-install \
    --base-paths /home/trickfire/urc-2023 \
    --packages-select zed_wrapper \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo

# Build everything else.
colcon build \
    --symlink-install \
    --base-paths /home/trickfire/urc-2023\
    --cmake-args \
        -DCMAKE_BUILD_TYPE=RelWithDebInfo
