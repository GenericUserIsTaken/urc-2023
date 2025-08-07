#!/usr/bin/env bash


# clear ros stuff
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset ROS_PACKAGE_PATH
unset PYTHONPATH

# source only from this workspace
source /opt/ros/humble/setup.bash
source /home/trickfire/jake-urc-2023/urc-2023/install/setup.bash

#modprobe can
#modprobe can_raw
#modprobe mttcan
#ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
#ip link set can0 up

# Add to the python import pathes. Not the best, but will work for now
export PYTHONPATH="/home/trickfire/jake-urc-2023/urc-2023/src/:$PYTHONPATH"
ros2 pkg prefix autonomous_nav

ros2 launch viator_launch robot.launch.py   