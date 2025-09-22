#!/bin/bash
set -e

# Source ROS2 setup file
source /opt/ros/foxy/setup.bash

# 执行传递给容器的命令
exec "$@"
