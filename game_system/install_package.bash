#!/usr/bin/env bash
# Adding game_system folder to ros package path
CURDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )/game_system"
echo "export ROS_PACKAGE_PATH=$CURDIR:\$ROS_PACKAGE_PATH" >> ~/.bashrc
source ~/.bashrc
