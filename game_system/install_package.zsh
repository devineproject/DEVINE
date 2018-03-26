#!/usr/bin/env zsh
# Adding game_system folder to ros package path
CURDIR="$( cd "$( dirname "${(%):-%N}" )" && pwd )/game_system"
echo "export ROS_PACKAGE_PATH=$CURDIR:\$ROS_PACKAGE_PATH" >> ~/.zshrc
source ~/.zshrc
