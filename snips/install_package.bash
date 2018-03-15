#!/usr/bin/env bash
# Adding snips folder to ros package path
CURDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )/snips"
echo "export ROS_PACKAGE_PATH=$CURDIR:\$ROS_PACKAGE_PATH" >> ~/.bashrc
source ~/.bashrc