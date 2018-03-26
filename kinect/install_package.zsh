#!/usr/bin/env zsh
# Adding devine_kinect folder to ros package path
CURDIR="$( cd "$( dirname "${(%):-%N}" )" && pwd )/devine_kinect"
echo "export ROS_PACKAGE_PATH=$CURDIR:\$ROS_PACKAGE_PATH" >> ~/.zshrc
source ~/.zshrc