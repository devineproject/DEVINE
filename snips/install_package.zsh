#!/usr/bin/env zsh
# Adding snips folder to ros package path
CURDIR="$( cd "$( dirname "${(%):-%N}" )" && pwd )/snips"
echo "export ROS_PACKAGE_PATH=$CURDIR:\$ROS_PACKAGE_PATH" >> ~/.zshrc
source ~/.zshrc