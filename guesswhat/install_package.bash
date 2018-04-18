#!/usr/bin/env bash
# Adding devine_guesswhat folder to ros package path
CURDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )/devine_guesswhat"
echo "export ROS_PACKAGE_PATH=$CURDIR:\$ROS_PACKAGE_PATH" >> ~/.bashrc

wget https://github.com/projetdevine/static/releases/download/v0.0.1/weights.zip
unzip weights.zip -d devine_guesswhat/data

# Adding guesswhat repo to pythonpath
CURDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )/guesswhat/src"
echo "export PYTHONPATH=$CURDIR:\$PYTHONPATH" >> ~/.bashrc

# Reload env
exec bash
