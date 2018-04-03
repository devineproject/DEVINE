#!/usr/bin/env bash
# Adding devine_segmentation folder to ros package path
CURDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )/devine_segmentation"
echo "export ROS_PACKAGE_PATH=$CURDIR:\$ROS_PACKAGE_PATH" >> ~/.bashrc

#Env setup
sudo pip install --upgrade setuptools
sudo pip3 install Cython scikit-image bson pymongo pycocotools keras catkin_pkg rospkg
wget https://github.com/matterport/Mask_RCNN/releases/download/v2.0/mask_rcnn_coco.h5

#Reload env
exec bash
