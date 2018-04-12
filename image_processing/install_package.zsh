#!/usr/bin/env zsh
# Adding devine_segmentation folder to ros package path
CURDIR="$( cd "$( dirname "${(%):-%N}" )" && pwd )/devine_image_processing"
echo "export ROS_PACKAGE_PATH=$CURDIR:\$ROS_PACKAGE_PATH" >> ~/.zshrc

#Env setup
sudo pip3 install Cython scikit-image bson pymongo pycocotools keras catkin_pkg rospkg
wget https://github.com/matterport/Mask_RCNN/releases/download/v2.1/mask_rcnn_coco.h5
wget http://download.tensorflow.org/models/vgg_16_2016_08_28.tar.gz
tar xzvf vgg_16_2016_08_28.tar.gz

#Reload env
exec zsh
