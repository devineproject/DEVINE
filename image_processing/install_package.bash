#!/usr/bin/env bash
# Adding devine_segmentation folder to ros package path
CURDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )/devine_image_processing"
echo "export ROS_PACKAGE_PATH=$CURDIR:\$ROS_PACKAGE_PATH" >> ~/.bashrc

#Env setup
sudo pip install imutils
sudo pip install opencv-python
sudo pip3 install Cython scikit-image bson pymongo pycocotools keras catkin_pkg rospkg
wget https://github.com/matterport/Mask_RCNN/releases/download/v2.0/mask_rcnn_coco.h5
wget http://download.tensorflow.org/models/vgg_16_2016_08_28.tar.gz
tar xzf vgg_16_2016_08_28.tar.gz
rm -f vgg_16_2016_08_28.tar.gz

#Reload env
exec bash
