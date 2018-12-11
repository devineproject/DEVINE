.. _ros-image-dispatcher:

Image disptacher
#################

Description
===========

The image dispatcher is responsible for distributing images from the kinect to the various modules that need them in the correct order. It takes raw images, checks them for blur, applies the depth mask and sends the processed images to be segmented and have their features extracted. 


ROS Installation
================

Run the install script `install.sh`

Usage
=====

.. code-block:: bash

  $ rosrun devine_image_processing image_dispatcher.py __ns:=devine
