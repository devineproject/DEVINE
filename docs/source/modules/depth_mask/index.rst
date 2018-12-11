.. _ros-depth-mask:

Depth mask
############

Description
===========

To filter out extraneous objects in the background, the kinect's depth sensor is used to create a mask. This mask blacks out all objects further then 1.5m.

The body tracking node outputs the masked image. It is is published on the `sensor_msgs/CompressedImage` topic.

ROS Installation
================

1. Run the install script `install.sh`

Usage
=====

.. code-block:: bash

    rosrun devine_image_processing mask.py __ns:=devine

