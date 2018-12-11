.. _ros-segmentation:

Segmentation
############

Description
===========

We currently use `Mask R-CNN`_ to detect and segment the objects of our images. It was coded using tensorflow and trained using MSCOCO, which means that the classes it uses to segment objects are compatible with GuessWhat?!

The segmentation node outputs a `SegmentedImage` object which contains the class of the object, a box which delimits the object and a segmentation mask. It is is published on the `objects` topic. 

ROS Installation
================

Run the install script `install.sh`

Usage
=====

.. code-block:: bash

    $ rosrun devine_image_processing segmentation.py __ns:=devine

.. _Mask R-CNN: https://github.com/matterport/Mask_RCNN
