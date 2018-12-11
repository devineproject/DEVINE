.. _ros-body-tracking:

Bodytracking
############

Description
===========

Detecting people is an important part of our project. By detecting nearby humans, we can follow them using the robots eyes and find potential players. This functionality is provided by `tf-pose-estimation`_.

The body tracking node outputs a JSON which contains a skeleton of all the people in a given image. It is is published on the `image/body_tracking` topic. 

ROS Installation
================

Run the install script `install.sh`

Usage
=====

.. code-block:: bash

   $ rosrun devine_image_processing body_tracking.py __ns:=devine

.. _tf-pose-estimation: https://github.com/ildoonet/tf-pose-estimation
