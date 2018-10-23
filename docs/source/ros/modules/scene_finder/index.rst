Head Coordinator
############

Description
===========

Scene finding:

We're using april tags and the apriltags2_ros library to find the scene where the objets are located.

Installation
================

1. Clone the apriltags2_ros repository in your catkin workspace, presumably ~/catkin_ws.

    ``git clone git@github.com:dmalyuta/apriltags2_ros.git``


2. Copy the settings available in `./src/head_coordinator/apriltags2_config` in the config directory of the newly cloned repository under `./apriltags2_ros/config`

3. Build the module using catkin_make:

    ``catkin_make -C ~/catkin_ws``

Usage
=====
Using a kinect, place two 11cm by 11cm tag36h11 identified 0 and 1 in the top left and botom right corners of the scene you are trying to find.

.. code-block:: bash

    roslaunch devine devine.launch launch_all:=false kinect:=true find_scene:=true

The robot's head should turn in order to find the scene when the `zone_detection` topic is triggered.