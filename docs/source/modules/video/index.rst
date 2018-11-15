Video
#####

Description
===========

We currently use a Microsoft `Kinect for a Xbox 360`_ in combination with OpenNI to use it inside the ROS ecosystem.

Pre requirement Installation
============================

1. Install OpenNI

.. code-block:: bash

    sudo apt-get install ros-kinetic-openni-launch ros-kinetic-openni-camera ros-kinetic-openni-description
    sudo apt-get install ros-kinetic-compressed-image-transport #Image compression plugin


2. Start OpenNI server

.. code-block:: bash

    roslaunch devine devine.launch launch_all:=false kinect:=true dashboard:=true


3. View Data 

You can use the dashboard (http://localhost:8080) or the image_view package:

.. code-block:: bash

    rosrun image_view image_view image:=/openni/rgb/image_color #color
    rosrun image_view image_view image:=/openni/rgb/image_mono #mono
    rosrun image_view disparity_view image:=/openni/depth_registered/disparity #disparity

4. Read the ROS `OpenNI documentation`_ for more info!

ROS Installation
================

1. Run the install script `./install_package.bash`
2. Build the module using `catkin_make`:

.. code-block:: bash

    roscd
    cd ..
    catkin_make

.. _Kinect for a Xbox 360: https://en.wikipedia.org/wiki/Kinect#Kinect_for_Xbox_360_(2010)
.. _OpenNI documentation: http://wiki.ros.org/openni_launch/