.. _ros-body-tracking:

Bodytracking
############

Description
===========

We currently use `Mask R-CNN`_ to detect and segment the objects of our images. It was coded using tensorflow and trained using MSCOCO, which means that the classes it uses to segment objects are compatible with GuessWhat?!

The segmentation node outputs a JSON which contains the class of the object, a box which delimits the object and a segmentation mask. It is is published on the `objects` topic. 

ROS Installation
================

1. Run the install script `source install_package.bash`
2. Install `tf-pose-estimation`_ for python 2.
3. Build the module using catkin_make:

.. code-block:: bash

    roscd
    cd ..
    catkin_make

Usage
=====

.. code-block:: bash

    rosrun devine_image_processing segmentation.py __ns:=devine
    rosrun devine_image_processing features_extraction.py __ns:=devine

.. _tf-pose-estimation: https://github.com/ildoonet/tf-pose-estimation
.. _Mask R-CNN: https://github.com/matterport/Mask_RCNN
