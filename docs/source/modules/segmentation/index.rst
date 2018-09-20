Segmentation
############

Description
===========

We currently use `Mask R-CNN`_ to detect and segment the objects of our images.

TODO: More in depth: https://trello.com/c/nuJhnfI9

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

    rosrun devine_image_processing segmentation.py
    rosrun devine_image_processing features_extraction.py

.. _tf-pose-estimation: https://github.com/ildoonet/tf-pose-estimation
.. _Mask R-CNN: https://github.com/matterport/Mask_RCNN
