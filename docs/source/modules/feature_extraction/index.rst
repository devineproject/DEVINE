.. _ros-feature-extraction:

Feature extraction
##################

Description
===========

`VGG-16`_ is used to extract image features which was in turn used by the question generator. It was coded using tensorflow and is available on github. 

The feature extraction node outputs an array which contains the class of the object, which contains the FC8 layer's output. It is is published on the `features` topic. 

ROS Installation
================

Run the install script `source install_package.sh`

Usage
=====

.. code-block:: bash

   $ rosrun devine_image_processing features_extraction.py __ns:=devine

.. _VGG-16: https://github.com/tensorflow/models/tree/master/research/slim
