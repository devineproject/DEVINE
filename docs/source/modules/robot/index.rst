Robot
#####

Description
===========

We currently use the `IntroLab`_'s robot IRL-1 for our demonstrations.

TODO: Cleaner page and more information.

Dependencies
============

See `package.xml` for dependencies.

Details
=======

Arms controller use `JointTrajectory`

Head controller use `JointTrajectory`

Grippers use `Command`

* `marker.py` use:
    * `MarkerArray` to show position to point in RVIZ
    * `TF` to broadcast position to point and to calculate the orientations error (should be +/- 5 degrees).

* `example.py` use:
    * `TF` to listen position between `/[right|left]_shoulder_fixed_link` and position to point `/obj`
    * `ik.py` to get joint position knowing position to point (see documentation from Arnaud Aumont, IntRoLab, version 22/01/2012)

Constants
=========

File `irl_constant.py` contains

* Controllers names
* Joints names
* Joints limits

Possible Mouvements
===================

* Point To Position (x, y, z) with
    * Right arm
    * Left arm
    * Head
* Open And Close
    * Right gripper
    * Left gripper

* SIMULATION ONLY, Do complex movements with arms and head:
    * Happy (confidence >= threshold, success 1)
    * Satisfied (confidence < threshold, success 1)
    * Disappointed (confidence >= threshold, success 0)
    * Sad (confidence < threshold, success 0)

* TODO Facial expression

Running Examples
================

Before running any examples, you need to:

1. Launch jn0 with RViz UI and irl_control nodes

.. code-block:: bash

    roslaunch irl_control irl_control.launch

2. Load RVIZ configuration

.. code-block:: bash

    File -> Open Config -> irl_point.rviz

You can now execute any of the examples:

* Point to position [x, y, z]

.. code-block:: bash

    rosrun irl_control example_point.py -p 0.6,0.3,0.5
    # Position is referenced from base_link

* Do complex move (SIMULATION ONLY!!!)

.. code-block:: bash

    rosrun irl_control example_emotion.py -c 0 -s 0

.. _IntroLab: https://introlab.3it.usherbrooke.ca/mediawiki-introlab/index.php/Autonomous_Robot