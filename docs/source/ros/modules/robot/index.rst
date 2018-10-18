Robot Behavior
##############

Description
===========

.. image:: irl1_full.jpg
   :scale: 50 %
   :alt: alternate text
   :align: right

We currently use robot IRL-1_ from IntRoLab_ for our demonstrations. See official `IRL-1 GitHub`_ for more details.

.. _IntRoLab: https://introlab.3it.usherbrooke.ca
.. _IRL-1: https://introlab.3it.usherbrooke.ca/mediawiki-introlab/index.php/Autonomous_Robot
.. _IRL-1 GitHub: https://github.com/introlab/IRL-1

Possible Mouvements
===================

* Point to position (x, y, z) with
    * Right arm
    * Left arm
    * Head
* Open and close
    * Right gripper
    * Left gripper
* SIMULATION ONLY, Do complex movements with arms and head:
    * Happy (confidence >= threshold, success 1)
    * Satisfied (confidence < threshold, success 1)
    * Disappointed (confidence >= threshold, success 0)
    * Sad (confidence < threshold, success 0)
* TODO Facial expression
    * Surprise
    * Anger
    * Joy


Running Examples
================

Before running any examples, you need to:

1. Launch jn0 with RViz UI and devine_irl_control nodes

.. code-block:: bash

    roslaunch devine_irl_control devine_irl_control.launch jn0:=true

2. Load RVIZ configuration

.. code-block:: bash

    File -> Open Config -> src/robot_control/launch/irl_point.rviz

You can now execute any of the examples:

* Point to position [x, y, z]

.. code-block:: bash

    rosrun devine_irl_control example_point.py --point 0.6,0.3,0.5 --look 1,-0.6,0.5
    # Position is referenced from base_link

* Do complex move (SIMULATION ONLY!!!)

.. code-block:: bash

    rosrun devine_irl_control example_emotion.py -c 0 -s 0

Dependencies
============

See `package.xml` for dependencies.

Topics
=======

Topics input and output from this module

+--------+-------------------------------------+-----------------------------------------+
| In/Out | Topic                               | ROS Message                             |
+========+=====================================+=========================================+
| In     | /devine/guess_location/world        | `geometry_msgs/PoseStamped`_ *          |
+--------+-------------------------------------+                                         +
| In     | /devine/robot/robot_look_at         |                                         |
+--------+-------------------------------------+-----------------------------------------+
| In     | /devine/robot/head_joint_traj_point | `trajectory_msgs/JointTrajectoryPoint`_ |
+--------+-------------------------------------+-----------------------------------------+
| Out    | /devine/robot/is_pointing           |`std_msgs/Bool`_                         |
+--------+-------------------------------------+                                         +
| Out    | /devine/robot/is_looking            |                                         |
+--------+-------------------------------------+-----------------------------------------+
| Out    | /devine/robot/err_pointing          |`std_msgs/Float64MultiArray`_            |
+--------+-------------------------------------+-----------------------------------------+

\* PoseStamped are relative to `base_link` (see `frame_id`)

.. _geometry_msgs/PoseStamped: http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html
.. _trajectory_msgs/JointTrajectoryPoint: http://docs.ros.org/api/trajectory_msgs/html/msg/JointTrajectoryPoint.html
.. _std_msgs/Bool: http://docs.ros.org/api/std_msgs/html/msg/Bool.html
.. _std_msgs/Float64MultiArray: http://docs.ros.org/api/std_msgs/html/msg/Float64MultiArray.html

Constants
=========

File `irl_constant.py` contains

* Controllers names
* Joints names
* Joints limits
