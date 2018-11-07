ROS
###

CheatSheet
==========

.. role:: bash(code)
   :language: bash

Here you can see a couple of usefull ROS commands to help you out!


* :bash:`$ roscore`

    * Starts the ros core node, you need this before starting any other node.

* :bash:`$ rosrun {rosPackageName} {pythonFileContainingTheRosNode}`

    * Example: :bash:`$ rosrun devine_irl_control node_facial_expression.py`
    * This will start the node specified inside the `node_facial_expression.py`

* :bash:`$ rostopic pub {/topic_name} std_msgs/{dataType} {Payload}`

    * Example: :bash:`$ rostopic pub /devine/objects_confidence std_msgs/Float64MultiArray "{layout: {dim: [{label: '', size: 0, stride: 0}], data_offset: 0}, data: [0,0.8, 0.7]}"`
    * This will publish the specified payload to the specified topic.

* :bash:`$ rostopic echo {topicName}`

    * Example: :bash:`$ rostopic echo /devine/robot/facial_expression`
    * This will listen and print out any messages on the specified topic.

* :bash:`$ roslaunch devine devine.launch`

    * This will launch **ALL** Devine nodes.
    * You can also use this to launch specific nodes like so :bash:`$ roslaunch devine devine.launch launch_all:=false dashboard:=true` 

* :bash:`$ rosrun topic_tools throttle messages /openni/rgb/image_color/compressed 0.33 /devine/image/segmentation`

    * Segments every 30 seconds 

* :bash:`$ rosrun rqt_gui rqt_gui`

    * Starts a GUI with many usefull ROS development tools that enables you to subscribe and monitor ROS topics for example.

* :bash:`$ rosrun rqt_top rqt_top`

    * See the actually ressources consumed by your ROS environment.

Modules
=======

All DEVINE modules:

.. toctree::
    :glob:
    :maxdepth: 1
   
    modules/*/index


Tests
=====

.. toctree::
    :glob:
    :maxdepth: 1
   
    tests/index