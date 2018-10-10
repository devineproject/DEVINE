ROS
###

CheatSheet
==========

Here you can see a couple of usefull ROS commands to help you out!

* roscore

    * starts the ros core node, you need this before starting any other node.

* rosrun {rosPackageName} {pythonFileContainingTheRosNode}

    * example: `rosrun devine_irl_control node_facial_expression.py`
    * This will start the node specified inside the `node_facial_expression.py`

* rostopic pub {/topic_name} std_msgs/{dataType} {Payload}

    * example: `rostopic pub /devine/objects_confidence std_msgs/Float64MultiArray "{layout: {dim: [{label: '', size: 0, stride: 0}], data_offset: 0}, data: [0,0.8, 0.7]}"`
    * This will publish the specified payload to the specified topic.

* rostopic echo {topicName}

    * example: `rostopic echo /devine/robot/facial_expression`
    * This will listen and print out any messages on the specified topic.

* roslaunch devine devine.launch

    * This will launch **ALL** Devine nodes.
    * You can also use this to launch specific nodes like so `roslaunch devine devine.launch launch_all:=false dashboard:=true` 


Modules
=======

All DEVINE modules:

.. toctree::
    :glob:
    :maxdepth: 1
   
    modules/*/index