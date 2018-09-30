#! /usr/bin/env python2

''' Node to visualize object in RViz and broadcast object_frame '''

import rospy
from irl_control import irl_constant
from irl_control.objects import ObjectMaker, ObjectTf
from devine_config import topicname

TOPIC_OBJECT_LOCATION = topicname('guess_location_world')
TOPIC_OBJECT_FRAME = '/object_frame'
TOPIC_ROBOT_BASE_FRAME = irl_constant.ROBOT_LINK['base']


def main():
    ''' Init ROS Node to create marker and broadcast TF at postion /object_location '''

    node_name = 'object'
    rospy.init_node(node_name)
    rospy.loginfo('Running node \'' + node_name + '\'')

    ObjectMaker(TOPIC_OBJECT_LOCATION)
    obj_tf = ObjectTf(TOPIC_OBJECT_LOCATION, TOPIC_OBJECT_FRAME, TOPIC_ROBOT_BASE_FRAME)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        obj_tf.brodcast_tf()
        rate.sleep()

if __name__ == '__main__':
    main()
