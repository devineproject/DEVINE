#! /usr/bin/env python2
""" Node to visualize object in RViz and broadcast object_frame """

import rospy
from devine_irl_control.objects import ObjectMaker
from devine_config import topicname

TOPIC_OBJECT_LOCATION = topicname('guess_location_world')


def main():
    """ Init ROS Node to create marker and broadcast TF at postion /object_location """
    node_name = 'devine_irl_control_object'
    rospy.init_node(node_name)
    rospy.loginfo('Running node \'%s\'', node_name)

    ObjectMaker(TOPIC_OBJECT_LOCATION)
    rospy.spin()


if __name__ == '__main__':
    main()
