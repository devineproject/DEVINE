#! /usr/bin/env python

import rospy

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

class CommandPublisher(object):
    def __init__(self, robot_name, controller_name):
        topic = '/'.join(['', robot_name, controller_name, 'command'])
        self._pub = rospy.Publisher(topic, Float64, queue_size=10)
        rospy.wait_for_message('/'.join([robot_name, 'joint_states']), JointState)

    def publish(self, position):
        self._pub.publish(Float64(position))
