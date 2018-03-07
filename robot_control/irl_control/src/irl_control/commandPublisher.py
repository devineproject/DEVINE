#! /usr/bin/env python

import sys, argparse
import rospy

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

class CommandPublisher(object):
    def __init__(self, robotName, controllerName):
        topic = '/'.join(['', robotName, controllerName, 'command'])
        self._pub = rospy.Publisher(topic, Float64, queue_size=10)
        rospy.wait_for_message('/'.join([robotName, 'joint_states']), JointState)

    def publish(self, position):
        self._pub.publish(Float64(position))
