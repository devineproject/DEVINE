#!/usr/bin/env python2
""" Simple test of the human finder """
import rospy

import actionlib

from devine_head_coordinator.msg import LookAtHumanAction, LookAtHumanGoal

def get_result():
    """ Call the human finder actionlib and get the result """
    client = actionlib.SimpleActionClient('devine_human_finder', LookAtHumanAction)

    client.wait_for_server()

    rospy.loginfo('Sending goal')
    client.send_goal(LookAtHumanGoal(period=rospy.rostime.Duration(0)))

    rospy.loginfo('Waiting for results')
    client.wait_for_result()

    return client.get_result() 

if __name__ == '__main__':
    rospy.init_node('test_human_client')
    result = get_result()
    rospy.loginfo('Result: %s', str(result))
