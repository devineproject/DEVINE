#!/usr/bin/env python2
"""
Node to find a human
"""
import rospy
import actionlib
from devine_head_coordinator.msg import (LookAtHumanAction, LookAtHumanResult, LookAtHumanFeedback)

class LookAtHumanActionServer(object):
    '''Action server to look at humans for a fixed period of time'''

    def __init__(self, name = rospy.get_name()):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, LookAtHumanAction, execute_cb=self.execute_cb)
        self._feedback = LookAtHumanFeedback()
        self._result = LookAtHumanResult()
      
    def execute_cb(self, goal):
        '''Callback when executing an action'''
        # helper variables
        rate = rospy.Rate(1)
        success = True
        
        # Init states
        self._feedback.nb_humans = 0
        self._result.nb_humans = 0
        
        # Starting
        rospy.loginfo("%s: Trying to find a human...", self._action_name)
        
        # Should be in the loop if any
        if self._as.is_preempt_requested():
            rospy.loginfo("%s: Preempted", self._action_name)
            self._as.set_preempted()
            success = False
            return

        self._feedback.nb_humans = 0
        self._as.publish_feedback(self._feedback)
        rate.sleep()

        if success:
            self._result.nb_humans = self._feedback.nb_humans
            rospy.loginfo("%s: Succeeded", self._action_name)
            self._as.set_succeeded(self._result)


def main():
    '''Entry point of this file'''
    rospy.init_node('devine_human_finder')
    LookAtHumanActionServer()
    rospy.spin()

if __name__ == '__main__':
    main()
