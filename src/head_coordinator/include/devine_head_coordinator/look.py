""" Head mover helper library """

import rospy
import actionlib
from devine_head_coordinator.msg import LookAtHumanAction, LookAtHumanGoal

class Look(object):
    """ Easy to use intent to movement translator """
    def __init__(self):
        self.human_finder_client = actionlib.SimpleActionClient('devine_human_finder', LookAtHumanAction)
        self.human_finder_client.wait_for_server()
        self.humans_count = 0

    def at_human(self):
        """ Block until a human is found, and then look at him/her """
        self.humans_count = 0 #Reset previous human count
        goal = LookAtHumanGoal(period=rospy.rostime.Duration(0))
        self.human_finder_client.send_goal(goal, feedback_cb=self._human_feedback_callback)
        self._wait_for_human()

    def at_scene(self, time=0):
        """ Block until a scene is found, and then look at it """
        self.human_finder_client.cancel_all_goals()
        # TODO look and wait for scene

    def _human_feedback_callback(self, feedback):
        """ Callback method to update the human_count from the feedback CB """
        self.humans_count = feedback.nb_humans

    def _wait_for_human(self):
        """ Blocking call until a human was detected """
        while not rospy.is_shutdown():
            if self.humans_count > 0:
                break
            rospy.sleep(0.1)
