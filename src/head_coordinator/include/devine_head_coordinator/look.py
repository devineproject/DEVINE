""" Head mover helper library """
__author__ = "Jordan Prince Tremblay, Ismael Balafrej, Felix Labelle, Félix Martel-Denis, Eric Matte, Adam Letourneau, Julien Chouinard-Beaupré, Antoine Mercier-Nicol"
__copyright__ = "Copyright 2018, DEVINE Project"
__credits__ = ["Simon Brodeur", "François Ferland", "Jean Rouat"]
__license__ = "BSD"
__version__ = "1.0.0"
__email__ = "devine.gegi-request@listes.usherbrooke.ca"
__status__ = "Production"

import rospy
import actionlib
from devine_config import topicname
from std_msgs.msg import Bool
from devine_head_coordinator.msg import LookAtHumanAction, LookAtHumanGoal

SCENE_DETECTION_TOPIC = topicname('start_scene_detection')
TOPIC_SCENE_FOUND = topicname('scene_found')

class Look(object):
    """ Easy to use intent to movement translator """

    def __init__(self):
        self._human_finder_client = actionlib.SimpleActionClient(
            'human_finder', LookAtHumanAction)
        self._human_finder_client.wait_for_server()
        self.humans_count = 0
        self._start_scene_publisher = rospy.Publisher(SCENE_DETECTION_TOPIC, Bool, queue_size=1)

    def at_human(self):
        """ Block until a human is found, and then look at him/her """
        # TODO: Cancel look_at_scene_goal
        rospy.logdebug('Trying to look at human...')
        self.humans_count = 0  # Reset previous human count
        goal = LookAtHumanGoal(period=rospy.rostime.Duration(0))
        self._human_finder_client.send_goal(
            goal, feedback_cb=self._human_feedback_callback)
        self._wait_for_human()
        rospy.logdebug('Now looking at human')

    def at_scene(self):
        """ Block until a scene is found, and then look at it """
        rospy.logdebug('Trying to look at scene...')
        self._human_finder_client.cancel_all_goals()
        self._start_scene_publisher.publish(True)
        scene_found = rospy.wait_for_message(TOPIC_SCENE_FOUND, Bool, timeout=None)
        rospy.logdebug('Now looking at scene' if scene_found else 'Scene not found')
        return scene_found

    def _human_feedback_callback(self, feedback):
        """ Callback method to update the human_count from the feedback CB """
        self.humans_count = feedback.nb_humans

    def _wait_for_human(self):
        """ Blocking call until a human was detected """
        while not rospy.is_shutdown():
            if self.humans_count > 0:
                break
            rospy.sleep(0.1)
