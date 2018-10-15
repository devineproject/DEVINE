#! /usr/bin/env python2
''' Announce selected object once IRL-1 has pointed to it'''

from Queue import Queue, Empty
import rospy

from std_msgs.msg import Bool
from std_msgs.msg import String

from devine_config import topicname
from devine_dialog.msg import TtsQuery
from devine_dialog import TTSAnswerType, send_speech

NODE_NAME = 'announce_object'
TOPIC_OBJECT_CATEGORY = topicname('guess_category')
TOPIC_SNIPS = topicname('tts_query')
TOPIC_IS_POINTING_OBJECT = topicname('is_pointing_object')
TOPIC_IS_END_OF_GAME = topicname('end_of_game')
TRANSITION_DELAY = 1  # Amount of time waited before asking for new game


class AnnounceNode:
    """ This node announces the selected object after it has been pointed"""
    def __init__(self):
        self.current_category = ""
        self.pointing_state = False
        rospy.init_node(NODE_NAME)
        rospy.Subscriber(TOPIC_OBJECT_CATEGORY, String, self.category_callback)
        rospy.Subscriber(TOPIC_IS_POINTING_OBJECT, Bool, self.is_pointing_callback)
        end_of_game = rospy.Publisher(TOPIC_IS_END_OF_GAME, Bool, queue_size=1)
        snips = rospy.Publisher(TOPIC_SNIPS, TtsQuery, queue_size=1)
        rospy.sleep(1)

    def category_callback(self, data):
        '''Callback for the category topic'''
        self.current_category = data.data

    def is_pointing_callback(self, data):
        '''Callback for the is_pointing topic'''
        self.pointing_state = data.data

    def run_node(self):
        ''' Checks if selected object has been pointed to and announces its category '''
        rate = rospy.Rate(1)  # TBD
        while not rospy.is_shutdown():
            if self.pointing_state is True:
                send_speech(snips, self.current_category, TTSAnswerType.NO_ANSWER)
                rospy.sleep(TRANSITION_DELAY)
                end_of_game.publish(True)
                self.pointing_state = False
            rate.sleep()


if __name__ == '__main__':
    node_inst = AnnounceNode()
    node_inst.run_node()
