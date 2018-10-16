#! /usr/bin/env python2
''' Announce selected object once IRL-1 has pointed to it'''

from Queue import Queue, Empty
import json

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import String

from devine_dialog.msg import TtsQuery
from devine_config import topicname
from devine_dialog import TTSAnswerType, send_speech

NODE_NAME = 'game_starter'
TOPIC_SNIPS = topicname('tts_query')
TOPIC_IS_END_OF_GAME = topicname('end_of_game') 
OPEN_POSE_TOPIC = topicname('body_tracking')
TOPIC_NEW_GAME = topicname('new_game')
BYE_BYE_DELAY = 5

class NewGameStarter:
    """ Once the robot has announced which object has been selected, this node restarts a game"""

    def __init__(self):
        self.is_new_game = False
        self.player_has_left = False
        
        rospy.init_node(NODE_NAME)
        self.snips = rospy.Publisher(TOPIC_SNIPS, TtsQuery, queue_size=1) 
        self.new_game = rospy.Publisher(TOPIC_NEW_GAME,Bool,queue_size=1)
        rospy.Subscriber(TOPIC_IS_END_OF_GAME, Bool, self.is_end_of_game_callback)
        rospy.Subscriber(OPEN_POSE_TOPIC, String, self.__is_player_gone__)
        self.rate = rospy.Rate(1)


    def __is_player_gone__(self,data):
        """ A naive way to check if the player has left """
        humans = json.loads(data.data)
        self.player_has_left = (len(humans) == 0)
    
    def __start_new_game__(self):
        """ Restarts game from the getgo"""
        self.new_game.pub(True)
        
    def __restart_game__(self):
        """ Currently starts game from the beggining but could be changed"""
        self.new_game.pub(True)
 
    def is_end_of_game_callback(self,data):
        """ Call back that checks if object has been announced"""
        self.is_new_game = data.data
    
    def run_node(self):
        """ Once the object has been announced, it asks the player if they want to play again and starts a new game"""
        while not rospy.is_shutdown():
            if self.is_new_game is True:
                answer = send_speech(self.snips, "Would you like to play again?", TTSAnswerType.YES_NO)
                if answer ==  "no":
                    while not self.player_has_left:
                        send_speech(snips, "Bye bye!", TTSAnswerType.NO_ANSWER)
                        rospy.sleep(BYE_BYE_DELAY)
                    self.__restart_game__()
                else:
                    self.__start_new_game__()
                self.is_new_game = False
            self.rate.sleep()
            
if __name__ == '__main__':
    node_inst = NewGameStarter()
    node_inst.run_node()
