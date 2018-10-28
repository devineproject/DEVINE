#!/usr/bin/env python3
''' Dialog controller of the DEVINE project - Decides when and what to send to the TTS engine'''

import os
import json
import rospy
import time
import random
from devine_dialog.msg import TtsQuery
from std_msgs.msg import String, Bool
from devine_config import topicname
from devine_dialog import TTSAnswerType, send_speech

#IN topics
HUMAN_READY_DETECTED_TOPIC = topicname('body_tracking')
IS_POINTING_OBJECT_TOPIC = topicname('is_pointing_object')
OBJECT_CATEGORY_TOPIC = topicname('guess_category')
EXPRESSION_DONE_TOPIC = topicname('robot_facial_expression_completed')

#OUT topics
TTS_PUBLISHER = rospy.Publisher(topicname('tts_query'), TtsQuery, queue_size=10)
READY_TO_PLAY_PUBLISHER = rospy.Publisher(topicname('player_name'), String, queue_size=1)
GUESS_SUCCEEDED = rospy.Publisher(topicname('object_guess_success'), Bool, queue_size=1)
START_NEW_GAME = rospy.Publisher(topicname('new_game'), Bool, queue_size=1)

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
CONST_FILE = os.path.join(SCRIPT_DIR, 'dialogs.json')

class DialogControl():
    
    def __init__(self):
        dialogs = open(CONST_FILE)
        self.dialogs = json.loads(dialogs.read())
        self.can_start = True

    def send_sentence(self, sentence, answer_type, **format_args):
        '''Wrapper to randomly send a sentence from an array with send_speech'''
        return send_speech(TTS_PUBLISHER, random.choice(self.dialogs[sentence]).format(**format_args), answer_type)

    def wait_for_player(self, message_name, **format_args):
        '''Ask 5 time the human a question. Return true if he did answer 'yes' one time.'''
        for _i in range(2):
            answer = self.send_sentence(message_name, TTSAnswerType.YES_NO, **format_args)
            if answer == 'yes' or not answer:
                break
            time.sleep(5) #The 5 seconds includes the time it takes for the TTS engine to say the sentence
            
        return answer == 'yes'

    class HumanDialogInterrupted(Exception):
        '''Exception type for interruptions of the dialog'''
        pass

    def on_human_detected(self, human_object):
        '''When a human is detected, begin the robot/human dialog'''
        humans = json.loads(human_object.data)
        if len(humans) == 0 or not self.can_start: # No humans detected
            return
        self.can_start = False
        try:
            self.send_sentence('welcome', TTSAnswerType.NO_ANSWER)
            answer = self.send_sentence('ask_to_play', TTSAnswerType.YES_NO)

            if not answer == "yes":
                raise DialogControl.HumanDialogInterrupted()

            player_name = self.send_sentence('asking_the_name', TTSAnswerType.PLAYER_NAME)

            self.send_sentence('instructions', TTSAnswerType.NO_ANSWER)
            if not self.wait_for_player('ready', player_name=player_name):
                raise DialogControl.HumanDialogInterrupted()
            
            READY_TO_PLAY_PUBLISHER.publish(player_name)

            object_cat_name = rospy.wait_for_message(OBJECT_CATEGORY_TOPIC, String).data
            rospy.wait_for_message(IS_POINTING_OBJECT_TOPIC, Bool)

            answer = self.send_sentence('ask_got_it_right', TTSAnswerType.YES_NO, object_name=object_cat_name)
            
            GUESS_SUCCEEDED.publish(answer == 'yes')

            rospy.wait_for_message(EXPRESSION_DONE_TOPIC, Bool)
            
            answer = self.send_sentence('ask_play_again', TTSAnswerType.YES_NO)

            if answer == "no":
                raise DialogControl.HumanDialogInterrupted()

            START_NEW_GAME.publish(True)

        except DialogControl.HumanDialogInterrupted:
            self.send_sentence('bye_bye', TTSAnswerType.NO_ANSWER)
            rospy.sleep(8) #Sleep while the player leaves
        finally:
            self.can_start = True


def hook_listeners():
    dialog_control = DialogControl()
    rospy.Subscriber(HUMAN_READY_DETECTED_TOPIC, String, dialog_control.on_human_detected)

if __name__ == '__main__':
    rospy.init_node('dialog_control')
    hook_listeners()
    rospy.spin()
