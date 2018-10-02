#!/usr/bin/env python3
''' Dialog controller of the DEVINE project - Decides when and what to send to the TTS engine'''

import os
import json
import rospy
import time
import random
import uuid
from enum import Enum
from devine_dialog.msg import TtsQuery
from std_msgs.msg import String

from devine_config import topicname

TTS_ANSWER_TOPIC = topicname('tts_answer')
HUMAN_READY_DETECTED_TOPIC = topicname('body_tracking')

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
CONST_FILE = os.path.join(SCRIPT_DIR, 'dialogs.json')

class TTSAnswerType(Enum):
    NO_ANSWER = 0
    YES_NO = 1
    PLAYER_NAME = 2

class DialogControl():
    TTS_PUBLISHER = rospy.Publisher(topicname('tts_query'), TtsQuery, queue_size=10)
    READY_TO_PLAY_PUBLISHER = rospy.Publisher(topicname('player_name'), String, queue_size=10)
    
    def __init__(self):
        dialogs = open(CONST_FILE)

        self.dialogs = json.loads(dialogs.read())

    def wait_for_answer(self, answer_type, message_uid):
        '''Wait on the TTS answer topic for the answer to the question'''
        answer = None
        while not rospy.is_shutdown():
            answer = rospy.wait_for_message(TTS_ANSWER_TOPIC, TtsQuery, timeout=None) #TODO: Handle timeouts ? -> throws a RosException when timeout
            if answer.uid == message_uid:
                break
        if answer:
            return answer.text
        return None

    def send_speech(self, message_name, answer_type, **format_args):
        '''Convert text string to speech using the snips topic'''
        msg = TtsQuery()
        msg.text = random.choice(self.dialogs[message_name]).format(**format_args)
        msg.uid = uuid.uuid4().int & 0xFFFFFFFF #32 LSB of random uid
        msg.answer_type = answer_type.value
        self.TTS_PUBLISHER.publish(msg) # Send the payload
        if answer_type != TTSAnswerType.NO_ANSWER:
            return self.wait_for_answer(answer_type, msg.uid)
        return None

    def wait_for_player(self, message_name, **format_args):
        '''Ask 5 time the human a question. Return true if he did answer 'yes' one time.'''
        for _i in range(5):
            answer = self.send_speech(message_name, TTSAnswerType.YES_NO, **format_args)
            if answer == 'yes' or not answer:
                break
            time.sleep(5) #The 5 seconds includes the time it takes for the TTS engine to say the sentence
            
        return answer == 'yes'


    def on_human_detected(self, human_object):
        '''When a human is detected, begin the robot/human dialog'''
        humans = json.loads(human_object.data)
        if len(humans) == 0: # No humans detected
            return

        self.send_speech('welcome', TTSAnswerType.NO_ANSWER)
        answer = self.send_speech('ask_to_play', TTSAnswerType.YES_NO)

        if not answer == 'yes':
            self.send_speech('bye_bye', TTSAnswerType.NO_ANSWER)
            return

        player_name = self.send_speech('asking_the_name', TTSAnswerType.PLAYER_NAME)
    
        if not self.wait_for_player('ready', first_name=player_name):
            self.send_speech('bye_bye', TTSAnswerType.NO_ANSWER)
            return

        self.send_speech('instructions', TTSAnswerType.NO_ANSWER)
        if not self.wait_for_player('ready', first_name=player_name):
            self.send_speech('bye_bye', TTSAnswerType.NO_ANSWER)
            return
        
        self.READY_TO_PLAY_PUBLISHER.publish(player_name)


def hook_listeners():
    dialog_control = DialogControl()
    rospy.Subscriber(HUMAN_READY_DETECTED_TOPIC, String, dialog_control.on_human_detected)

if __name__ == '__main__':
    rospy.init_node('dialog_control')
    hook_listeners()
    rospy.spin()
