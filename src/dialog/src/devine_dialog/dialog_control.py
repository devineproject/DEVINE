#!/usr/bin/env python3
""" Dialog controller of the DEVINE project - Decides when and what to send to the TTS engine """
__author__ = "Jordan Prince Tremblay, Ismael Balafrej, Felix Labelle, Félix Martel-Denis, Eric Matte, Adam Letourneau, Julien Chouinard-Beaupré, Antoine Mercier-Nicol"
__copyright__ = "Copyright 2018, DEVINE Project"
__credits__ = ["Simon Brodeur", "François Ferland", "Jean Rouat"]
__license__ = "BSD"
__version__ = "1.0.0"
__email__ = "devine.gegi-request@listes.usherbrooke.ca"
__status__ = "Production"

import json
import time
import random
from std_msgs.msg import String, Bool
import rospy
from devine_dialog.msg import TtsQuery
from devine_dialog import TTSAnswerType, send_speech
from devine_config import topicname
from devine_common import ros_utils
from devine_head_coordinator.look import Look

# IN topics
HUMAN_READY_DETECTED_TOPIC = topicname('body_tracking')
IS_POINTING_OBJECT_TOPIC = topicname('is_pointing_object')
OBJECT_CATEGORY_TOPIC = topicname('guess_category')
EXPRESSION_DONE_TOPIC = topicname('robot_facial_expression_completed')

# OUT topics
TTS_PUBLISHER = rospy.Publisher(topicname('tts_query'), TtsQuery, queue_size=10)
READY_TO_PLAY_PUBLISHER = rospy.Publisher(topicname('player_name'), String, queue_size=1)
GUESS_SUCCEEDED = rospy.Publisher(topicname('object_guess_success'), Bool, queue_size=1)
START_NEW_GAME = rospy.Publisher(topicname('new_game'), Bool, queue_size=1)

CONST_FILE = ros_utils.get_fullpath(__file__, 'dialogs.json')


class DialogControl():
    """ Class that control the dialog of the game """

    def __init__(self):
        dialogs_file = open(CONST_FILE)
        self.dialogs = json.loads(dialogs_file.read())
        self.is_new_player = True
        self._look = Look()

    def send_sentence(self, sentence, answer_type, **format_args):
        """ Wrapper to randomly send a sentence from an array with send_speech """
        return send_speech(TTS_PUBLISHER, random.choice(self.dialogs[sentence]).format(**format_args), answer_type)

    def wait_for_player(self, message_name, **format_args):
        """ Ask 2 time the human a question. Return true if he did answer 'yes' one time. """
        for _ in range(2):
            answer = self.send_sentence(message_name, TTSAnswerType.YES_NO, **format_args)
            if answer == 'yes' or not answer:
                break
            time.sleep(5)  # The 5 seconds includes the time it takes for the TTS engine to say the sentence

        return answer == 'yes'

    class HumanDialogInterrupted(Exception):
        """ Exception type for interruptions of the dialog """
        pass

    def loop(self):
        """ When a human is detected, begin the robot/human dialog """
        object_category_blocker = ros_utils.TopicBlocker(OBJECT_CATEGORY_TOPIC, String)
        is_pointing_blocker = ros_utils.TopicBlocker(IS_POINTING_OBJECT_TOPIC, Bool)
        expression_done_blocker = ros_utils.TopicBlocker(EXPRESSION_DONE_TOPIC, Bool)


        while not rospy.is_shutdown():
            try:
                self._look.at_human() # Blocks until a human is found
                if self.is_new_player:
                    self.send_sentence('welcome', TTSAnswerType.NO_ANSWER)
                    answer = self.send_sentence('ask_to_play', TTSAnswerType.YES_NO)
                    if not answer == 'yes':
                        raise DialogControl.HumanDialogInterrupted()

                    player_name = self.send_sentence('asking_the_name', TTSAnswerType.PLAYER_NAME)

                self.send_sentence('instructions', TTSAnswerType.NO_ANSWER)
                if not self.wait_for_player('ready', player_name=player_name):
                    raise DialogControl.HumanDialogInterrupted()

                self._look.at_scene()
                READY_TO_PLAY_PUBLISHER.publish(player_name)

                object_found = object_category_blocker.wait_for_message().data

                is_pointing_blocker.wait_for_message()

                answer = self.send_sentence('ask_got_it_right', TTSAnswerType.YES_NO, object_name=object_found)

                GUESS_SUCCEEDED.publish(answer == 'yes')

                expression_done_blocker.wait_for_message()

                answer = self.send_sentence('ask_play_again', TTSAnswerType.YES_NO)
                if answer == 'no':
                    raise DialogControl.HumanDialogInterrupted()
                
                self.is_new_player = False

            except DialogControl.HumanDialogInterrupted:
                self.is_new_player = True
                self.send_sentence('bye_bye', TTSAnswerType.NO_ANSWER)
                rospy.sleep(8) # Sleep while the player leaves
            finally:
                START_NEW_GAME.publish(True)


def hook_listeners():
    """ Create the dialogControl object and hook the listeners """
    dialog_control = DialogControl()
    dialog_control.loop()

if __name__ == '__main__':
    rospy.init_node('dialog_control')
    hook_listeners()
    rospy.spin()
