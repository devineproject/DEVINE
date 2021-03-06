# -*- coding: utf-8 -*-
__author__ = "Jordan Prince Tremblay, Ismael Balafrej, Felix Labelle, Felix Martel-Denis, Eric Matte, Adam Letourneau, Julien Chouinard-Beaupre, Antoine Mercier-Nicol"
__copyright__ = "Copyright 2018, DEVINE Project"
__credits__ = ["Simon Brodeur", "Francois Ferland", "Jean Rouat"]
__license__ = "BSD"
__version__ = "1.0.0"
__email__ = "devine.gegi-request@listes.usherbrooke.ca"
__status__ = "Production"

import random
import json
import uuid
from enum import Enum
import rospy
from devine_config import topicname
from devine_dialog.msg import TtsQuery, TtsAnswer
from devine_common import ros_utils

TTS_ANSWER_TOPIC = topicname('tts_answer')
DIALOGS = None


class TTSAnswerType(Enum):
    NO_ANSWER = 0
    YES_NO = 1
    PLAYER_NAME = 2


def load_dialogs():
    """ Load dialogs from json file """
    global DIALOGS
    if not DIALOGS:
        with open(ros_utils.get_fullpath(__file__, 'static/dialogs.json')) as f:
            DIALOGS = json.loads(f.read())
    return DIALOGS


def wait_for_answer(answer_type, message_uid):
    """ Wait on the TTS answer topic for the answer to the question """
    while not rospy.is_shutdown():
        answer = rospy.wait_for_message(
            TTS_ANSWER_TOPIC, TtsAnswer, timeout=None)
        if answer.original_query.uid == message_uid:
            return answer.text if answer.probability > 0.4 else None


def send_speech(tts_publisher, message, answer_type):
    """ Convert text string to speech using the snips topic """
    msg = TtsQuery()
    msg.text = message
    msg.uid = uuid.uuid4().int & 0xFFFFFFFF  # 32 LSB of random uid
    msg.answer_type = answer_type.value
    tts_publisher.publish(msg)  # Send the payload
    if answer_type != TTSAnswerType.NO_ANSWER:
        dialogs = load_dialogs()
        answer = wait_for_answer(answer_type, msg.uid)
        if answer is None:
            for i in dialogs['did_not_understand'] + dialogs['say_again']:
                message = message.replace(i, '')
            repeat_query = random.choice(
                dialogs['did_not_understand']) + random.choice(dialogs['say_again'])
            return send_speech(tts_publisher, repeat_query + message, answer_type)
        return answer
    return None
