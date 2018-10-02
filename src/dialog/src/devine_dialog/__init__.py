import random
import uuid
from enum import Enum
import rospy
from devine_config import topicname
from devine_dialog.msg import TtsQuery

TTS_ANSWER_TOPIC = topicname('tts_answer')

class TTSAnswerType(Enum):
    NO_ANSWER = 0
    YES_NO = 1
    PLAYER_NAME = 2

def wait_for_answer(answer_type, message_uid):
    '''Wait on the TTS answer topic for the answer to the question'''
    answer = None
    while not rospy.is_shutdown():
        answer = rospy.wait_for_message(TTS_ANSWER_TOPIC, TtsQuery, timeout=None) #TODO: Handle timeouts ? -> throws a RosException when timeout
        if answer.uid == message_uid:
            break
    if answer:
        return answer.text
    return None

def send_speech(tts_publisher, message, answer_type):
    '''Convert text string to speech using the snips topic'''
    msg = TtsQuery()
    msg.text = message
    msg.uid = uuid.uuid4().int & 0xFFFFFFFF #32 LSB of random uid
    msg.answer_type = answer_type.value
    tts_publisher.publish(msg) # Send the payload
    if answer_type != TTSAnswerType.NO_ANSWER:
        return wait_for_answer(answer_type, msg.uid)
    return None
