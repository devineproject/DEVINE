'''ROS wrappers for guesswhat models'''

from queue import Queue, Empty

import rospy
from std_msgs.msg import String, Float64MultiArray
from devine_dialog.msg import TtsQuery
from devine_dialog import TTSAnswerType, send_speech

from guesswhat.models.guesser.guesser_wrapper import GuesserWrapper
from devine_config import topicname

TTS_ANSWER_TOPIC = topicname('tts_answer')
TTS_QUERY_TOPIC = topicname('tts_query')
CONFIDENCE_TOPIC = topicname('objects_confidence')

class GuesserROSWrapper(GuesserWrapper):
    '''Wraps the guesser model and publishes confidence levels'''
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.confidence = rospy.Publisher(CONFIDENCE_TOPIC, Float64MultiArray, queue_size=1, latch=True)

    def find_object(self, *args, **kwargs):
        '''find_object interface for the looper'''
        found, softmax, selected_object = super().find_object(*args, **kwargs)
        found = [False] # found is based on the oracle's choice, which we dont know

        self.confidence.publish(Float64MultiArray(data=softmax[0].tolist()))

        return found, softmax, selected_object

class OracleROSWrapper(object):
    '''Wraps the oracle model, publishes questions and waits for their answers'''
    def __init__(self, tokenizer):
        self.tokenizer = tokenizer
        self.questions = rospy.Publisher(TTS_QUERY_TOPIC, TtsQuery, queue_size=1, latch=True)

    def initialize(self, sess):
        '''initialize interface'''
        pass

    def answer_question(self, sess, question, **kwargs):
        '''answer_question interface for the looper'''
        if self.tokenizer.stop_dialogue in question[0]:
            return [self.tokenizer.non_applicable_token]

        text_question = self.tokenizer.decode(question[0]).replace('<padding>', '').strip()

        answer = send_speech(self.questions, text_question, TTSAnswerType.YES_NO)

        if answer == 'yes':
            token = self.tokenizer.yes_token
        elif answer == 'no':
            token = self.tokenizer.no_token
        elif answer == 'na':
            token = self.tokenizer.non_applicable_token
        else:
            rospy.logerr('Garbage on {}, expects yes|no|na'.format(TTS_ANSWER_TOPIC))
            exit(1)

        return [token]
