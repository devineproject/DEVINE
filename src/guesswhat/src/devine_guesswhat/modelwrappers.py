# -*- coding: utf-8 -*-
""" ROS wrappers for guesswhat models """
__author__ = "Jordan Prince Tremblay, Ismael Balafrej, Felix Labelle, Felix Martel-Denis, Eric Matte, Adam Letourneau, Julien Chouinard-Beaupre, Antoine Mercier-Nicol"
__copyright__ = "Copyright 2018, DEVINE Project"
__credits__ = ["Simon Brodeur", "Francois Ferland", "Jean Rouat"]
__license__ = "BSD"
__version__ = "1.0.0"
__email__ = "devine.gegi-request@listes.usherbrooke.ca"
__status__ = "Production"

import rospy
from std_msgs.msg import Float64MultiArray
from devine_dialog.msg import TtsQuery
from devine_dialog import TTSAnswerType, send_speech
from guesswhat.models.guesser.guesser_wrapper import GuesserWrapper
from devine_config import topicname

TTS_ANSWER_TOPIC = topicname('tts_answer')
TTS_QUERY_TOPIC = topicname('tts_query')
CONFIDENCE_TOPIC = topicname('objects_confidence')


class GuesserROSWrapper(GuesserWrapper):
    """ Wraps the guesser model and publishes confidence levels """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.confidence = rospy.Publisher(CONFIDENCE_TOPIC, Float64MultiArray, queue_size=1, latch=True)

    def find_object(self, *args, **kwargs):
        """ find_object interface for the looper """
        found, softmax, selected_object = super().find_object(*args, **kwargs)
        found = [False]  # found is based on the oracle's choice, which we dont know

        self.confidence.publish(Float64MultiArray(data=softmax[0].tolist()))

        return found, softmax, selected_object


class OracleROSWrapper(object):
    """ Wraps the oracle model, publishes questions and waits for their answers """

    def __init__(self, tokenizer):
        self.tokenizer = tokenizer
        self.questions = rospy.Publisher(TTS_QUERY_TOPIC, TtsQuery, queue_size=1)
        self.previous_question = None
        self.previous_answer = None

    def initialize(self, sess):
        """ Initialize interface """
        pass

    def is_same_question(self, question):
        """ Checks if question is repeated """
        return self.previous_question is not None and question == self.previous_question

    def select_answer(self, question):
        """ Repeat answer or send speech """
        if self.is_same_question(question):
            rospy.logwarn('Skipped repeated question')
            return self.previous_answer

        answer = send_speech(self.questions, question, TTSAnswerType.YES_NO)
        self.previous_question = question
        self.previous_answer = answer
        return answer

    def answer_question(self, _sess, question, **kwargs):
        """ answer_question interface for the looper """
        if self.tokenizer.stop_dialogue in question[0]:
            return [self.tokenizer.non_applicable_token]

        text_question = self.tokenizer.decode(question[0]).replace('<padding>', '').strip()

        answer = self.select_answer(text_question)

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
