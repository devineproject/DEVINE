'''ROS wrappers for guesswhat models'''

from queue import Queue, Empty

import rospy
from std_msgs.msg import String, Float64MultiArray

from guesswhat.models.guesser.guesser_wrapper import GuesserWrapper

ANSWER_TOPIC = '/answer'
QUESTION_TOPIC = '/question'
CONFIDENCE_TOPIC = '/confidence'
SELECTION_TOPIC = '/object_found'

class GuesserROSWrapper(GuesserWrapper):
    '''Wraps the guesser model and publishes confidence levels'''
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.confidence = rospy.Publisher(CONFIDENCE_TOPIC, Float64MultiArray, queue_size=1)

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
        self.answers = Queue(1)
        self.questions = rospy.Publisher(QUESTION_TOPIC, String, queue_size=1)
        rospy.Subscriber(ANSWER_TOPIC, String, self.answer_callback)

    def initialize(self, sess):
        '''initialize interface'''
        pass

    def answer_question(self, sess, question, **kwargs):
        '''answer_question interface for the looper'''
        if self.tokenizer.stop_dialogue in question[0]:
            return [self.tokenizer.non_applicable_token]

        text_question = self.tokenizer.decode(question[0]).replace('<padding>', '').strip()

        self.questions.publish(text_question)

        while not rospy.is_shutdown():
            try:
                answer = self.answers.get(timeout=1)

                if answer == 'yes':
                    token = self.tokenizer.yes_token
                    break
                elif answer == 'no':
                    token = self.tokenizer.no_token
                    break
                elif answer == 'na':
                    token = self.tokenizer.non_applicable_token
                    break
                else:
                    rospy.logerr('Garbage on {}, expects yes|no|na'.format(ANSWER_TOPIC))
            except Empty:
                pass
        else:
            exit()

        return [token]

    def answer_callback(self, data):
        '''Callback for the answer topic'''
        self.answers.put(data.data)
