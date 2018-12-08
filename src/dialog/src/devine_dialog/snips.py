#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Snips ROS integration

ROS Topics
tts_query -> query as TtsQuery input
tts_answer -> answer as TtsAnswer output
"""
__author__ = "Jordan Prince Tremblay, Ismael Balafrej, Felix Labelle, Felix Martel-Denis, Eric Matte, Adam Letourneau, Julien Chouinard-Beaupre, Antoine Mercier-Nicol"
__copyright__ = "Copyright 2018, DEVINE Project"
__credits__ = ["Simon Brodeur", "Francois Ferland", "Jean Rouat"]
__license__ = "BSD"
__version__ = "1.0.0"
__email__ = "devine.gegi-request@listes.usherbrooke.ca"
__status__ = "Production"

import json
import yaml
import rospy
import paho.mqtt.client as mqtt
from devine_config import topicname
from devine_dialog.msg import TtsQuery, TtsAnswer
from devine_dialog import TTSAnswerType
from threading import RLock

# Snips settings
SNIPS_HOST = 'localhost'
SNIPS_PORT = 1883
SNIPS_INTENT = {
    'yes': 'Devine-UdeS:Yes',
    'no': 'Devine-UdeS:No',
    'na': 'Devine-UdeS:NA',
    'name': 'Devine-UdeS:GetName'
}
MQTT_CLIENT = mqtt.Client()

# Topics
TTS_QUERY = topicname('tts_query')
TTS_ANSWER = topicname('tts_answer')


class SnipsRosWrapper(mqtt.Client):
    def __init__(self):
        self._publisher = rospy.Publisher(TTS_ANSWER, TtsAnswer, queue_size=10)
        rospy.Subscriber(TTS_QUERY, TtsQuery, self.on_query)
        rospy.Subscriber(TTS_ANSWER, TtsAnswer, self.on_answer)
        super(SnipsRosWrapper, self).__init__()
        self.connect(SNIPS_HOST, SNIPS_PORT)
        self.queries = {}
        self.query_mutex = RLock()

    def on_connect(self, _client, _userdata, _flags, _rc):
        """ Callback executed when snips is connected """
        rospy.loginfo('Connected to snips at %s:%i', SNIPS_HOST, SNIPS_PORT)

        for intent in SNIPS_INTENT.values():
            rospy.loginfo('Subscribed to intent: %s', intent)
            self.subscribe('hermes/intent/' + intent)

        self.subscribe('hermes/dialogueManager/sessionStarted')
        self.subscribe('hermes/dialogueManager/sessionEnded')

    def on_message(self, _client, _userdata, msg):
        """ Callback executed when an mqtt message is received """
        topic = msg.topic
        payload = json.loads(msg.payload)
        session_id = payload['sessionId']
        original_query = yaml.load(payload['customData'])

        if 'sessionStarted' in topic:
            self.query_mutex.acquire()
            self.queries[session_id] = original_query['uid']
            self.query_mutex.release()
        elif 'sessionEnded' in topic:
            if payload['termination']['reason'] == 'intentNotRecognized':
                # Snips repeated 3 times the question, and the answer wasn't understood
                tts_answer = TtsAnswer()
                tts_answer.original_query = TtsQuery(*original_query.values())
                tts_answer.probability = 0
                tts_answer.text = ''
                self._publisher.publish(tts_answer)
            else:
                self.query_mutex.acquire()
                del self.queries[session_id]  # Session ended correctly
                self.query_mutex.release()
        else:
            intent_name = payload['intent']['intentName']
            intent_probability = payload['intent']['probability']
            rospy.loginfo('Detected intent %s with a probability of %f',
                          intent_name, intent_probability)
            answer = intent_name.split(':')[-1].lower()
            if intent_name == SNIPS_INTENT['name']:
                slots = payload['slots']
                if slots:
                    answer = slots[0]['rawValue']
                else:
                    answer = ''
            tts_answer = TtsAnswer()
            tts_answer.original_query = TtsQuery(*original_query.values())
            tts_answer.probability = intent_probability
            tts_answer.text = answer
            self._publisher.publish(tts_answer)
            self.publish('hermes/dialogueManager/endSession', json.dumps({
                'sessionId': session_id
            }))

    def on_disconnect(self, _client, _userdata, _rc):
        rospy.logwarn('Disconnected from snips')
        self.loop_start()  # Try to connect again

    def on_query(self, data):
        """ Callback when receiving a query from ros """
        rospy.loginfo('%s received: %s', rospy.get_name(), data.text)
        args = {'init': {'text': data.text, 'canBeEnqueued': True},
                'customData': str(data)}

        # Switch to check what kind of data was received
        if data.answer_type == TTSAnswerType.NO_ANSWER.value:
            args['init']['type'] = 'notification'
        elif data.answer_type == TTSAnswerType.YES_NO.value:
            args['init']['type'] = 'action'
            args['init']['intentFilter'] = [SNIPS_INTENT['yes'],
                                            SNIPS_INTENT['no'], SNIPS_INTENT['na']]
        elif data.answer_type == TTSAnswerType.PLAYER_NAME.value:
            args['init']['type'] = 'action'
            args['init']['intentFilter'] = [SNIPS_INTENT['name']]

        self.publish('hermes/dialogueManager/startSession', json.dumps(args))

    def on_answer(self, data):
        """ If somebody else took care of the answer, delete it from the snips buffer """
        self.query_mutex.acquire()
        for i in self.queries:
            if self.queries[i] == data.original_query.uid:
                self.publish('hermes/dialogueManager/endSession', json.dumps({
                    'sessionId': i
                }))
        self.query_mutex.release()


def main():
    """ Starting point of this file """
    rospy.init_node('snips')
    wrapper = SnipsRosWrapper()
    wrapper.loop_start()
    rospy.spin()


if __name__ == '__main__':
    main()
