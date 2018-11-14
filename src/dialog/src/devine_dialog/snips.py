#!/usr/bin/env python2
"""
Snips ROS integration

ROS Topics
tts_query -> query as TtsQuery input
tts_answer -> answer as TtsQuery output
"""

import json
import rospy
import paho.mqtt.client as mqtt
from devine_config import topicname
from devine_dialog.msg import TtsQuery
from devine_dialog import TTSAnswerType, send_speech

# Snips settings
SNIPS_HOST = 'localhost'
SNIPS_PORT = 1883
SNIPS_TOPICS = {
    'yes': 'Devine-UdeS:Yes',
    'no': 'Devine-UdeS:No',
    'na': 'Devine-UdeS:NA',
    'name': 'Devine-UdeS:GetName'
}
MQTT_CLIENT = mqtt.Client()

# Topics
TTS_QUERY = topicname('tts_query')
TTS_ANSWER = topicname('tts_answer')

# ROS
ROS_PUBLISHER = rospy.Publisher(TTS_ANSWER, TtsQuery, queue_size=10)


def snips_ask_callback(data):
    """ Callback executed when a question is received from ROS """
    rospy.loginfo('%s received: %s', rospy.get_name(), data.text)
    args = {'init': {'text': data.text, 'canBeEnqueued': True}, 'customData': {'uid' : data.uid, 'answerType' : data.answer.type}}

    # Switch to check what kind of data was received
    if data.answer_type == TTSAnswerType.NO_ANSWER.value:
        args['init']['type'] = 'notification'        
    elif data.answer_type == TTSAnswerType.YES_NO.value:
        args['init']['type'] = 'action'
        args['init']['intentFilter'] = [SNIPS_TOPICS['yes'], SNIPS_TOPICS['no'], SNIPS_TOPICS['na']]
    elif data.answer_type == TTSAnswerType.PLAYER_NAME.value:
        args['init']['type'] = 'action'
        args['init']['intentFilter'] = [SNIPS_TOPICS['name']]

    MQTT_CLIENT.publish('hermes/dialogueManager/startSession', json.dumps(args))


def on_snips_connect(*_):
    """ Callback executed when snips is connected
    Inputs: client, userdata, flags, connection_result
    """
    rospy.loginfo('Connected to snips at %s:%i', SNIPS_HOST, SNIPS_PORT)
    for topic in SNIPS_TOPICS.values():
        rospy.loginfo('Subscribe to topic: %s', topic)
        MQTT_CLIENT.subscribe('hermes/intent/' + topic)


# Args: client, userdata, msg
def on_snips_message(_client, _userdata, msg):
    """ Callback executed when snips receive an answer """
    # Parse the json response
    mqtt_topic = json.loads(msg.payload)
    intent_name = mqtt_topic['intent']['intentName']
    intent_probability = mqtt_topic['intent']['probability']

    rospy.loginfo('Detected intent %s with a probability of %f', intent_name, intent_probability)
    answer = None

    if intent_probability < 0.5:
        rospy.logwarn('Dropped intent because probability was too low. Asking player to repeat his answer.')
        answer = send_speech(ROS_PUBLISHER, "I am sorry, I didn't understand your answer, can you repeat please?", mqtt_topic['customData']['answerType'])
    else:
        # Get the raw text from the recognition
        answer = intent_name.split(':')[-1].lower()
        if intent_name == SNIPS_TOPICS['name']:
            slots = mqtt_topic['slots']
            if slots:
                answer = slots[0]['rawValue']
            else:
                answer = ''

    tts_answer = TtsQuery()
    tts_answer.uid = mqtt_topic['customData']['uid']
    tts_answer.text = answer

    ROS_PUBLISHER.publish(tts_answer)


def create_ros_listener():
    """ Create the ROS listeners """
    rospy.Subscriber(TTS_QUERY, TtsQuery, snips_ask_callback)


def on_snips_disconnect():
    """ Callback executed when snips is disconnected """
    rospy.loginfo('Disconnected from snips')
    MQTT_CLIENT.loop_start()


def setup_snips():
    """ Snips setup function """
    MQTT_CLIENT.on_connect = on_snips_connect
    MQTT_CLIENT.on_message = on_snips_message
    MQTT_CLIENT.on_disconnect = on_snips_disconnect
    MQTT_CLIENT.connect(SNIPS_HOST, SNIPS_PORT)


if __name__ == '__main__':
    rospy.init_node('snips')
    setup_snips()
    create_ros_listener()
    MQTT_CLIENT.loop_start()
    rospy.spin()
