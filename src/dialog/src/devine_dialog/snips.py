#!/usr/bin/env python2
'''
Snips ROS integration

ROS Topics
tts_query -> query as TtsQuery input
tts_answer -> answer as TtsQuery output
'''
import json
import paho.mqtt.client as mqtt
import rospy
from devine_config import topicname
from devine_dialog.msg import TtsQuery
from enum import Enum

# Snips settings
SNIPS_HOST = "localhost"
SNIPS_PORT = 1883
SNIPS_TOPICS = {
    'yes': 'Picnic8:yes',
    'no': 'Picnic8:no',
    'na': 'Picnic8:na'
}
MQTT_CLIENT = mqtt.Client()

#Topics
TTS_QUERY = topicname('tts_query')
TTS_ANSWER = topicname('tts_answer')

# ROS
ROS_PUBLISHER = rospy.Publisher(TTS_ANSWER, TtsQuery, queue_size=10)

class TTSAnswerType(Enum):
    NO_ANSWER = 0
    YES_NO = 1
    PLAYER_NAME = 2

def snips_ask_callback(data):
    '''
    Callback executed when a question is received from ROS
    '''
    rospy.loginfo("%s received: %s", rospy.get_name(), data.text)
    args = {'init': {'text': data.text, 'canBeEnqueued': True}, 'customData': str(data.uid)}
    if data.answer_type == TTSAnswerType.NO_ANSWER.value:
        args['init']['type'] = 'notification'
    elif data.answer_type == TTSAnswerType.YES_NO.value:
        args['init']['type'] = 'action'
        args['init']['intentFilter'] = [SNIPS_TOPICS['yes'], SNIPS_TOPICS['no'], SNIPS_TOPICS['na']]
    elif data.answer_type == TTSAnswerType.PLAYER_NAME.value:
        raise NotImplementedError()
    
    MQTT_CLIENT.publish('hermes/dialogueManager/startSession', json.dumps(args))


# Args: client, userdata, flags, connection_result
def on_snips_connect(*_):
    '''
    Callback executed when snips is connected
    '''
    rospy.loginfo("Connected to snips at %s:%i", SNIPS_HOST, SNIPS_PORT)
    for topic in SNIPS_TOPICS.values():
        rospy.loginfo("Subscribe to topic: %s", topic)
        MQTT_CLIENT.subscribe('hermes/intent/' + topic)


# Args: client, userdata, msg
def on_snips_message(_client, _userdata, msg):
    '''
    Callback executed when snips receive an answer
    '''
    # Parse the json response
    mqtt_topic = json.loads(msg.payload)
    intent_name = mqtt_topic['intent']['intentName']
    intent_probability = mqtt_topic['intent']['probability']

    rospy.loginfo("Detected intent %s with a probability of %f", intent_name, intent_probability)

    if intent_probability < 0.5:
        rospy.logwarn("Dropped intent, probability was too low")
        return

    tts_answer = TtsQuery()
    tts_answer.uid = int(mqtt_topic['customData'])
    tts_answer.text = intent_name.split(":")[-1].lower()

    ROS_PUBLISHER.publish(tts_answer)


def create_ros_listener():
    '''
    Create the ROS listeners
    '''
    rospy.Subscriber(TTS_QUERY, TtsQuery, snips_ask_callback)


def on_snips_disconnect():
    '''
    Callback executed when snips is disconnected
    '''
    rospy.loginfo("Disconnected from snips")
    MQTT_CLIENT.loop_start()


def setup_snips():
    '''
    Snips setup function
    '''
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
