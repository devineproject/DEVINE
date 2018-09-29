#!/usr/bin/env python2
'''
Snips ROS integration

ROS Topics
question -> question as string input
answer -> answer as string output
'''
import json
import paho.mqtt.client as mqtt
import rospy
from std_msgs.msg import String
from devine_config import topicname

# Snips settings
SNIPS_HOST = "localhost"
SNIPS_PORT = 1883
SNIPS_TOPICS = ['hermes/intent/#'] # Wild card for every intents
MQTT_CLIENT = mqtt.Client()

#Topics
SNIPS_ANSWER = topicname('answer')
SNIPS_QUESTION = topicname('question')

# ROS
ROS_PUBLISHER = rospy.Publisher(SNIPS_ANSWER, String, queue_size=10)

def snips_ask_callback(data):
    '''
    Callback executed when a question is received from ROS
    '''
    question = data.data
    rospy.loginfo("%s received: %s", rospy.get_name(), question)
    args = {'init': {'type': 'action', 'text': question, 'canBeEnqueued': True}} # TODO: Add an intentFilter
    MQTT_CLIENT.publish(
        'hermes/dialogueManager/startSession', json.dumps(args))


# Args: client, userdata, flags, connection_result
def on_snips_connect(*_):
    '''
    Callback executed when snips is connected
    '''
    rospy.loginfo("Connected to snips at %s:%i", SNIPS_HOST, SNIPS_PORT)
    for topic in SNIPS_TOPICS:
        MQTT_CLIENT.subscribe(topic)


# Args: client, userdata, msg
def on_snips_message(_client, _userdata, msg):
    '''
    Callback executed when snips receive an answer
    '''
    rospy.loginfo("Received intent from snips: %s", msg.payload)
    data = json.loads(msg.payload)
    if data['slots']:
        rospy.loginfo("Received message %s, detected: %s", data['input'],
                      data['slots'][0]['value']['value'].lower())
        ROS_PUBLISHER.publish(data['slots'][0]['value']['value'].lower())


def create_ros_listener():
    '''
    Create the ROS listeners
    '''
    rospy.Subscriber(SNIPS_QUESTION, String, snips_ask_callback)


def on_snips_disconnect():
    '''
    Callback executed when snips is disconnected
    '''
    rospy.loginfo("Disconnected from snips")


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
