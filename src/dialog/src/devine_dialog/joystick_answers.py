#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Simple joystick controller to answer snips call when demoing in a noisy environment
"""

import rospy
from devine_config import topicname
from devine_dialog.msg import TtsQuery, TtsAnswer
from sensor_msgs.msg import Joy

JOYSTICK_TOPIC = topicname('joystick')
TTS_QUERY_TOPIC = topicname('tts_query')
TTS_ANSWER_TOPIC = topicname('tts_answer')

# See http://wiki.ros.org/joy
LOGITECH_JOYSTICK_BTN_MAP = {
    'X': 0,
    'A': 1,
    'B': 2,
    'Y': 3,
    'LB': 4,
    'RB': 5,
    'LT': 6,
    'RT': 7,
    'back': 8,
    'start': 9,
    'LStick': 10,
    'RStick': 11,
}

class JoystickTTSAnswerer(object):
    def __init__(self):
        rospy.init_node('JoystickTTSAnswerer')
        self._publisher = rospy.Publisher(TTS_ANSWER_TOPIC, TtsAnswer, queue_size=1)
        rospy.Subscriber(TTS_ANSWER_TOPIC, TtsAnswer, self._on_tts_answer)
        rospy.Subscriber(TTS_QUERY_TOPIC, TtsQuery, self._on_tts_query)
        rospy.Subscriber(JOYSTICK_TOPIC, Joy, self._on_joystick_msg)
        self.current_query = None

    def _on_tts_answer(self, _):
        """ Callback when receiving answers from TTS """
        self.current_query = None

    def _on_tts_query(self, data):
        """ Callback when receiving answers from TTS """
        self.current_query = data.uid

    def _on_joystick_msg(self, data):
        if self.current_query is not None:
            answer = None
            if data.buttons[LOGITECH_JOYSTICK_BTN_MAP['A']] == 1:
                answer = 'yes'
            elif data.buttons[LOGITECH_JOYSTICK_BTN_MAP['B']] == 1:
                answer = 'no'
            elif data.buttons[LOGITECH_JOYSTICK_BTN_MAP['Y']] == 1:
                answer = 'na'

            if answer is not None:
                msg = TtsAnswer()
                msg.original_query.uid = self.current_query
                msg.text = answer
                msg.probability = 1
                self._publisher.publish(msg)

def main():
    """ Starting point of this file """
    JoystickTTSAnswerer()
    rospy.spin()


if __name__ == '__main__':
    main()
