#!/usr/bin/env python2
"""
Simple ROS node that subscribes to objects_confidence and object_guess_success
and publishes to robot_facial_expression to show facial emotions
"""

from enum import Enum
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float64MultiArray
from jn0_face_msgs.msg import EmoIntensity
from devine_config import topicname

class RobotExpression(Enum):
    """ Valid expressions """

    SURPRISE = "Surprise"
    SAD = "Sad"
    ANGER = "Anger"
    JOY = "Joy"

OBJECT_CONFIDENCE_TOPIC = topicname('objects_confidence')
GAME_SUCCESS_TOPIC = topicname('object_guess_success')
ROBOT_EXPRESSION_TOPIC = topicname('robot_facial_expression')

class FacialExpression():
    """ Subscribes to object confidence and publishes facial expression for a specific duration """

    EXPRESSION_DURATION = 10


    def __init__(self):
        self.initialize_for_new_game()
        self.robot_expression_publisher = rospy.Publisher(ROBOT_EXPRESSION_TOPIC,
                                                          EmoIntensity, queue_size=1)

        rospy.Subscriber(OBJECT_CONFIDENCE_TOPIC, Float64MultiArray,
                         self.on_new_object_confidence)

        rospy.Subscriber(GAME_SUCCESS_TOPIC, Bool,
                         self.on_game_success)

    def on_new_object_confidence(self, objects_confidence):
        """ callback on new object confidence. Updates max confidence """

        self.confidence = max(objects_confidence.data)


    def on_game_success(self, game_success):
        """ callback on end game. Shows emotion depending on success and confidence """

        expression = self.get_expression(self.confidence, game_success.data)

        if expression is not None and not self.showing_emotion:
            self.show_expression_for(expression, self.EXPRESSION_DURATION)

    def show_expression_for(self, expression, duration):
        """ Publishes the expression for a specific duration"""
        value = 1 # default value
        self.showing_emotion = True

        # Special case for anger which is a bit too intense
        if expression == RobotExpression.ANGER:
            value = 0.5

        face_expression = EmoIntensity(name=expression.value, value=value)
        self.robot_expression_publisher.publish(face_expression)

        d = rospy.Duration(duration, 0)
        rospy.sleep(d)

        face_expression = EmoIntensity(name=expression.value, value=0)
        self.robot_expression_publisher.publish(face_expression)
        self.initialize_for_new_game()

    def initialize_for_new_game(self):
        """ Init vars """
        self.showing_emotion = False
        self.confidence = None

    def get_expression(self, confidence, success):
        """
        returns the wanted expression depending on the received confidence.
        Defaults to ANGER
        """
        expression = None

        if self.confidence is not None:
            expression = RobotExpression.ANGER

            # Devine Won
            if success:
                if 0 <= confidence < .6:
                    expression = RobotExpression.SURPRISE
                else:
                    expression = RobotExpression.JOY
            # Devine Lost
            else:
                if 0 <= confidence < .6:
                    expression = RobotExpression.SAD
                else:
                    expression = RobotExpression.ANGER

        return expression

if __name__ == '__main__':
    rospy.init_node('facial_expression')
    FacialExpression()
    rospy.spin()
