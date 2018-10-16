#!/usr/bin/env python2
''' 
Simple ROS node that subscribes to objects_confidence
and publishes to robot_facial_expression 
'''

from enum import Enum
import rospy
from jn0_face_msgs.msg import EmoIntensity
from std_msgs.msg import Float64MultiArray
from devine_config import topicname

class RobotExpression(Enum):
    ''' Valid expressions '''
    
    SURPRISE = "Surprise"
    ANGER = "Anger"
    JOY = "Joy"

OBJECT_CONFIDENCE_TOPIC = topicname('objects_confidence')
ROBOT_EXPRESSION_TOPIC = topicname('robot_facial_expression')

class FacialExpression():
    ''' Subscribes to object confidence and publishes facial expression '''

    def __init__(self):
        self.robot_expression_publisher = rospy.Publisher(ROBOT_EXPRESSION_TOPIC,
                                                          EmoIntensity, queue_size=10)

        rospy.Subscriber(OBJECT_CONFIDENCE_TOPIC, Float64MultiArray,
                         self.on_new_object_confidence)

    def on_new_object_confidence(self, objects_confidence):
        ''' callback on new object confidence. Publishes a facial expression '''

        max_confidence = max(objects_confidence.data)
        expression = self.get_expression(max_confidence)

        face_expression = EmoIntensity(name=expression, value=1)

        self.robot_expression_publisher.publish(face_expression)

    def get_expression(self, confidence):
        ''' 
        returns the wanted expression depending on the received confidence.
        Default value is ANGER 
        '''

        expression = RobotExpression.ANGER

        if 0 <= confidence < .6:
            expression = RobotExpression.ANGER
        elif .6 <= confidence < .8:
            expression = RobotExpression.SUPRISE
        elif confidence >= .8:
            expression = RobotExpression.JOY

        return expression.value

if __name__ == '__main__':
    rospy.init_node('facial_expression')
    FacialExpression()
    rospy.spin()
