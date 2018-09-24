#! /usr/bin/env python

''' Example to emote robot emotion with command line '''

import argparse
import rospy

from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray

TOPIC_GUESSWHAT_CONFIDENCE = '/confidence'
TOPIC_GUESSWHAT_SUCCEED = '/is_guesswhat_succeed'

def main(arguments):
    ''' Publish on GuessWhat?! confidence and success topic '''

    # Parse arguments
    confidence_array = [float(i) for i in arguments.confidence.split(',')]
    is_guesswhat_succeed = bool(int(arguments.succeed))

    # Start ROS node
    node_name = 'irl_control_emotion_example'
    rospy.init_node(node_name)
    rospy.loginfo('Running node \'' + node_name + '\'with\n\tconfidence: ' +
                  str(confidence_array) + '\n\tsucceed: ' + str(is_guesswhat_succeed))

    while not rospy.is_shutdown():
        pub_confidence = rospy.Publisher(TOPIC_GUESSWHAT_CONFIDENCE,
                                         Float32MultiArray,
                                         queue_size=10)
        ros_packet_float = Float32MultiArray()
        ros_packet_float.data = confidence_array
        pub_confidence.publish(ros_packet_float)

        rospy.sleep(1)

        pub_is_guesswhat_succeed = rospy.Publisher(TOPIC_GUESSWHAT_SUCCEED, Bool, queue_size=10)
        ros_packet_bool = Bool()
        ros_packet_bool.data = is_guesswhat_succeed
        pub_is_guesswhat_succeed.publish(ros_packet_bool)

        rospy.sleep(10)
        rospy.loginfo('SHOULD BE DONE')

def parser():
    ''' Command Line Parser'''

    arg_fmt = argparse.RawDescriptionHelpFormatter
    arg_parser = argparse.ArgumentParser(formatter_class=arg_fmt, description=main.__doc__)
    required = arg_parser.add_argument_group('required arguments')
    required.add_argument('-c', '--confidence', required=True, help='GuessWhat?! confidence')
    required.add_argument('-s', '--succeed', required=True, help='Did GuessWhat?! succeed')
    arguments = arg_parser.parse_args(rospy.myargv()[1:])
    return arguments

if __name__ == '__main__':
    main(parser())
