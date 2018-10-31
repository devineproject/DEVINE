#! /usr/bin/env python2
""" Example to emote robot emotion with command line """

import argparse
import rospy

from std_msgs.msg import Bool
from std_msgs.msg import Float64MultiArray

from devine_config import topicname

TOPIC_GUESSWHAT_CONFIDENCE = topicname('objects_confidence')
TOPIC_GUESSWHAT_SUCCEED = topicname('object_guess_success')


def main(arguments):
    """ Publish on GuessWhat?! confidence and success topic """

    # Parse arguments
    confidence_array = [float(i) for i in arguments.confidence.split(',')]
    is_guesswhat_succeed = bool(int(arguments.succeed))

    # Start ROS node
    node_name = 'devine_irl_control_example_emotion'
    rospy.init_node(node_name)
    rospy.loginfo('Running node \'%s\' with\n\tconfidence: %d\n\tsucceed: %d',
                  node_name, confidence_array, is_guesswhat_succeed)

    while not rospy.is_shutdown():
        pub_confidence = rospy.Publisher(TOPIC_GUESSWHAT_CONFIDENCE,
                                         Float64MultiArray,
                                         queue_size=10)
        ros_packet_float = Float64MultiArray()
        ros_packet_float.data = confidence_array
        pub_confidence.publish(ros_packet_float)

        rospy.sleep(1)

        pub_is_guesswhat_succeed = rospy.Publisher(TOPIC_GUESSWHAT_SUCCEED, Bool, queue_size=10)
        ros_packet_bool = Bool()
        ros_packet_bool.data = is_guesswhat_succeed
        pub_is_guesswhat_succeed.publish(ros_packet_bool)

        rospy.sleep(10)
        rospy.loginfo('Game should be completed.')


def parser():
    """ Command Line Parser"""
    arg_fmt = argparse.RawDescriptionHelpFormatter
    arg_parser = argparse.ArgumentParser(formatter_class=arg_fmt, description=main.__doc__)
    required = arg_parser.add_argument_group('required arguments')
    required.add_argument('-c', '--confidence', required=True, help='GuessWhat?! confidence')
    required.add_argument('-s', '--succeed', required=True, help='Did GuessWhat?! succeed')
    arguments = arg_parser.parse_args(rospy.myargv()[1:])
    return arguments


if __name__ == '__main__':
    main(parser())
