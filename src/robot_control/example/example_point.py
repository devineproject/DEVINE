#! /usr/bin/env python

''' Script to point with command line '''

import argparse
import rospy

from std_msgs.msg import Float32MultiArray
from devine_config import topicname

def main(args):
    ''' Publish on the 3D position from /base_link to point '''

    # Parse arguments
    point = [float(i) for i in args.point.split(',')]

    # Start ROS node
    node_name = 'devine_irl_control_example_point'
    rospy.init_node(node_name)
    rospy.loginfo('Running node \'' + node_name + '\' with object location at ' + str(point))

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        pub = rospy.Publisher(topicname('guess_location_world'), Float32MultiArray, queue_size=10)
        ros_packet = Float32MultiArray()
        ros_packet.data = point
        pub.publish(ros_packet)
        rospy.loginfo(point)
        rate.sleep()

def parser():
    ''' Command Line Parser'''

    arg_fmt = argparse.RawDescriptionHelpFormatter
    arg_parser = argparse.ArgumentParser(formatter_class=arg_fmt, description=main.__doc__)
    required = arg_parser.add_argument_group('required arguments')
    required.add_argument('-p', '--point', required=True, help='What 3D position to point?')
    arguments = arg_parser.parse_args(rospy.myargv()[1:])
    return arguments

if __name__ == '__main__':
    main(parser())
