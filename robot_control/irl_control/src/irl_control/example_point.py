#! /usr/bin/env python

import argparse
import rospy

from std_msgs.msg import Float32MultiArray

def main(args):
    """Example to control head, arms and gripper with command line
    """
    # Parse arguments
    point = [float(i) for i in args.point.split(',')]

    # Start ROS node
    node_name = 'irl_control_example'
    rospy.init_node(node_name)
    rospy.loginfo('Running node \'' + node_name + '\' with object location at ' + str(point))

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        pub = rospy.Publisher('/object_location', Float32MultiArray, queue_size=10)
        ros_packet = Float32MultiArray()
        ros_packet.data = point
        pub.publish(ros_packet)
        rospy.loginfo(point)
        rate.sleep()

if __name__ == '__main__':
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt, description=main.__doc__)
    required = parser.add_argument_group('required arguments')
    required.add_argument('-p', '--point', required=True, help='What 3D position to point?')
    args = parser.parse_args(rospy.myargv()[1:])

    main(args)
