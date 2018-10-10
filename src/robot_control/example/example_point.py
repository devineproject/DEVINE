#! /usr/bin/env python2

''' Script to point with command line '''

import argparse
import rospy

from std_msgs.msg import Float32MultiArray
from trajectory_msgs.msg import JointTrajectoryPoint
from devine_config import topicname


NODE_NAME = 'devine_irl_control_example_point'
TOPIC_OBJECT_LOCATION = topicname('guess_location_world')
TOPIC_HEAD_JOINT_STATE = topicname('robot_head_joint_traj_point')
TIME = 2

def main(args):
    ''' Publish on the 3D position from /base_link to point '''
    # Parse arguments
    if args.point:
        point = [float(i) for i in args.point.split(',')]
        rospy.loginfo('Running \'' + NODE_NAME + '\' with object location at ' + str(point))
    elif args.head_joint_position:
        head_joint_pos = [float(i) for i in args.head_joint_position.split(',')]
        rospy.loginfo('Running \'' + NODE_NAME + '\' with head_joint at ' + str(head_joint_pos))
    if args.time:
        time = int(args.time)
    else:
        time = TIME

    rospy.init_node(NODE_NAME)
    rate = rospy.Rate(0.1)
    while not rospy.is_shutdown():
        if args.point:
            pub = rospy.Publisher(TOPIC_OBJECT_LOCATION, Float32MultiArray, queue_size=1)
            ros_packet = Float32MultiArray()
            ros_packet.data = point
        elif args.head_joint_position:
            pub = rospy.Publisher(TOPIC_HEAD_JOINT_STATE, JointTrajectoryPoint, queue_size=1)
            ros_packet = JointTrajectoryPoint()
            ros_packet.positions = head_joint_pos
            ros_packet.time_from_start = rospy.Duration(time)
        else:
            rospy.signal_shutdown('Missing arguments')
        rospy.loginfo(ros_packet)
        pub.publish(ros_packet)
        rate.sleep()

def parser():
    ''' Command Line Parser'''
    arg_fmt = argparse.RawDescriptionHelpFormatter
    arg_parser = argparse.ArgumentParser(formatter_class=arg_fmt, description=main.__doc__)
    required = arg_parser.add_argument_group('required arguments')
    required.add_argument('-p', '--point', required=False, help='What 3D position to point?')
    required.add_argument('-hjp', '--head_joint_position',
                          required=False, help='What joint positions?')
    required.add_argument('-t', '--time', required=False, help='Time to accomplish trajectory?')
    arguments = arg_parser.parse_args(rospy.myargv()[1:])
    return arguments

if __name__ == '__main__':
    main(parser())
