#! /usr/bin/env python2
""" Script to point with command line """

import argparse
import rospy

from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped
from devine_config import topicname
import devine_common.ros_utils as ros_utils


NODE_NAME = 'devine_irl_control_example_point'
TOPIC_OBJECT_LOCATION = topicname('guess_location_world')
TOPIC_HEAD_LOOK_AT = topicname('robot_look_at')
TOPIC_HEAD_JOINT_STATE = topicname('robot_head_joint_traj_point')
TIME = 2


def main(args):
    """ Publish on the 3D position from /base_link to point """
    # Parse arguments
    if args.point:
        point = [float(i) for i in args.point.split(',')]

    if args.look:
        look = [float(i) for i in args.look.split(',')]
    elif args.head_joint_position:
        head_joint_pos = [float(i) for i in args.head_joint_position.split(',')]

    if args.time:
        time = int(args.time)
    else:
        time = TIME

    rospy.init_node(NODE_NAME)
    rate = rospy.Rate(0.1)

    while not rospy.is_shutdown():
        if args.point:
            pub_arm = rospy.Publisher(TOPIC_OBJECT_LOCATION, PoseStamped, queue_size=1)
            pose_stamp_arm = ros_utils.pose_stamped(point[0], point[1], point[2])
            pub_arm.publish(pose_stamp_arm)

        if args.look:
            pub_head = rospy.Publisher(TOPIC_HEAD_LOOK_AT, PoseStamped, queue_size=1)
            pose_stamp_head = ros_utils.pose_stamped(look[0], look[1], look[2])
            pub_head.publish(pose_stamp_head)
        elif args.head_joint_position:
            pub = rospy.Publisher(TOPIC_HEAD_JOINT_STATE, JointTrajectoryPoint, queue_size=1)
            ros_packet = JointTrajectoryPoint()
            ros_packet.positions = head_joint_pos
            ros_packet.time_from_start = rospy.Duration(time)
            pub.publish(ros_packet)

        if not args.point and not args.look and not args.head_joint_position:
            rospy.logerr('Missing arguments')
            rospy.signal_shutdown('Missing arguments')
        else:
            rate.sleep()


def parser():
    """ Command Line Parser """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    arg_parser = argparse.ArgumentParser(formatter_class=arg_fmt, description=main.__doc__)
    required = arg_parser.add_argument_group('required arguments')
    required.add_argument('-p', '--point', required=False, help='What 3D position to point?')
    required.add_argument('-l', '--look', required=False, help='What 3D position to look?')
    required.add_argument('-hjp', '--head_joint_position',
                          required=False, help='What joint positions?')
    required.add_argument('-t', '--time', required=False, help='Time to accomplish trajectory?')
    arguments = arg_parser.parse_args(rospy.myargv()[1:])
    return arguments


if __name__ == '__main__':
    main(parser())
