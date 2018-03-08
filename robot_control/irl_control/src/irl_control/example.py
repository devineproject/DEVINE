#! /usr/bin/env python

import argparse
import rospy

from trajectory_client import TrajectoryClient
from gripper import Gripper

def main():
    """Example to control head, arms and gripper with command line
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt, description=main.__doc__)
    required = parser.add_argument_group('required arguments')
    required.add_argument('-r', '--robot', required=True, choices=['jn0'], help='Which robot?')
    required.add_argument('-c', '--controller', required=True, choices=['head_controller', 'left_arm_controller', 'right_arm_controller'], help='Which controller?')
    required.add_argument('-p', '--positions', required=True, help='What goal positions?')
    required.add_argument('-t', '--time', required=True, help='How much seconds?')

    args = parser.parse_args(rospy.myargv()[1:])
    robot = args.robot
    controller = args.controller
    positions = [float(i) for i in args.positions.replace('[', '').replace(']', '').split(',')]
    time = float(args.time)

    print('Init node...')
    node_name = 'irl_control' + '_' + controller
    rospy.init_node(node_name)
    print('Running node \'' + node_name + '\'...')

    traj = TrajectoryClient(robot, controller)
    gripper_left = Gripper(robot, 'left')

    traj.add_point(positions, time)
    traj.start()
    traj.wait(time)

    for i in range(3):
        gripper_left.open(0.3)
        rospy.sleep(0.5)
        gripper_left.open(0.1)
        rospy.sleep(0.5)

    traj.clear()
    traj.add_point([0, 0, 0, 0], time)
    traj.start()
    traj.wait(time)

    print('Completed')

if __name__ == '__main__':
    main()
