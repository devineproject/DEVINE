#! /usr/bin/env python

import sys, argparse
import rospy

from irl_constant import ROBOT_NAME
from irl_constant import ROBOT_CONTROLLER

from trajectoryClient import TrajectoryClient
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
    positions = list(map(float, args.positions.replace('[', '').replace(']', '').split(',')))
    time = float(args.time)

    print('Init node...')
    nodeName = 'irl_control' + '_' + controller
    rospy.init_node(nodeName)
    print('Running node \'' + nodeName + '\'...')
    
    traj = TrajectoryClient(robot, controller)
    rospy.on_shutdown(traj.stop)
    
    gripperLeft = Gripper(ROBOT_NAME, 'left')

    traj.add_point(positions, time)
    traj.start()
    traj.wait(time)

    for i in range(3):
        gripperLeft.open(0.3)
        rospy.sleep(0.5)
        gripperLeft.open(0.1)
        rospy.sleep(0.5)
    
    print('Completed')

if __name__ == '__main__':
    main()
