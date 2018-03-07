#! /usr/bin/env python

import sys, argparse
import rospy, actionlib

from control_msgs.msg import (FollowJointTrajectoryAction, FollowJointTrajectoryGoal)
from trajectory_msgs.msg import (JointTrajectoryPoint)

from irl_constant import ROBOT_NAME
from irl_constant import ROBOT_CONTROLLER

# From https://github.com/RethinkRobotics/baxter_examples/blob/master/scripts/joint_trajectory_client.py?utf8=%E2%9C%93
class TrajectoryClient(object):
    def __init__(self, robotName, controllerName):
        topic = '/'.join([robotName, controllerName, 'follow_joint_trajectory'])
        self._client = actionlib.SimpleActionClient(topic, FollowJointTrajectoryAction)
        self._goal = FollowJointTrajectoryGoal()
        self._goal_time_tolerance = rospy.Time(0.1)
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        server_up = self._client.wait_for_server(timeout=rospy.Duration(20.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self.clear(controllerName)

    def add_point(self, positions, time):
        point = JointTrajectoryPoint(positions=positions, time_from_start= rospy.Duration(time))
        self._goal.trajectory.points.append(point)

    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time.now()
        #print(self._goal)
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()

    def clear(self, controllerName):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        self._goal.trajectory.joint_names = ROBOT_CONTROLLER[controllerName]['joints_name']
