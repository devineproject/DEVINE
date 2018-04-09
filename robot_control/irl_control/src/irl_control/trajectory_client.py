#! /usr/bin/env python

import rospy, actionlib

from control_msgs.msg import (FollowJointTrajectoryAction, FollowJointTrajectoryGoal)
from trajectory_msgs.msg import (JointTrajectoryPoint)

from irl_constant import ROBOT_CONTROLLER

# From https://github.com/RethinkRobotics/baxter_examples/blob/master/scripts/joint_trajectory_client.py?utf8=%E2%9C%93
class TrajectoryClient(object):
    def __init__(self, robot_name, controller_name):
        topic = '/'.join([robot_name, controller_name, 'follow_joint_trajectory'])
        self.controller_name = controller_name
        self._client = actionlib.SimpleActionClient(topic, FollowJointTrajectoryAction)
        self._goal = FollowJointTrajectoryGoal()
        self._goal_time_tolerance = rospy.Time(0.1)
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            raise RuntimeError('Timed out waiting for Action Serve', controller_name)
        self.clear()
        rospy.on_shutdown(self.stop)

    def add_point(self, positions, time):
        point = JointTrajectoryPoint(positions=positions, time_from_start=rospy.Duration(time))
        self._goal.trajectory.points.append(point)

    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time.now()
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()

    def clear(self):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        self._goal.trajectory.joint_names = ROBOT_CONTROLLER[self.controller_name]['joints_name']
