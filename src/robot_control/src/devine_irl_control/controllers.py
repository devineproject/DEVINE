""" Joints controllers """

import rospy
import actionlib
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
    JointTrajectoryControllerState)
from trajectory_msgs.msg import JointTrajectoryPoint

from devine_irl_control.irl_constant import ROBOT_CONTROLLER


class CommandPublisher(object):
    """ Wrapper to send command to controller.
    Used by gripper.
    """

    def __init__(self, robot_name, controller_name):
        topic = '/'.join(['', robot_name, controller_name, 'command'])
        self._pub = rospy.Publisher(topic, Float64, queue_size=10)
        rospy.wait_for_message('/'.join([robot_name, 'joint_states']), JointState)

    def publish(self, position):
        """ Publish joint position """

        self._pub.publish(Float64(position))


class TrajectoryClient(object):
    """ Joint Trajectory Client using ROS ActionLib.
    Used by head and arms controllers.
    """
    # From: https://github.com/RethinkRobotics/baxter_examples/blob/master/scripts/joint_trajectory_client.py

    def __init__(self, robot_name, controller_name):
        self.controller_name = controller_name
        self.full_name = '/'.join(['', robot_name, controller_name])
        topic = self.full_name + '/follow_joint_trajectory'
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
        """ Adds points to trajectory """
        point = JointTrajectoryPoint(positions=positions, time_from_start=rospy.Duration(time))
        self._goal.trajectory.points.append(point)

    def start(self):
        """ Send goal to controller """
        self._goal.trajectory.header.stamp = rospy.Time.now()
        self._client.send_goal(self._goal)

    def stop(self):
        """ Stop movement towards goal """
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        """ Controller begins moving towards the goal.
        It blocks other trajectories to run simultaneously
        """
        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        """ Return if the Goal has succeeded """
        return self._client.get_result()

    def clear(self):
        """ Clear previous goal """
        self._goal = FollowJointTrajectoryGoal()
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        self._goal.trajectory.joint_names = ROBOT_CONTROLLER[self.controller_name]['joints_name']

    def get_position(self):
        """ Return the current position of the joints """
        joint_state = self.full_name + "/state"
        state = rospy.wait_for_message(joint_state, JointTrajectoryControllerState)
        return state.actual.positions  # one of [desired, actual or error]
