#!/usr/bin/env python2
# -*- coding: utf-8 -*-
""" Node to control robot joints """
__author__ = "Jordan Prince Tremblay, Ismael Balafrej, Felix Labelle, Felix Martel-Denis, Eric Matte, Adam Letourneau, Julien Chouinard-Beaupre, Antoine Mercier-Nicol"
__copyright__ = "Copyright 2018, DEVINE Project"
__credits__ = ["Simon Brodeur", "Francois Ferland", "Jean Rouat"]
__license__ = "BSD"
__version__ = "1.0.0"
__email__ = "devine.gegi-request@listes.usherbrooke.ca"
__status__ = "Production"

import rospy
import tf

from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectoryPoint

from devine_config import topicname
from devine_irl_control import irl_constant
from devine_irl_control.movement import Movement
from devine_irl_control.controllers import TrajectoryClient
from devine_irl_control.gripper import Gripper
from devine_irl_control import ik
from devine_irl_control.admittance import Admittance

ROBOT_NAME = irl_constant.ROBOT_NAME
LOW_ADMITTANCE = 3
HIGH_ADMITTANCE = 15

# IN
TOPIC_OBJECT_LOCATION = topicname('guess_location_world')
TOPIC_HEAD_LOOK_AT = topicname('robot_look_at')
TOPIC_HEAD_JOINT_STATE = topicname('robot_head_joint_traj_point')
GUESS_SUCCESS = topicname('object_guess_success')

# OUT
TOPIC_IS_POINTING = topicname('is_pointing_object')
TOPIC_IS_LOOKING = topicname('is_looking')

class Controller(object):
    """ Arms, head and gripper controller """

    def __init__(self, is_head_activated=True, is_arms_activated=True, is_gripper_activated=True):
        self.arm_data = None
        self.head_data = None
        self.time = 10 #TODO: Calc speed
        self.admittance_service = None
        self.tf_listener = tf.TransformListener()
        self.is_arms_activated = is_arms_activated
        self.is_sim = rospy.get_param('~is_sim')
        rospy.loginfo('Waiting for controllers')

        if not self.is_sim:
            self.admittance_service = Admittance()
            self.admittance_service.set_admittance(
                'L', [LOW_ADMITTANCE, LOW_ADMITTANCE, LOW_ADMITTANCE, LOW_ADMITTANCE])

        if is_gripper_activated:
            self.gripper_right = Gripper(ROBOT_NAME, 'right')
            self.gripper_left = Gripper(ROBOT_NAME, 'left')
            rospy.loginfo('Gripper controllers are ready')

        try:
            if is_head_activated:
                self.head = TrajectoryClient(ROBOT_NAME, 'neck_controller')
                rospy.loginfo('Head controller are ready')
            if is_arms_activated:
                self.arm_right = TrajectoryClient(ROBOT_NAME, 'right_arm_controller')
                self.arm_left = TrajectoryClient(ROBOT_NAME, 'left_arm_controller')
                rospy.loginfo('Arms controllers are ready')
        except RuntimeError as err:
            rospy.logerr(err)
            rospy.signal_shutdown(err)

        rospy.Subscriber(TOPIC_OBJECT_LOCATION, PoseStamped, self.arm_pose_callback)
        rospy.Subscriber(GUESS_SUCCESS, Bool, self.on_guess_success_callback)

        self.pub_is_pointing = rospy.Publisher(TOPIC_IS_POINTING,
                                               Bool, queue_size=1)
        if is_head_activated:
            rospy.Subscriber(TOPIC_HEAD_LOOK_AT, PoseStamped, self.head_pose_callback)
            rospy.Subscriber(TOPIC_HEAD_JOINT_STATE,
                             JointTrajectoryPoint,
                             self.head_joint_traj_point_callback)
            self.pub_is_looking = rospy.Publisher(TOPIC_IS_LOOKING,
                                                  Bool, queue_size=1)
    
    def on_guess_success_callback(self, _msg):
        """ Callback when the robot knows if he points the right or wrong object """
        self.move_init(10)
        if not self.is_sim:
            self.admittance_service.set_admittance(
                'L', [LOW_ADMITTANCE, LOW_ADMITTANCE, LOW_ADMITTANCE, LOW_ADMITTANCE])

    def head_joint_traj_point_callback(self, msg):
        """ On topic /head_joint_traj_point, move head """
        pos = msg.positions
        time = msg.time_from_start
        self.move({'head': pos}, time.to_sec())

    def arm_pose_callback(self, msg):
        """ On topic /object_location, compute and move joints """
        if self.arm_data != msg:
            self.arm_data = msg
            self.arm_data.header.stamp = rospy.Time.now() - rospy.rostime.Duration(0.1) # TODO: what if head moves ? FIXME
            if self.is_arms_activated:
                if msg.pose.position != (0, 0, 0):
                    # TODO: add decision left/right arms in ik.py
                    arm_decision = 'left'
                    joints_position = self.calcul_arm(arm_decision)
                    self.move({'arm_' + arm_decision: joints_position},
                              self.time)
                else:
                    self.move_init(10)

        self.pub_is_pointing.publish(True)

    def head_pose_callback(self, msg):
        """ On topic /look_at, compute and move joints """
        if self.head_data != msg:
            self.head_data = msg
            if msg.pose.position != (0, 0, 0):
                joints_position = self.calcul_head()
                self.move({'head': joints_position},
                          self.time)
            else:
                self.move_init(10)

        self.pub_is_looking.publish(True)

    def calcul_arm(self, controller):
        """ Get arm translation from TF and apply inverse kinematic """
        arm_joints_position = None

        topic_robot_shoulder_frame = irl_constant.ROBOT_LINK['r_shoulder_fixed']
        if controller == 'left':
            topic_robot_shoulder_frame = irl_constant.ROBOT_LINK['l_shoulder_fixed']

        try:
            tf_pose_stamp = self.tf_listener.transformPose(topic_robot_shoulder_frame,
                                                           self.arm_data)
            tf_position = tf_pose_stamp.pose.position

            arm_joints_position = ik.arms_pan_tilt(controller,
                                                   tf_position.x,
                                                   tf_position.y,
                                                   tf_position.z)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as err:
            rospy.logerr(err)

        return arm_joints_position

    def calcul_head(self):
        """ Get head translation from TF and apply inverse kinematic """
        head_joints_position = None

        try:
            tf_pose_stamp = self.tf_listener.transformPose(irl_constant.ROBOT_LINK['l_eye'],
                                                           self.head_data)
            tf_position = tf_pose_stamp.pose.position

            head_joints_position = ik.head_pan_tilt(tf_position.x,
                                                    tf_position.y,
                                                    tf_position.z)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as err:
            rospy.logerr(err)

        return head_joints_position

    def move_init(self, time):
        """ Move joints to initial position """
        self.move({
            'head': [0, 0],
            'arm_left':  [0, 0, 0, 0],
            'arm_right':  [0, 0, 0, 0]
        }, time)

    def move(self, controller_joints_positions, time):
        """ Move joints """
        move_gripper = False

        times = get_joints_time(controller_joints_positions, time)
        if not self.is_sim:
            self.admittance_service.set_admittance(
                'L', [HIGH_ADMITTANCE, HIGH_ADMITTANCE, HIGH_ADMITTANCE, HIGH_ADMITTANCE])

        for key in controller_joints_positions:
            getattr(self, key).clear()
            getattr(self, key).add_point(controller_joints_positions[key], times[key])
            getattr(self, key).start()
        for key in controller_joints_positions:
            getattr(self, key).wait(times[key])

        if move_gripper:
            i = 0
            while i < 3:
                self.gripper_right.open(0.3)
                rospy.sleep(0.5)
                self.gripper_right.open(0.1)
                rospy.sleep(0.5)
                i = i + 1


def get_joints_time(controller_joints, time):
    """ Converte a single time to an array of time """
    if isinstance(time, (int, float)):
        times = {}
        for key in controller_joints:
            times[key] = time
    else:
        times = time
    return times


def main():
    """ Node initialize controllers """
    node_name = 'devine_irl_control'
    rospy.init_node(node_name)
    rospy.loginfo('Running node \'%s\'', node_name)

    is_head_activated = rospy.get_param('~is_head_activated')
    is_arms_activated = rospy.get_param('~is_arms_activated')
    is_grippers_activated = rospy.get_param('~is_grippers_activated')

    is_sim = rospy.get_param('~is_sim')
    if is_sim:
        # Wait for gazebo before initializing controllers
        rospy.wait_for_service('gazebo/set_physics_properties')

    controller = Controller(is_head_activated, is_arms_activated, is_grippers_activated)
    if is_sim:
        Movement(controller)

    rospy.spin()


if __name__ == '__main__':
    main()
