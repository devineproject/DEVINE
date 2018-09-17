#!/usr/bin/env python

import rospy
import tf
from std_msgs.msg import Float32MultiArray
from trajectory_client import TrajectoryClient
from gripper import Gripper
import ik
from movement import Movement

ROBOT = 'jn0'
TOPIC_OBJECT_LOCATION = "/object_location"
TOPIC_OBJECT_FRAME = "/object_frame"
TOPIC_ROBOT_BASE = "/base_link"
TOPIC_ROBOT_R_SHOULDER_FIXED_FRAME = "/R_shoulder_fixed_link"
TOPIC_ROBOT_L_SHOULDER_FIXED_FRAME = "/L_shoulder_fixed_link"
TOPIC_ROBOT_NECK_PAN_FRAME = "/neck_pan_link"
TOPIC_GUESSWHAT_SUCCEED = '/is_guesswhat_succeed'

class Controller(object):
    def __init__(self):
        self.object_location = None
        self.tf_listener = tf.TransformListener()
        self.gripper = Gripper(ROBOT, 'right')
        self.time = 3

        try:
            rospy.loginfo('Waiting for Arm and Head controllers')
            self.arm_right = TrajectoryClient(ROBOT, 'right_arm_controller')
            self.arm_left = TrajectoryClient(ROBOT, 'left_arm_controller')
            self.head = TrajectoryClient(ROBOT, 'neck_controller')
        except RuntimeError as err:
            rospy.logerr(err)
            rospy.signal_shutdown(err)

        try:
            self.init_tf(TOPIC_ROBOT_R_SHOULDER_FIXED_FRAME, TOPIC_OBJECT_FRAME)
            self.init_tf(TOPIC_ROBOT_R_SHOULDER_FIXED_FRAME, TOPIC_ROBOT_BASE)

            self.init_tf(TOPIC_ROBOT_L_SHOULDER_FIXED_FRAME, TOPIC_OBJECT_FRAME)
            self.init_tf(TOPIC_ROBOT_L_SHOULDER_FIXED_FRAME, TOPIC_ROBOT_BASE)

        except tf.Exception as err:
            rospy.logerr(err)
            rospy.signal_shutdown(err)

        rospy.Subscriber(TOPIC_OBJECT_LOCATION, Float32MultiArray, self.object_location_callback)

    def init_tf(self, target_frame, source_frame):
        time = self.tf_listener.getLatestCommonTime(target_frame, source_frame)
        self.tf_listener.lookupTransform(target_frame, source_frame, time)

    def object_location_callback(self, msg):
        if self.object_location != msg.data:
            rospy.loginfo(msg.data)
            self.object_location = msg.data
            self.now = rospy.Time().now()
            if msg.data != (0, 0, 0):
                self.calcul()
                self.move({'head': self.head_joints_position,
                           # 'arm_left': self.left_joints_position,
                           # 'arm_right': self.right_joints_position
                          },
                          self.time)
            else:
                self.move_init(10)

    def calcul(self):
        trans_r_arm = None
        trans_l_arm = None
        timeout = rospy.Duration(4)
        try:
            self.tf_listener.waitForTransform(TOPIC_ROBOT_R_SHOULDER_FIXED_FRAME,
                                              TOPIC_OBJECT_FRAME,
                                              self.now,
                                              timeout)
            (trans_r_arm, rot_r_arm) = self.tf_listener.lookupTransform(
                TOPIC_ROBOT_R_SHOULDER_FIXED_FRAME,
                TOPIC_OBJECT_FRAME,
                self.now)

            self.tf_listener.waitForTransform(TOPIC_ROBOT_L_SHOULDER_FIXED_FRAME,
                                              TOPIC_OBJECT_FRAME,
                                              self.now,
                                              timeout)
            (trans_l_arm, rot_l_arm) = self.tf_listener.lookupTransform(
                TOPIC_ROBOT_L_SHOULDER_FIXED_FRAME,
                TOPIC_OBJECT_FRAME,
                self.now)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as err:
            rospy.logerr(err)
            rospy.signal_shutdown(err)

        if not rospy.is_shutdown():
            trans_head = None
            try:
                self.tf_listener.waitForTransform(TOPIC_ROBOT_NECK_PAN_FRAME,
                                                  TOPIC_OBJECT_FRAME,
                                                  self.now, rospy.Duration(4))
                (trans_head, rot_head) = self.tf_listener.lookupTransform(
                    TOPIC_ROBOT_NECK_PAN_FRAME,
                    TOPIC_OBJECT_FRAME,
                    self.now)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as err:
                rospy.logerr(err)
                rospy.signal_shutdown(err)

        if not rospy.is_shutdown():
            rospy.loginfo('Translation from r_shoulder_fixed_link to obj: %s', trans_r_arm)
            rospy.loginfo('Translation from l_shoulder_fixed_link to obj: %s', trans_l_arm)
            rospy.loginfo('Translation from neck_pan_link to obj: %s', trans_head)

            # Calculate inverse kinematic
            self.right_joints_position = ik.arm_pan_tilt('right',
                                                         trans_r_arm[0],
                                                         trans_r_arm[1],
                                                         trans_r_arm[2])
            rospy.loginfo('Right Joint Position: %s', self.right_joints_position)

            self.left_joints_position = ik.arm_pan_tilt('left',
                                                        trans_l_arm[0],
                                                        trans_l_arm[1],
                                                        trans_l_arm[2])
            rospy.loginfo('Left Arm Joint Position: %s', self.left_joints_position)

            self.head_joints_position = ik.head_pan_tilt(trans_head[0],
                                                         trans_head[1],
                                                         trans_head[2])
            rospy.loginfo('Head Joint Position: %s', self.head_joints_position)

    def move_init(self, time):
        rospy.loginfo('move_init')
        self.move({'head': [0, 0],
                   'arm_left':  [0, 0, 0, 0],
                   'arm_right':  [0, 0, 0, 0]
                  },
                  time)

        rospy.loginfo('Completed')

    def move(self, controller_joints_positions, time):

        move_gripper = False

        times = get_joints_time(controller_joints_positions, time)

        for key in controller_joints_positions:
            getattr(self, key).clear()
            getattr(self, key).add_point(controller_joints_positions[key], times[key])
            getattr(self, key).start()
        for key in controller_joints_positions:
            getattr(self, key).wait(times[key])

        if move_gripper:
            i = 0
            while i < 3:
                self.gripper.open(0.3)
                rospy.sleep(0.5)
                self.gripper.open(0.1)
                rospy.sleep(0.5)
                i = i + 1

        rospy.loginfo('Completed')

def get_joints_time(controller_joints, time):
    if isinstance(time, int) or isinstance(time, float):
        times = {}
        for key in controller_joints:
            times[key] = time
    else:
        times = time
    return times

def main():
    node_name = 'irl_control_control'
    rospy.init_node(node_name)
    rospy.loginfo('Running node \'' + node_name + '\'')

    controller = Controller()
    Movement(controller)
    rospy.spin()

if __name__ == '__main__':
    main()
