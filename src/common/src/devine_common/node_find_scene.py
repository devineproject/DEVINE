#!/usr/bin/env python2
"""
Node to find the scene
"""
from __future__ import division
import json

import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from control_msgs.msg import FollowJointTrajectoryActionFeedback
# http://docs.ros.org/fuerte/api/control_msgs/html/msg/

from devine_config import topicname
from devine_irl_control.irl_constant import ROBOT_CONTROLLER
from devine_common.actionlib_goal_status import ActionlibGoalStatus

NODE_NAME = 'SceneFinder'
ZONE_DETECTION_TOPIC = topicname('zone_detection')
NECK_TRAJ_STATE_TOPIC = '/jn0/neck_controller/follow_joint_trajectory/feedback'
NECK_CTRL_STATE_TOPIC = '/jn0/neck_controller/state'
TOPIC_HEAD_JOINT_STATE = topicname('robot_head_joint_traj_point')
TOPIC_SCENE_FOUND = topicname('scene_found')

LIMITS = ROBOT_CONTROLLER['neck_controller']['joints_limit']

DELTA_TIME = 1
DELTA_POS = 0.8

class SceneFinder(object):
    ''' Find scene to play game '''

    def __init__(self, theta, time):
        self.current_joint_position = [0, 0]
        self.new_joint_position = [0, 0]
        self.current_error = [0, 0]
        self.is_scene_finder_moved = False
        self.theta = theta
        self.scene_joint_position = None
        self.time = time
        self.direction = -1
        self.current_status = ActionlibGoalStatus.SUCCEEDED
        self.current_zone = {
            "top_left_corner": [-1, -1],
            "bottom_right_corner": [-1, -1]
        }
        initial_joint_position = rospy.wait_for_message(NECK_CTRL_STATE_TOPIC,
                                                        JointTrajectoryControllerState)
        self.current_joint_position = initial_joint_position.actual.positions
        rospy.Subscriber(NECK_TRAJ_STATE_TOPIC, FollowJointTrajectoryActionFeedback,
                         self.traj_state_callback, queue_size=1)
        rospy.Subscriber(ZONE_DETECTION_TOPIC, String,
                         self.zone_callback, queue_size=1)

        self.pub_neck_ctrl = rospy.Publisher(TOPIC_HEAD_JOINT_STATE,
                                             JointTrajectoryPoint, queue_size=1)
        self.pub_scene_found = rospy.Publisher(TOPIC_SCENE_FOUND,
                                               Bool, queue_size=1)

    def traj_state_callback(self, data):
        ''' Monitor neck trajectory state '''

        self.current_status = ActionlibGoalStatus(data.status.status)
        self.current_joint_position = list(data.feedback.actual.positions)
        if self.current_status is ActionlibGoalStatus.SUCCEEDED:
            if self.is_scene_finder_moved is True:
                self.pub_scene_found.publish(False)
                self.is_scene_finder_moved = False

    def zone_callback(self, data):
        ''' Listen to zone_detection '''

        self.current_zone = json.loads(data.data)
        self.update()

    def update(self):
        ''' Move head to find scene '''

        # If joint position exceed limits, turn head in the other direction
        temp_delta_theta = self.theta * self.direction
        temp_joint_pos = self.new_joint_position[0] + temp_delta_theta
        if temp_joint_pos <= LIMITS[0][0]:
            self.direction = 1
        elif temp_joint_pos > LIMITS[0][1]:
            self.direction = -1

        # Find scene depending on zone_dection
        delta_theta = self.theta * self.direction
        top_left = self.current_zone['top_left_corner']
        bottom_right = self.current_zone['bottom_right_corner']

        if top_left == [-1, -1] and bottom_right == [-1, -1] or top_left == [-1, -1]:
            if self.scene_joint_position is None:
                self.new_joint_position[0] = self.current_joint_position[0] + delta_theta
            else:
                self.new_joint_position = self.scene_joint_position
                self.scene_joint_position = None
        elif bottom_right == [-1, -1]:
            if self.scene_joint_position is None:
                self.new_joint_position[0] = self.current_joint_position[0] - delta_theta
            else:
                self.new_joint_position = self.scene_joint_position
                self.scene_joint_position = None
        else:
            self.scene_joint_position = self.new_joint_position
            ros_packet = Bool(True)
            self.pub_scene_found.publish(ros_packet)

        if self.scene_joint_position is not self.new_joint_position:
            ros_packet = JointTrajectoryPoint(positions=self.new_joint_position,
                                              time_from_start=rospy.Duration(self.time))
            self.pub_neck_ctrl.publish(ros_packet)
            self.is_scene_finder_moved = True

def main():
    '''Entry point of this file'''

    rospy.init_node(NODE_NAME)
    SceneFinder(DELTA_POS, DELTA_TIME)
    rospy.spin()

if __name__ == '__main__':
    main()
