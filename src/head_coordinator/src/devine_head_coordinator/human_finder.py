#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Node to find a human
"""
__author__ = "Jordan Prince Tremblay, Ismael Balafrej, Felix Labelle, Félix Martel-Denis, Eric Matte, Adam Letourneau, Julien Chouinard-Beaupré, Antoine Mercier-Nicol"
__copyright__ = "Copyright 2018, DEVINE Project"
__credits__ = ["Simon Brodeur", "François Ferland", "Jean Rouat"]
__license__ = "BSD"
__version__ = "1.0.0"
__email__ = "devine.gegi-request@listes.usherbrooke.ca"
__status__ = "Production"

from __future__ import division
import json
import rospy
import tf
import actionlib
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, Point
from devine_head_coordinator.msg import LookAtHumanAction, LookAtHumanResult, LookAtHumanFeedback
from devine_config import topicname, constant
from devine_common.ros_utils import pose_stamped, get_image_dim
from devine_common.math_utils import upper_left_to_zero_center, pixel_to_meters
import head_action_control

# IN
BODY_TRACKING = topicname('body_tracking')
CAM_FRAME_OPTICAL = constant('cam_frame_optical')

# OUT
# /devine/human_finder actionlib

# Consts
HARDCODED_HUMAN_DIST = 1.5

class LookAtHumanActionServer(object):
    """ Action server to look at humans for a fixed period of time """

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name, LookAtHumanAction, execute_cb=self.execute_cb, auto_start=False)
        self._feedback = LookAtHumanFeedback()
        self._result = LookAtHumanResult()
        self._ctrl = head_action_control.HeadActionCtrl(
            [-0.5, 0.5], [-0.17, 0.79], -0.1, 0, [None, 0])
        iter_time = 0.5
        self.iter_rate = rospy.Rate(1/iter_time)
        self._as.start()
        self.image_dim = get_image_dim()

    def get_human_eye(self, human):
        """ Return the eye position of an human """
        left_eye = None
        right_eye = None
        for body_part in human['body_parts']:
            if body_part['name'] == 'LEye':
                left_eye = body_part
                break
            elif body_part['name'] == 'REye':
                right_eye = body_part
                # Keep looking for the left eye for consistent results

        target_eye = left_eye or right_eye
        if target_eye:
            (image_width, image_height) = self.image_dim
            target_x = target_eye['x'] * image_width + 0.5
            target_y = target_eye['y'] * image_height + 0.5
            centered = upper_left_to_zero_center(
                target_x, target_y, image_width, image_height)
            # TODO: Dynamic z calculation
            in_meters = pixel_to_meters(
                centered[0], centered[1], HARDCODED_HUMAN_DIST, image_width)
            return [target_eye['stamp'], in_meters]
        return None

    def execute_cb(self, goal):
        """ Callback when executing an action """
        rospy.loginfo('%s: Trying to find a human...', self._action_name)

        is_infinite = goal.period.is_zero()
        end_time = rospy.Time.now() + goal.period
        head_iterator = iter(self._ctrl)

        while (is_infinite and not rospy.is_shutdown()) or rospy.Time.now() < end_time:
            try:
                human_object = rospy.wait_for_message(
                    BODY_TRACKING, String, timeout=None)
                humans = json.loads(human_object.data)
            except rospy.ROSException:
                humans = []

            # Find the eyes of the bodies
            eyes = map(self.get_human_eye, humans)
            # Filter out body w/o eyes
            eyes = filter(lambda i: i is not None, eyes)
            # Sort by closest to where the robot is seeing
            eyes.sort(key=lambda pos: (pos[1][0] ** 2) + (pos[1][1] ** 2))

            self._feedback.nb_humans = len(eyes)
            self._as.publish_feedback(self._feedback)

            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted', self._action_name)
                self._as.set_preempted()
                return

            human_found = self._feedback.nb_humans > 0
            if human_found:
                [stamp, [x, y, z]] = eyes[0]
                point = PointStamped(point=Point(x, y, z))
                point.header.stamp = rospy.Time(0, stamp)
                point.header.frame_id = CAM_FRAME_OPTICAL
                # look_at returns false if human is out of bounds
                human_found &= self._ctrl.look_at(point)

            if not human_found:
                next(head_iterator)

            self.iter_rate.sleep()

        self._result.nb_humans = self._feedback.nb_humans
        self._as.set_succeeded(self._result)


def main():
    """ Entry point of this file """
    rospy.init_node('human_finder')
    LookAtHumanActionServer('human_finder')
    rospy.spin()


if __name__ == '__main__':
    main()
