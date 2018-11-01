#!/usr/bin/env python2
"""
Node to find a human
"""
from __future__ import division
import json
import rospy
import actionlib
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from devine_head_coordinator.msg import LookAtHumanAction, LookAtHumanResult, LookAtHumanFeedback
from devine_config import topicname, constant
from devine_common.ros_utils import pose_stamped
from devine_common.math_utils import upper_left_to_zero_center, pixel_to_meters
import neck_action_control

# IN
BODY_TRACKING = topicname('body_tracking')
CAM_FRAME_OPTICAL = constant('cam_frame_optical')

# OUT
TOPIC_HEAD_LOOK_AT = topicname('robot_look_at')


class LookAtHumanActionServer(object):
    """ Action server to look at humans for a fixed period of time """

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name, LookAtHumanAction, execute_cb=self.execute_cb, auto_start=False)
        self._feedback = LookAtHumanFeedback()
        self._result = LookAtHumanResult()
        self._ctrl = rospy.Publisher(TOPIC_HEAD_LOOK_AT, PoseStamped, queue_size=1)
        # TODO: Validate with IRL-1 these numbers
        rospy.sleep(3)  # TODO: Wait for node to be initialized or starting the ctrl will fail
        self._ctrl_iterator = neck_action_control.NeckActionIterator([-0.5, 0.5], [0, 0], -0.1, 0)
        iter_time = 0.5
        self.iter_rate = rospy.Rate(1/iter_time)

        self._as.start()

    def get_human_eye(self, human):
        """ Return the eye position of an human """
        left_eye = None
        right_eye = None
        for body_part in human['body_parts']:
            if body_part['name'] == 'left_eye':
                left_eye = body_part
                break
            elif body_part['name'] == 'right_eye':
                right_eye = body_part
                # Keep looking for the left eye for consistent results

        target_eye = left_eye or right_eye
        if target_eye:
            centered = upper_left_to_zero_center(target_eye['x'], target_eye['y'], 640, 480)  # TODO: Dynamic image size
            in_meters = pixel_to_meters(centered[0], centered[1], 1)  # TODO: Dynamic z calculation
            return in_meters

        return None

    def execute_cb(self, goal):
        """ Callback when executing an action """
        rospy.loginfo('%s: Trying to find a human...', self._action_name)

        is_infinite = goal.period.is_zero()
        end_time = rospy.Time.now() + goal.period
        neck_iterator = iter(self._ctrl_iterator)

        while (is_infinite and not rospy.is_shutdown()) or rospy.Time.now() < end_time:
            try:
                human_object = rospy.wait_for_message(BODY_TRACKING, String, timeout=0.1)
                humans = json.loads(human_object.data)
            except rospy.ROSException:
                humans = []

            eyes = map(self.get_human_eye, humans)  # Find the eyes of the bodies
            eyes = filter(lambda i: i is not None, eyes)  # Filter out body w/o eyes
            eyes.sort(key=lambda pos: (pos[0] ** 2) + (pos[1] ** 2))  # Sort by closest to where the robot is seeing

            self._feedback.nb_humans = len(eyes)
            self._as.publish_feedback(self._feedback)

            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted', self._action_name)
                self._as.set_preempted()
                return

            if self._feedback.nb_humans is 0:
                next(neck_iterator)
            else:
                [x, y, z] = eyes[0]
                self._ctrl.publish(pose_stamped(x, y, z, 0, 0, 0, CAM_FRAME_OPTICAL))

            self.iter_rate.sleep()

        self._result.nb_humans = self._feedback.nb_humans
        self._as.set_succeeded(self._result)


def main():
    """ Entry point of this file """
    rospy.init_node('devine_human_finder')
    LookAtHumanActionServer('devine_human_finder')
    rospy.spin()


if __name__ == '__main__':
    main()
