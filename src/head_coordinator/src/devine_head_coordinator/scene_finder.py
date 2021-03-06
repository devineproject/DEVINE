#!/usr/bin/env python2
# -*- coding: utf-8 -*-
""" Node to find the scene """
from __future__ import division

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
from devine_config import topicname
from devine_irl_control.irl_constant import ROBOT_CONTROLLER, ROBOT_NAME
from devine_irl_control.controllers import TrajectoryClient
from devine_dialog.msg import TtsQuery, TtsAnswer
from devine_dialog import TTSAnswerType, send_speech

SCENE_DETECTION_TOPIC = topicname('start_scene_detection')
TOPIC_SCENE_FOUND = topicname('scene_found')
CAMERA_FRAME_TOPIC = '/openni_rgb_optical_frame'
TOP_LEFT_TOPIC = '/top_left'
BOTTOM_RIGHT_TOPIC = '/bottom_right'

DELTA_TIME = 0.3
DELTA_POS = 0.2
TILT = 0.30  # TODO: Iterate over tilt too ?

SPEAK_PUBLISHER = rospy.Publisher(topicname('tts_query'), TtsQuery, queue_size=1)


class SceneFinder(object):
    """ Scene finder based of april tags """

    def __init__(self, neck_joint_ctrl):
        self.joint_ctrl = neck_joint_ctrl
        self.limits = ROBOT_CONTROLLER[neck_joint_ctrl.controller_name]['joints_limit']
        self.tf = tf.TransformListener()
        self.scene_position = None
        self.pub_scene_found = rospy.Publisher(TOPIC_SCENE_FOUND, Bool, queue_size=1)
        rospy.Subscriber(SCENE_DETECTION_TOPIC, Bool, self.find, queue_size=1)

    def move_joints(self, positions):
        """ Wrapper to move a joint and wait for the result """
        self.joint_ctrl.clear()
        self.joint_ctrl.add_point(positions, 1)
        self.joint_ctrl.start()
        self.joint_ctrl.wait(30)
        result = self.joint_ctrl.result()
        success = result.error_code == 0
        if not success and result.error_string:
            rospy.logerr(result.error_string)
        return True

    def find(self, *_):
        """ Iterate joint angles until a scene is found """
        if self.scene_position:  # If we had previously find a pos, start from there
            self.move_joints(self.scene_position)
            self.scene_position = None

        rate = rospy.Rate(1/DELTA_TIME)
        not_in_bound_ctr = 0
        direction = -1
        while not rospy.is_shutdown():
            [pan, tilt] = self.joint_ctrl.get_position()
            tilt = TILT
            top_left_pos = self.look_up_tag_position(TOP_LEFT_TOPIC)
            bottom_right_pos = self.look_up_tag_position(BOTTOM_RIGHT_TOPIC)

            if top_left_pos and bottom_right_pos:
                self.scene_position = [pan, tilt]
                break
            if not bottom_right_pos and top_left_pos:
                direction = -1
            elif bottom_right_pos and not top_left_pos:
                direction = 1

            delta_pan = direction * DELTA_POS

            if not self.in_bounds(pan + delta_pan, tilt):
                if (not_in_bound_ctr % 4) == 3:
                    send_speech(SPEAK_PUBLISHER, 'I can\'t find the scene. Are the april tags visible ?', TTSAnswerType.YES_NO)
                    rospy.logwarn('Couldn\'t find the scene :(')
                not_in_bound_ctr += 1
                direction *= -1
                delta_pan = direction * DELTA_POS

            if not self.move_joints([pan+delta_pan, tilt]):
                break
            rate.sleep()

        # Publish if the scene was found or not
        self.pub_scene_found.publish(bool(self.scene_position))

    def look_up_tag_position(self, topic):
        """ Find a tag position based its topic name """
        position = None
        try:
            time = self.tf.getLatestCommonTime(CAMERA_FRAME_TOPIC, topic)
            if abs(time.to_sec() - rospy.get_rostime().to_sec()) < 4:  # If it was found in the last 4 sec
                position, _ = self.tf.lookupTransform(CAMERA_FRAME_TOPIC, topic, time)
        except tf.Exception as err:
            rospy.loginfo(err)  # Sometime they are not yet initialized
        return position

    def in_bounds(self, pan, tilt):
        """ Check if neck pan and tilt are in the limits """
        [pan_limits, tilt_limits] = self.limits
        return pan_limits[0] <= pan <= pan_limits[1] and \
            tilt_limits[0] <= tilt <= tilt_limits[1]


def main():
    """ Entry point of this file """
    rospy.init_node('devine_scene_finder')
    neck_joint_ctrl = TrajectoryClient(ROBOT_NAME, 'neck_controller')
    SceneFinder(neck_joint_ctrl)
    rospy.spin()


if __name__ == '__main__':
    main()
