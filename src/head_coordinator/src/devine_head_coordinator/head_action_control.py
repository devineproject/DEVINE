""" Head Motor Controls Helper Library """
import math
import rospy
import tf
from devine_irl_control.irl_constant import ROBOT_CONTROLLER, ROBOT_NAME, ROBOT_LINK
from devine_irl_control.controllers import TrajectoryClient
from devine_irl_control import ik
from devine_common.math_utils import clip


class HeadActionCtrl(object):
    """ Controller for the head motors (neck_pan, neck_tilt, eyes) """

    def __init__(self, neck_pan_bounds, neck_tilt_bounds, neck_pan_delta, neck_tilt_delta, starting_position=[None, None]):
        self.joint_ctrl = TrajectoryClient(ROBOT_NAME, 'neck_controller')
        self.limits = ROBOT_CONTROLLER[self.joint_ctrl.controller_name]['joints_limit']
        self.tf = tf.TransformListener()

        if neck_pan_bounds[0] > neck_pan_bounds[1]:
            raise Exception("Expected neck pan bounds to be [min, max]")
        elif self.limits[0][0] > neck_pan_bounds[0] or self.limits[0][1] < neck_pan_bounds[1]:
            raise Exception("Neck pan bounds are out of the available limits")
        elif neck_tilt_bounds[0] > neck_tilt_bounds[1]:
            raise Exception("Expected neck tilt bounds to be [min, max]")
        elif self.limits[1][0] > neck_tilt_bounds[0] or self.limits[1][1] < neck_tilt_bounds[1]:
            raise Exception("Neck tilt bounds are out of the available limits")

        self.neck_pan_bounds = neck_pan_bounds
        self.neck_tilt_bounds = neck_tilt_bounds
        self.neck_pan_delta = neck_pan_delta * -1  # Start from left to right
        self.neck_tilt_delta = neck_tilt_delta * -1
        self.current_pan = None
        self.current_tilt = None
        self.starting_position = starting_position

    def __iter__(self):
        [current_pan, current_tilt] = self.joint_ctrl.get_position()
        next_pan = self.starting_position[0]
        next_tilt = self.starting_position[1]
        if next_pan is None:
            next_pan = clip(
                current_pan, self.neck_pan_bounds[0], self.neck_pan_bounds[1])
        if next_tilt is None:
            next_tilt = clip(
                current_tilt, self.neck_tilt_bounds[0], self.neck_tilt_bounds[1])
        rospy.loginfo("Moving head joints to starting values")
        self._move_joints([next_pan, next_tilt])
        return self

    def __next__(self):
        next_pan = self.current_pan + self.neck_pan_delta
        if not self._is_in_pan_bounds(next_pan):
            self.change_pan_direction()
            next_pan = self.current_pan + self.neck_pan_delta

        next_tilt = self.current_tilt + self.neck_tilt_delta
        if not self._is_in_tilt_bounds(next_tilt):
            self.change_tilt_direction()
            next_tilt = self.current_tilt + self.neck_tilt_delta

        next_pan = clip(
            next_pan, self.neck_pan_bounds[0], self.neck_pan_bounds[1])
        next_tilt = clip(
            next_tilt, self.neck_tilt_bounds[0], self.neck_tilt_bounds[1])

        self._move_joints([next_pan, next_tilt])
        return self.joint_ctrl.get_position()

    def next(self):
        """ Python 2 support of __next__ """
        return self.__next__()

    def _is_in_pan_bounds(self, pan):
        """ Returns true if pan is in the current limits """
        return self.neck_pan_bounds[0] <= pan and self.neck_pan_bounds[1] >= pan

    def _is_in_tilt_bounds(self, tilt):
        """ Returns true if tilt is in the current limits """
        return self.neck_tilt_bounds[0] <= tilt and self.neck_tilt_bounds[1] >= tilt

    def _is_in_bounds(self, joints):
        """ Returns true if the joints are is in the current limits """
        return self._is_in_pan_bounds(joints[0]) and self._is_in_tilt_bounds(joints[1])

    def change_pan_direction(self):
        """ Change direction of the pan movement """
        self.neck_pan_delta *= -1

    def change_tilt_direction(self):
        """ Change direction of the tilt movement """
        self.neck_tilt_delta *= -1

    def _move_joints(self, positions):
        """ Wrapper to move a joint and wait for the result """
        self.joint_ctrl.clear()
        self.joint_ctrl.add_point(positions, 1)
        self.joint_ctrl.start()
        self.joint_ctrl.wait(30)
        self.current_pan = positions[0]
        self.current_tilt = positions[1]
        result = self.joint_ctrl.result()
        success = result and result.error_code == 0
        if not success and result.error_string:
            rospy.logerr('Failed to move joint: %s', result.error_string)
            return False
        return True

    def look_at(self, point):
        """ Do the reverse kinematik for IRL-1 to look at a point """
        joints = None
        try:
            # Average between left and right eyes for the look at
            pt_l_eye_ref = self.tf.transformPoint('/head_l_eye_link', point)
            pt_r_eye_ref = self.tf.transformPoint('/head_r_eye_link', point)
            x_eyes = (pt_l_eye_ref.point.x + pt_r_eye_ref.point.x) / 2.0
            y_eyes = (pt_l_eye_ref.point.y + pt_r_eye_ref.point.y) / 2.0
            z_eyes = (pt_l_eye_ref.point.z + pt_r_eye_ref.point.z) / 2.0
            joints = ik.head_pan_tilt(x_eyes, y_eyes, z_eyes)
            current_joints = self.joint_ctrl.get_position()
            joints[0] += current_joints[0]
            joints[1] += current_joints[1]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as err:
            rospy.logerr(err)

        if joints is not None and self._is_in_bounds(joints):
            return self._move_joints(joints)
        return False
