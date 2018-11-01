""" Head Motor Controls Helper Library """ 
import rospy
import actionlib
from devine_irl_control.irl_constant import ROBOT_CONTROLLER, ROBOT_NAME
from devine_irl_control.controllers import TrajectoryClient
from devine_common.math_utils import clip

class NeckActionIterator(object):
    """ Move iteratively the neck motors (neck_pan, neck_tilt) """
    def __init__(self, neck_pan_bounds, neck_tilt_bounds, neck_pan_delta, neck_tilt_delta):
        self.joint_ctrl = TrajectoryClient(ROBOT_NAME, 'neck_controller')
        self.limits = ROBOT_CONTROLLER[self.joint_ctrl.controller_name]['joints_limit']

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
        self.neck_pan_delta = neck_pan_delta * -1
        self.neck_tilt_delta = neck_tilt_delta * -1
        self.current_pan = None
        self.current_tilt = None

    def __iter__(self):
        [current_pan, current_tilt] = self.joint_ctrl.get_position()
        next_pan = clip(current_pan, self.neck_pan_bounds[0], self.neck_pan_bounds[1])
        next_tilt = clip(current_tilt, self.neck_tilt_bounds[0], self.neck_tilt_bounds[1])
        rospy.loginfo("Moving head joints to bounded values")
        self._move_joints([next_pan, next_tilt])
        return self

    def __next__(self):
        next_pan = self.current_pan + self.neck_pan_delta
        if self.neck_pan_bounds[0] > next_pan or self.neck_pan_bounds[1] < next_pan:
            self.change_pan_direction()
            next_pan = self.current_pan + self.neck_pan_delta
        
        next_tilt = self.current_tilt + self.neck_tilt_delta
        if self.neck_tilt_bounds[0] > next_tilt or self.neck_tilt_bounds[1] < next_tilt:
            self.change_tilt_direction()
            next_tilt = self.current_tilt + self.neck_tilt_delta

        self._move_joints([next_pan, next_tilt])
        return self.joint_ctrl.get_position()

    def next(self):
        """ Python 2 support of __next__ """
        return self.__next__()

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
            rospy.logerr('Failed to move joint: ' + result.error_string)
            return False
        return True
