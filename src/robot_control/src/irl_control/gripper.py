''' IRL-1 Gripper use /command topic for 2 fingers on right & left hand '''

from irl_control.irl_constant import ROBOT_CONTROLLER

from irl_control.controllers import CommandPublisher

class Gripper(object):
    ''' Gripper Controller '''

    def __init__(self, robot_name, gripper):
        self._joints_limit = ROBOT_CONTROLLER[gripper + '_gripper_up_controller']['joints_limit']
        self._joints_limit_interval = self._joints_limit[1] - self._joints_limit[0]

        if gripper == 'left':
            self._gripper_up = CommandPublisher(robot_name, 'left_gripper_up_controller')
            self._gripper_down = CommandPublisher(robot_name, 'left_gripper_down_controller')
        if gripper == 'right':
            self._gripper_up = CommandPublisher(robot_name, 'right_gripper_up_controller')
            self._gripper_down = CommandPublisher(robot_name, 'right_gripper_down_controller')

    def close(self, percentage=1):
        ''' Close gripper with a percentage of maximum gripper closure '''

        self._gripper_down.publish(percentage * self._joints_limit_interval + self._joints_limit[0])
        self._gripper_up.publish(percentage * self._joints_limit_interval + self._joints_limit[0])

    def open(self, percentage=1):
        ''' Open gripper with a percentage of maximum gripper openness '''

        self._gripper_down.publish(percentage * -self._joints_limit_interval + self._joints_limit[1])
        self._gripper_up.publish(percentage * -self._joints_limit_interval + self._joints_limit[1])
