from command_publisher import CommandPublisher
from irl_constant import ROBOT_CONTROLLER

class Gripper(object):
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
        self._gripper_down.publish(percentage * self._joints_limit_interval + self._joints_limit[0])
        self._gripper_up.publish(percentage * self._joints_limit_interval + self._joints_limit[0])

    def open(self, percentage=1):
        self._gripper_down.publish(percentage * -self._joints_limit_interval + self._joints_limit[1])
        self._gripper_up.publish(percentage * -self._joints_limit_interval + self._joints_limit[1])
