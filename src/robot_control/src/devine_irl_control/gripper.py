# -*- coding: utf-8 -*-
""" IRL-1 Gripper use /command topic for 2 fingers on right & left hand """
__author__ = "Jordan Prince Tremblay, Ismael Balafrej, Felix Labelle, Felix Martel-Denis, Eric Matte, Adam Letourneau, Julien Chouinard-Beaupre, Antoine Mercier-Nicol"
__copyright__ = "Copyright 2018, DEVINE Project"
__credits__ = ["Simon Brodeur", "Francois Ferland", "Jean Rouat"]
__license__ = "BSD"
__version__ = "1.0.0"
__email__ = "devine.gegi-request@listes.usherbrooke.ca"
__status__ = "Production"

from devine_irl_control.irl_constant import ROBOT_CONTROLLER

from devine_irl_control.controllers import CommandPublisher


class Gripper(object):
    """ Gripper Controller """

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
        """ Close gripper with a percentage of maximum gripper closure """
        msg = percentage * self._joints_limit_interval + self._joints_limit[0]
        self._gripper_down.publish(msg)
        self._gripper_up.publish(msg)

    def open(self, percentage=1):
        """ Open gripper with a percentage of maximum gripper openness """
        msg = percentage * -self._joints_limit_interval + self._joints_limit[1]
        self._gripper_down.publish(msg)
        self._gripper_up.publish(msg)
