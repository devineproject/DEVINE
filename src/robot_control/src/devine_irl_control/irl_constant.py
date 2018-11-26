# -*- coding: utf-8 -*-
""" IRL-1 Constants: Contains the controller names, joint names and joint limits """
__author__ = "Jordan Prince Tremblay, Ismael Balafrej, Felix Labelle, Felix Martel-Denis, Eric Matte, Adam Letourneau, Julien Chouinard-Beaupre, Antoine Mercier-Nicol"
__copyright__ = "Copyright 2018, DEVINE Project"
__credits__ = ["Simon Brodeur", "Francois Ferland", "Jean Rouat"]
__license__ = "BSD"
__version__ = "1.0.0"
__email__ = "devine.gegi-request@listes.usherbrooke.ca"
__status__ = "Production"

ROBOT_NAME = 'jn0'

ROBOT_LINK = {
    'base': '/base_link',
    'neck_pan': '/neck_pan_link',
    'l_shoulder_fixed': '/L_shoulder_fixed_link',
    'r_shoulder_fixed': '/R_shoulder_fixed_link',
    'r_frame_tool': '/R_frame_tool_link',
    'l_frame_tool': '/L_frame_tool_link',
    'l_eye': '/head_l_eye_link',
}

ROBOT_CONTROLLER = {
    'neck_controller':  {
        'joints_name': ['neck_pan_joint', 'neck_tilt_joint'],
        'joints_limit': [[-1.57, 1.57], [-0.17, 0.79]]
    },
    'left_arm_controller': {
        'joints_name': ['L_shoulder_pan_joint',
                        'L_shoulder_tilt_joint',
                        'L_shoulder_roll_joint',
                        'L_elbow_tilt_joint'],
        'joints_limit': [[-1.57, 1.57], [-3.14, 1.57], [-1.57, 1.57], [-0.75, 1.22]]
    },
    'right_arm_controller': {
        'joints_name': ['R_shoulder_pan_joint',
                        'R_shoulder_tilt_joint',
                        'R_shoulder_roll_joint',
                        'R_elbow_tilt_joint'],
        'joints_limit': [[-1.57, 1.57], [-3.14, 1.57], [-1.57, 1.57], [-0.75, 1.22]]
    },
    'left_gripper_up_controller': {
        'joints_limit': [-1.7, 0.4]  # 0.4 is closed
    },
    'left_gripper_down_controller': {
        'joints_limit': [-1.7, 0.4]  # 0.4 is closed
    },
    'right_gripper_down_controller': {
        'joints_limit': [-1.7, 0.4]  # 0.4 is closed
    },
    'right_gripper_up_controller': {
        'joints_limit': [-1.7, 0.4]  # 0.4 is closed
    }
}
