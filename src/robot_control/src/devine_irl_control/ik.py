# -*- coding: utf-8 -*-
""" Inverse kinematics: from 3D position, compute joints position needed """
__author__ = "Jordan Prince Tremblay, Ismael Balafrej, Felix Labelle, Félix Martel-Denis, Eric Matte, Adam Letourneau, Julien Chouinard-Beaupré, Antoine Mercier-Nicol"
__copyright__ = "Copyright 2018, DEVINE Project"
__credits__ = ["Simon Brodeur", "François Ferland", "Jean Rouat"]
__license__ = "BSD"
__version__ = "1.0.0"
__email__ = "devine.gegi-request@listes.usherbrooke.ca"
__status__ = "Production"

import math

# Constant from IRL-1 specification documents
D_2_IRL1 = 0.09471141

def arms_pan_tilt(controller, obj_x, obj_y, obj_z):
    """ Inverse Kinematic for arms pan and tils joints
    Equations solved by Arnaud Aumont, IntRoLab, version 22/01/2012
    """
    obj_z = obj_z + 0.25  # by test & error

    if controller == 'right':
        d_2 = -D_2_IRL1
    elif controller == 'left':
        d_2 = D_2_IRL1
    else:
        raise Exception('The specified controller is not supported.')

    pan = math.acos(d_2 / (math.sqrt(math.pow(obj_x, 2) + math.pow(obj_y, 2))))
    pan -= math.atan2(obj_x, obj_y)

    tilt = -math.atan2(obj_z, (math.pow(obj_x, 2) + math.pow(obj_y, 2) - math.pow(d_2, 2)))
    tilt -= (math.pi / 2)

    positions = [pan, tilt, 0, 0]

    return positions


def head_pan_tilt(obj_x, obj_y, obj_z):
    """ Inverse Kinematic for head pan and tils joints """
    pan = math.atan2(obj_y, obj_x)
    tilt = -math.atan2(obj_z, obj_x)
    return [pan, tilt]
