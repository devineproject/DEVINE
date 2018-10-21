''' Inverse kinematics: from 3D position, compute joints position needed '''

import math

# Constant from IRL-1 specification documents
D_2_R = -0.09471141
D_2_L = 0.09471141

def arms_pan_tilt(controller, obj_x, obj_y, obj_z):
    ''''
    Inverse Kinematic for arms pan and tils joints
    Equations solved by Arnaud Aumont, IntRoLab, version 22/01/2012
    '''

    obj_z = obj_z + 0.25 # by test & error

    if controller == 'right':
        d_2 = D_2_R
    elif controller == 'left':
        d_2 = D_2_L
    else:
        raise Exception('The specified controller is not supported.')

    pan = math.acos(d_2/(math.sqrt(math.pow(obj_x, 2) + math.pow(obj_y, 2)))) \
          - math.atan2(obj_x, obj_y)
    tilt = -math.atan2(obj_z, (math.pow(obj_x, 2) + math.pow(obj_y, 2) - math.pow(d_2, 2))) \
           - (math.pi / 2)
    positions = [pan, tilt, 0, 0]

    return positions

def head_pan_tilt(obj_x, obj_y, obj_z):
    '''' Inverse Kinematic for head pan and tils joints '''

    pan = (math.pi / 2) - math.atan2(obj_x, obj_y)
    tilt = -math.atan2(obj_z, obj_x)
    positions = [pan, tilt]

    return positions
