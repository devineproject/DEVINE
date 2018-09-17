import math

# Example with 4 joints:
# for object at [0.7, 0.31430004, -0.5],
# joints at [0.5457686076657069, -0.8591788022096626, -0.1, -0.4]

# Constant from IRL-1 specification documents
D_2_R = -0.09471141
D_2_L = 0.09471141

def arm_pan_tilt(controller, obj_x, obj_y, obj_z):
    obj_z = obj_z + 0.25 # by test & error

    if controller == 'right':
        D_2 = D_2_R
    elif controller == 'left':
        D_2 = D_2_L
    else:
        raise Exception('The specified controller is not supported.')

    pan = math.acos(D_2 / (math.sqrt(math.pow(obj_x, 2) + math.pow(obj_y, 2)))) - math.atan2(obj_x, obj_y)
    tilt = -math.atan2(obj_z, (math.pow(obj_x, 2) + math.pow(obj_y, 2) - math.pow(D_2, 2))) - (math.pi / 2)
    positions = [pan, tilt, 0, 0]

    return positions

def head_pan_tilt(obj_x, obj_y, obj_z):
    pan = (math.pi / 2) - math.atan2(obj_x, obj_y)
    tilt = -math.atan2(obj_z, obj_x)
    positions = [pan, tilt]

    return positions
