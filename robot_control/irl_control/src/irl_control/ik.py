import math

# Example with 4 joints:
# for object at [0.7, 0.31430004, -0.5],
# joints at [0.5457686076657069, -0.8591788022096626, -0.1, -0.4]

# Constant from IRL-1 specification documents
D_2 = -0.09471141

def arm_pan_tilt(controller, obj_x, obj_y, obj_z):
    positions = None
    obj_z = obj_z + 0.25 # by test & error

    if controller == 'right':
        pan = math.acos(D_2 / (math.sqrt(math.pow(obj_x, 2) + math.pow(obj_y, 2)))) - math.atan(obj_x / obj_y)
        tilt = -math.atan(obj_z / (math.pow(obj_x, 2) + math.pow(obj_y, 2) - math.pow(D_2, 2))) - (math.pi / 2)
        positions = [pan, tilt, 0, 0]

    elif controller == 'left':
        pan = -math.acos(D_2 / (math.sqrt(math.pow(obj_x, 2) + math.pow(obj_y, 2)))) - math.atan(obj_x / obj_y)
        tilt = math.atan(obj_z / (math.pow(obj_x, 2) + math.pow(obj_y, 2) - math.pow(D_2, 2))) + (math.pi / 2)
        positions = [pan, -tilt, 0, 0]

    return positions

def head_pan_tilt(obj_x, obj_y, obj_z):
    pan = (math.pi / 2) - math.atan(obj_x / obj_y)
    tilt = -math.atan(obj_z / obj_x)
    positions = [pan, tilt]

    return positions
