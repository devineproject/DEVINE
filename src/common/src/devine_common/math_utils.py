""" Math Utils """

import math
import matplotlib.pyplot as plt


def quaternion_mult(q, r):
    """ Quaternion multiplication """
    return [r[0]*q[0]-r[1]*q[1]-r[2]*q[2]-r[3]*q[3],
            r[0]*q[1]+r[1]*q[0]-r[2]*q[3]+r[3]*q[2],
            r[0]*q[2]+r[1]*q[3]+r[2]*q[0]-r[3]*q[1],
            r[0]*q[3]-r[1]*q[2]+r[2]*q[1]+r[3]*q[0]]


def point_rotation_by_quaternion(point, q):
    """ Point rotation by a quaternion """
    # https://math.stackexchange.com/questions/40164/how-do-you-rotate-a-vector-by-a-unit-quaternion
    [x, y, z] = point
    r = [0, x, -1*y, -1*z]
    q_conj = [q[0], 1*q[1], 1*q[2], 1*q[3]]
    return quaternion_mult(quaternion_mult(q, r), q_conj)[1:]


def upper_left_to_zero_center(x, y, width, height):
    """ Change referential from center to upper left """
    return (x - int(width/2), y - int(height/2))

def pixel_to_meters(x, y, z_meters, width):
    '''Compute (x, y) pixel coords to (x, y, z) pixel coords in meters'''
    # 57 = fov angle in kinect spects
    f = width / (2 * math.tan(math.radians(57/2)))
    d = z_meters / f
    x = d * x
    y = d * y
    return [x, y, z_meters]

def calc_geometric_location(x_pixel, y_pixel, kinect_z,
                            width, _height,
                            trans_kinect, rot_kinect):
    """ Compute geometric 3D location """
    [x, y, kinect_z] = pixel_to_meters(x_pixel, y_pixel, kinect_z, width)

    [transz, transx, transy] = trans_kinect
    point = [kinect_z + transz, x + transx, y + transy]
    return point_rotation_by_quaternion(point, rot_kinect)


def plot_3d_matrix(points):
    """ Draw an array of (x, y, z) tuples with matplotlib """
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    vects = zip(*points)
    xdata = list(vects[0])
    ydata = list(vects[1])
    zdata = list(vects[2])
    ax.scatter(xdata, ydata, zdata, c=zdata, cmap='Greens')
    plt.show()


def clip(number, min_nb, max_nb):
    """ Clip a number between min and max inclusively """
    if number <= min_nb:
        return min_nb
    elif number >= max_nb:
        return max_nb
    return number
