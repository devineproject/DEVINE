""" Math Utils """
__author__ = "Jordan Prince Tremblay, Ismael Balafrej, Felix Labelle, Félix Martel-Denis, Eric Matte, Adam Letourneau, Julien Chouinard-Beaupré, Antoine Mercier-Nicol"
__copyright__ = "Copyright 2018, DEVINE Project"
__credits__ = ["Simon Brodeur", "François Ferland", "Jean Rouat"]
__license__ = "BSD"
__version__ = "1.0.0"
__email__ = "devine.gegi-request@listes.usherbrooke.ca"
__status__ = "Production"

import math
import matplotlib.pyplot as plt

FOV_CAM = 57  # fov angle in kinect spects


def upper_left_to_zero_center(x, y, width, height):
    """ Change referential from center to upper left """
    return (x - int(width/2), y - int(height/2))


def pixel_to_meters(x, y, z_meters, width):
    """ Compute (x, y) pixel coords to (x, y, z) pixel coords in meters """
    f = width / (2 * math.tan(math.radians(FOV_CAM/2)))
    d = z_meters / f
    x = d * x
    y = d * y
    return [x, y, z_meters]


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
    return max(min_nb, min(number, max_nb))
