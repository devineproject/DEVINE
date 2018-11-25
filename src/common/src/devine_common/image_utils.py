#! /usr/bin/env python2
""" Utils file for frequently used functions """
__author__ = "Jordan Prince Tremblay, Ismael Balafrej, Felix Labelle, Félix Martel-Denis, Eric Matte, Adam Letourneau, Julien Chouinard-Beaupré, Antoine Mercier-Nicol"
__copyright__ = "Copyright 2018, DEVINE Project"
__credits__ = ["Simon Brodeur", "François Ferland", "Jean Rouat"]
__license__ = "BSD"
__version__ = "1.0.0"
__email__ = "devine.gegi-request@listes.usherbrooke.ca"
__status__ = "Production"

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage


def image_file_to_ros_msg(image_path):
    """ Convert an image file to a ros readable data message """
    img = cv2.imread(image_path, cv2.IMREAD_COLOR)
    return image_to_ros_msg(img)


def image_to_ros_msg(image):
    """ Convert an image into a compressedImage message """
    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "png"
    msg.data = np.array(cv2.imencode(".png", image)[1]).tostring()
    return msg
