#! /usr/bin/env python2
""" Test to validate segmentation rates """
import os
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage

def image_file_to_ros_msg(image_path):
    """ Convert an image file to a ros readable data message """
    img = cv2.imread(get_fullpath(image_path), cv2.IMREAD_COLOR)

    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "png"
    msg.data = np.array(cv2.imencode(".png", img)[1]).tostring()

    return msg


def get_fullpath(relative_file):
    """ Return the full path of a file in a directory relative to this one """
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), relative_file)
