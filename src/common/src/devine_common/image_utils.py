#! /usr/bin/env python2
""" Utils file for frequently used functions """
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
