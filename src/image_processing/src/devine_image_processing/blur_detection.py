#!/usr/bin/env python2
'''ROS node for blur detection'''
from imutils import paths
import cv2
import numpy as np
from Queue import Queue, Empty
from PIL import Image
from io import BytesIO
import rospy
from std_msgs.msg import Bool 
from sensor_msgs.msg import CompressedImage
from devine_config import topicname

BLUR_DETECTION_TOPIC = topicname('blur_detection')
IMAGE_TOPIC = topicname('blur_detection_image')
DETECTION_THRESHOLD = 200

def detect_image_blur(gray,threshold):
    """ compute the variance of the Laplacian to measure focus"""
    focus_measure = cv2.Laplacian(gray, cv2.CV_64F).var() 
    return focus_measure < threshold

if __name__ == '__main__':
    rospy.init_node('blur_detection')
    blur_detection_publisher = rospy.Publisher(BLUR_DETECTION_TOPIC, Bool, queue_size=1, latch=True)

    image_queue = Queue(2)

    def image_callback(data):
        '''Callback for image topic'''
        if image_queue.full():
            rospy.logwarn('blur_detection: image receiving rate is too high !')
            image_queue.get()
        image_queue.put(data.data)

    rospy.Subscriber(IMAGE_TOPIC, CompressedImage, image_callback)


    while not rospy.is_shutdown():
        try:    
            img = image_queue.get(timeout=1)
                
            gray_img = cv2.imdecode(np.asarray(bytearray(img),dtype=np.uint8),cv2.IMREAD_GRAYSCALE)
            results = detect_image_blur(gray_img,DETECTION_THRESHOLD)
            blur_detection_publisher.publish(results)
            rospy.loginfo('Frame processed')
        except Empty:
            pass
