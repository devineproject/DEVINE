#!/usr/bin/env python2
'''Image dispatching node'''

import rospy
from sensor_msgs.msg import CompressedImage
from devine_config import topicname
from blur_detection import is_image_blurry
from time import time
from ros_image_processor import ImageProcessor, ROSImageProcessingWrapper

#topics
IMAGE_TOPIC = topicname('raw_image')

SEGMENTATION_IMAGE_TOPIC = topicname('segmentation_image')
ZONE_DETECTION_IMAGE_TOPIC = topicname('zone_detection_image')
FEATURES_EXTRACTION_IMAGE_TOPIC = topicname('features_extraction_image')
BODY_TRACKING_IMAGE_TOPIC = topicname('body_tracking_image')

TIMER_DELAY = 125

class ImageDispatcher(ImageProcessor):
    '''Dispatcher of frames to nodes'''

    def __init__(self):
        self.last_time = 0
        self.segmentation_pub = rospy.Publisher(SEGMENTATION_IMAGE_TOPIC, CompressedImage, queue_size=1)
        self.zone_detection_pub = rospy.Publisher(ZONE_DETECTION_IMAGE_TOPIC, CompressedImage, queue_size=1)
        self.features_extraction_pub = rospy.Publisher(FEATURES_EXTRACTION_IMAGE_TOPIC, CompressedImage, queue_size=1)
        self.body_tracking_pub = rospy.Publisher(BODY_TRACKING_IMAGE_TOPIC, CompressedImage, queue_size=1)

    def process(self, img):
        '''Remove blurry images and throttle the img rate'''
        current_time = time()
        if self.last_time + TIMER_DELAY > current_time:
            return None
        self.last_time = current_time            
        return is_image_blurry(img) ? None : img

    def bridge_img(self, img):
        if not img: # Remove discarded images
            return

        self.segmentation_pub.publish(img)
        self.zone_detection_pub.publish(img)
        self.features_extraction_pub.publish(img)
        self.body_tracking_pub.publish(img)

def main():
    '''Entry point of this file'''
    image_dispatcher = ImageDispatcher()
    processor = ROSImageProcessingWrapper(image_dispatcher, IMAGE_TOPIC)
    processor.loop(image_dispatcher.bridge_img)

if __name__ == '__main__':
    main()
