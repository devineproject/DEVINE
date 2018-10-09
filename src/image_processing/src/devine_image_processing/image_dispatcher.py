#!/usr/bin/env python2
'''Image dispatching node'''

import rospy

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String, Bool
from devine_config import topicname
from blur_detection import is_image_blurry
from time import time
from ros_image_processor import ImageProcessor, ROSImageProcessingWrapper
from threading import RLock

#topics
IMAGE_TOPIC = topicname('raw_image')

BODY_TRACKING_IMAGE_TOPIC = topicname('body_tracking_image')
ZONE_DETECTION_IMAGE_TOPIC = topicname('zone_detection_image')
SEGMENTATION_IMAGE_TOPIC = topicname('segmentation_image')
FEATURES_EXTRACTION_IMAGE_TOPIC = topicname('features_extraction_image')

class Throttle(object):
    '''ROS publisher throttler'''
    def __init__(self, publisher, delay):
        self.publisher = publisher
        self.last_time = 0
        self.delay = delay
    
    def publish(self, *args):
        '''Call at a throttled rate'''
        current_time = time()
        if self.last_time + self.delay > current_time:
            return False
        self.last_time = current_time - 0.00001
        return self.publisher.publish(*args)


class SubscriberReady(object):
    '''Wrapper that checks if a topic was published with valid data'''
    class ReadyState(Enum):
        NO_INFO = 0
        NOT_READY = 1
        READY = 2

    def __init__(self, topic, type, default_state = ReadyState.NO_INFO, validator=lambda x : bool(x.data)):
        self.lock = RLock()
        self.sub = rospy.Subscriber(topic, type, self.topic_callback)
        self.validator = validator
        self.default_state = default_state
        self.is_ready = default_state

    def __del__(self):
        if self.sub:
            self.sub.unregister()
            self.sub = None

    def __call__(self):
        with self.lock:
            state = self.is_ready
            self.is_ready = self.default_state
            return state

    def topic_callback(self, data):
        '''Callback when topic is published'''
        with self.lock:
            self.is_ready = ReadyState.READY if self.validator(data) else ReadyState.NOT_READY

class ImageDispatcher(ImageProcessor):
    '''Dispatcher of frames to nodes'''

    def __init__(self):
        publishers = [
            Throttle(rospy.Publisher(BODY_TRACKING_IMAGE_TOPIC, CompressedImage, queue_size=1), 3),
            rospy.Publisher(ZONE_DETECTION_IMAGE_TOPIC, CompressedImage, queue_size=1),
            rospy.Publisher(SEGMENTATION_IMAGE_TOPIC, CompressedImage, queue_size=1),
            rospy.Publisher(FEATURES_EXTRACTION_IMAGE_TOPIC, CompressedImage, queue_size=1),
            None #Restart
        ]
        validators = [
            SubscriberReady(topicname('player_name'), String),                                      # Body_tracking
            SubscriberReady(topicname('scene_found'), Bool, SubscriberReady.ReadyState.NOT_READY),  # Zone_detection
            SubscriberReady(SEGMENTATION_IMAGE_TOPIC, CompressedImage),                             # Segmentation
            SubscriberReady(FEATURES_EXTRACTION_IMAGE_TOPIC, CompressedImage),                      # Features Extraction
            SubscriberReady(topicname('guess_category'), String)                                    # Restart
        ]
        self.pub_topics = zip(publishers, validators)
        self.pub_index = 0

    def process(self, img):
        '''Remove blurry images and submit the images to the current module'''     
        if is_image_blurry(img):
            return         
        
        while not rospy.is_shutdown():
            (publisher, validator) = self.pub_topics[self.pub_index]
            is_valid = validator()

            if is_valid == SubscriberReady.ReadyState.READY:
                self.pub_index += 1
                self.pub_index %= len(self.pub_index)        
            elif is_valid == SubscriberReady.ReadyState.NOT_READY
                if publisher:
                    publisher.publish(img)
                break
            elif is_valid == SubscriberReady.ReadyState.NO_INFO
                break

            rospy.sleep(.05)

    def bridge_img(self, img):
        '''Bridge the images to a single node'''
        if not img: # Remove discarded images
            return

        while True:
            (publisher, validator) = self.pub_topics[self.pub_index]
            if publisher:
                publisher.publish(img)

            try:
                is_valid = validator()
            except rospy.ROSException:
                is_valid = False
            if is_valid:
                self.pub_index += 1
                self.pub_index %= len(self.pub_index)
            else:
                break

def main():
    '''Entry point of this file'''
    image_dispatcher = ImageDispatcher()
    processor = ROSImageProcessingWrapper(image_dispatcher, IMAGE_TOPIC)
    processor.loop(image_dispatcher.bridge_img)

if __name__ == '__main__':
    main()
