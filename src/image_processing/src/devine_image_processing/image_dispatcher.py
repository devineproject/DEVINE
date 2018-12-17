#!/usr/bin/env python2
# -*- coding: utf-8 -*-
""" Image dispatching node """
__author__ = "Jordan Prince Tremblay, Ismael Balafrej, Felix Labelle, Felix Martel-Denis, Eric Matte, Adam Letourneau, Julien Chouinard-Beaupre, Antoine Mercier-Nicol"
__copyright__ = "Copyright 2018, DEVINE Project"
__credits__ = ["Simon Brodeur", "Francois Ferland", "Jean Rouat"]
__license__ = "BSD"
__version__ = "1.0.0"
__email__ = "devine.gegi-request@listes.usherbrooke.ca"
__status__ = "Production"

from threading import RLock
from enum import IntEnum
from time import time
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String, Bool
from devine_config import topicname
from blur_detection import is_image_blurry
from ros_image_processor import ImageProcessor, ROSImageProcessingWrapper

# IN
IMAGE_TOPIC = topicname('masked_image')

# OUT
BODY_TRACKING_IMAGE_TOPIC = topicname('body_tracking_image')
ZONE_DETECTION_IMAGE_TOPIC = topicname('zone_detection_image_in')
SEGMENTATION_IMAGE_TOPIC = topicname('segmentation_image')
FEATURES_EXTRACTION_IMAGE_TOPIC = topicname('features_extraction_image')


class Throttle(object):
    """ ROS publisher throttler """

    def __init__(self, publisher, delay):
        self.publisher = publisher
        self.last_time = 0
        self.delay = delay

    def publish(self, *args):
        """ Call at a throttled rate """
        current_time = time()
        if self.last_time + self.delay > current_time:
            return False
        self.last_time = current_time - 0.00001
        return self.publisher.publish(*args)


class TopicState(IntEnum):
    """ Enum of the possible states of a topic """
    NOTHING_RECEIVED = 1 << 0
    RECEIVED_YES = 1 << 1        # validator returns true
    RECEIVED_NO = 1 << 2         # validator returns false

    def __contains__(self, item):
        if isinstance(item, TopicState):
            item = item.value
        return (self.value & item) > 0


class SubscriberReady(object):
    """ Wrapper that checks if a topic was published with valid data """

    def __init__(self, topic, topic_type, validator=lambda x: bool(x.data)):
        self.__lock = RLock()
        self.__sub = rospy.Subscriber(topic, topic_type, self._topic_callback)
        self.validator = validator
        self.topic_state = TopicState.NOTHING_RECEIVED

    def __del__(self):
        if self.__sub:
            self.__sub.unregister()
            self.__sub = None

    def __call__(self):
        with self.__lock:
            state = self.topic_state
            self.topic_state = TopicState.NOTHING_RECEIVED
            return state

    def _topic_callback(self, data):
        """ Callback when topic is published """
        with self.__lock:
            self.topic_state = TopicState.RECEIVED_YES if self.validator(data) else TopicState.RECEIVED_NO


class SmartImagePublisher(object):
    """ Republish an image to a topic_name based on the result of the state_validator """

    def __init__(self, topic_name, state_validator, throttle_rate=None, publish_state=TopicState.NOTHING_RECEIVED | TopicState.RECEIVED_NO):
        self.publisher = rospy.Publisher(topic_name, CompressedImage, queue_size=1) if topic_name else None
        if self.publisher and throttle_rate:
            self.publisher = Throttle(self.publisher, throttle_rate)

        self.state_validator = state_validator
        self.publish_state = publish_state

    def process(self, payload):
        """ Process func for a specific publisher """
        state = self.state_validator()
        if self.publish_state in state:
            if self.publisher:
                self.publisher.publish(payload)
        return TopicState.RECEIVED_YES in state


class ImageDispatcher(ImageProcessor):
    """ Dispatcher of frames to nodes """

    def __init__(self):
        body_tracking_validator = SubscriberReady(topicname('player_name'), String)
        zone_detection_validator = SubscriberReady(topicname('scene_found'), Bool)
        seg_validator = SubscriberReady(SEGMENTATION_IMAGE_TOPIC, CompressedImage)
        features_validator = SubscriberReady(FEATURES_EXTRACTION_IMAGE_TOPIC, CompressedImage)
        restart_validator = SubscriberReady(topicname('new_game'), Bool)

        self.smart_publishers = [
            SmartImagePublisher(BODY_TRACKING_IMAGE_TOPIC, body_tracking_validator, throttle_rate=1.5),
            SmartImagePublisher(ZONE_DETECTION_IMAGE_TOPIC, zone_detection_validator, throttle_rate=0.5),
            SmartImagePublisher(SEGMENTATION_IMAGE_TOPIC, seg_validator),
            SmartImagePublisher(FEATURES_EXTRACTION_IMAGE_TOPIC, features_validator),
            SmartImagePublisher(BODY_TRACKING_IMAGE_TOPIC, restart_validator, throttle_rate=1.5)
        ]

        self.pub_index = 0

    def process(self, img, img_payload):
        """ Remove blurry images and submit the images to the current module """
        if is_image_blurry(img):
            rospy.logwarn('Discarding blurry image')
            return
        while not rospy.is_shutdown():
            image_publisher = self.smart_publishers[self.pub_index]
            if image_publisher.process(img_payload):
                self.pub_index += 1
                self.pub_index %= len(self.smart_publishers)
            else:
                break


def main():
    """ Entry point of this file """
    processor = ROSImageProcessingWrapper(ImageDispatcher, IMAGE_TOPIC)
    processor.loop()


if __name__ == '__main__':
    main()
