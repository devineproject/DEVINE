#!/usr/bin/env python2
'''Image dispatching node'''

from queue import Queue, Empty
import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import CompressedImage
from devine_config import topicname
from blur_detection import is_image_blurry
from threading import Timer

#topics
IMAGE_TOPIC = topicname('raw_image')

SEGMENTATION_IMAGE_TOPIC = topicname('segmentation_image')
ZONE_DETECTION_IMAGE_TOPIC = topicname('zone_detection_image')
FEATURES_EXTRACTION_IMAGE_TOPIC = topicname('features_extraction_image')
BODY_TRACKING_IMAGE_TOPIC = topicname('body_tracking_image')
BLUR_DETECTION_TOPIC = topicname('blur_detection')

TIMER_DELAY = 10

def dispatch():
    '''Dipsatch frame to nodes'''
    global timer
    timer = Timer(TIMER_DELAY, dispatch)
    timer.start()
    try:
        image = raw_image_queue.get(timeout=TIMER_DELAY)
        is_blurry = is_image_blurry(image.data)
        blur_detection_pub.publish(is_blurry)
        if not is_blurry:
            segmentation_pub.publish(image)
            zone_detection_pub.publish(image)
            features_extraction_pub.publish(image)
            body_tracking_pub.publish(image)
    except Empty:
        pass

def raw_image_callback(data):
    '''Callback for image topic'''
    if raw_image_queue.full():
        raw_image_queue.get()
    raw_image_queue.put(data)

def cancel_timer():
    '''Cancel pending timer'''
    timer.cancel()

if __name__ == '__main__':
    rospy.init_node('image_dispatcher')
    raw_image_queue = Queue(1)
    is_blurry = True

    segmentation_pub = rospy.Publisher(SEGMENTATION_IMAGE_TOPIC, CompressedImage, queue_size=1)
    zone_detection_pub = rospy.Publisher(ZONE_DETECTION_IMAGE_TOPIC, CompressedImage, queue_size=1)
    features_extraction_pub = rospy.Publisher(FEATURES_EXTRACTION_IMAGE_TOPIC, CompressedImage, queue_size=1)
    body_tracking_pub = rospy.Publisher(BODY_TRACKING_IMAGE_TOPIC, CompressedImage, queue_size=1)
    blur_detection_pub = rospy.Publisher(BLUR_DETECTION_TOPIC, Bool, queue_size=1)

    rospy.Subscriber(IMAGE_TOPIC, CompressedImage, raw_image_callback)

    timer = Timer(0, dispatch)
    rospy.on_shutdown(cancel_timer)
    timer.start()

    rospy.spin()
