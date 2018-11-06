#!/usr/bin/env python2
""" Depth mask of images based on a distance threshold """

import rospy
import message_filters
import numpy as np
from sensor_msgs.msg import Image as ROSImage
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from devine_config import topicname
from devine_common.image_utils import image_to_ros_msg

#Topics
IMAGE_PUB_TOPIC = topicname('segmentation_image')
DEPTH_THRESHOLD = 1.5 # Depth in meters

class DepthMask(object):
    """ Node that applies a depth mask to RGB image and resends the masked image """
    def __init__(self):
        self.bridge = CvBridge()
        image_sub = message_filters.Subscriber('camera/rgb/image_color', ROSImage)
        depth_sub = message_filters.Subscriber('camera/depth/image', ROSImage)
        self.masked_image = None
        self.synched_topics = message_filters.ApproximateTimeSynchronizer([image_sub,\
 depth_sub], 1, 0.5)
        self.synched_topics.registerCallback(self.callback)
        self.depth_mask_publisher = rospy.Publisher(IMAGE_PUB_TOPIC, CompressedImage, queue_size=1)

    def callback(self, rgb_data, depth_data):
        """ Call back to synchronised topics that applies the depth mask"""
        try:
            image = self.bridge.imgmsg_to_cv2(rgb_data, 'bgr8')
            depth_image = self.bridge.imgmsg_to_cv2(depth_data, 'passthrough')
            self.send_masked_image(image, depth_image)
        except CvBridgeError as error_msg:
            rospy.logerr(error_msg)

    def send_masked_image(self, image, depth_image):
        """ Sends masked image to image pub topic """
        nan_indices = np.isnan(depth_image)
        idx = np.where(~nan_indices, np.arange(nan_indices.shape[1]), 0)
        np.maximum.accumulate(idx, axis=1, out=idx)
        filled_in_depth_image = depth_image[np.arange(idx.shape[0])[:, None], idx]
        mask = filled_in_depth_image < DEPTH_THRESHOLD
        self.masked_image = image
        self.masked_image[~mask] = [0, 0, 0]
        msg = image_to_ros_msg(self.masked_image)
        self.depth_mask_publisher.publish(msg)

def main():
    """ Init node """
    rospy.init_node('devine_depth_mask')
    DepthMask()
    rospy.loginfo('Node initialized')
    rospy.spin()


if __name__ == '__main__':
    main()
