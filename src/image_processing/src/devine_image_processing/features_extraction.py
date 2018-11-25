#!/usr/bin/env python3
# -*- coding: utf-8 -*-
""" ROS node for features extraction """

__author__ = "Jordan Prince Tremblay, Ismael Balafrej, Felix Labelle, Félix Martel-Denis, Eric Matte, Adam Letourneau, Julien Chouinard-Beaupré, Antoine Mercier-Nicol"
__copyright__ = "Copyright 2018, DEVINE Project"
__credits__ = ["Simon Brodeur", "François Ferland", "Jean Rouat"]
__license__ = "BSD"
__version__ = "1.0.0"
__email__ = "devine.gegi-request@listes.usherbrooke.ca"
__status__ = "Production"

import cv2
import tensorflow as tf
import numpy as np
import rospy
from PIL import Image
from rospy.numpy_msg import numpy_msg
from tensorflow.contrib.slim.python.slim.nets import vgg
from devine_image_processing.msg import VGG16Features
from devine_config import topicname
from devine_common import ros_utils
from ros_image_processor import ImageProcessor, ROSImageProcessingWrapper

VGG16_NTW_PATH = ros_utils.get_fullpath(__file__, '../../vgg_16.ckpt')

FEATURES_TOPIC = topicname('image_features')
IMAGE_TOPIC = topicname('features_extraction_image')

IMAGE_SIZE = 224
CHANNEL_MEAN = np.array([123.68, 116.779, 103.939])


class ROSVgg16(ImageProcessor):
    """ VGG-16 wrapper for ROS """

    def __init__(self):
        self.holder = tf.placeholder(tf.float32,
                                     [None, IMAGE_SIZE, IMAGE_SIZE,
                                         len(CHANNEL_MEAN)],
                                     name='image')
        _, self.end_points = vgg.vgg_16(
            self.holder, is_training=False, dropout_keep_prob=1.0)

        tf_config = tf.ConfigProto(log_device_placement=False)
        tf_config.gpu_options.allow_growth = True

        self.sess = tf.Session(config=tf_config)
        rospy.on_shutdown(self.sess.close)

        saver = tf.train.Saver()
        saver.restore(self.sess, VGG16_NTW_PATH)

    def process(self, image, image_payload):
        """ Process a single image """
        img = cv2.resize(image, dsize=(IMAGE_SIZE, IMAGE_SIZE),
                         interpolation=cv2.INTER_LINEAR)
        img = img.astype(np.float32)
        img -= CHANNEL_MEAN[None, None, :]
        feat = self.sess.run(
            self.end_points['vgg_16/fc8'], feed_dict={self.holder: np.array([img])})
        return VGG16Features(header=image_payload.header, data=feat[0])


def main():
    """ Entry point of this file """
    processor = ROSImageProcessingWrapper(ROSVgg16, IMAGE_TOPIC)
    publisher = rospy.Publisher(
        FEATURES_TOPIC, numpy_msg(VGG16Features), queue_size=1)
    processor.loop(
        lambda processor_output: publisher.publish(processor_output))


if __name__ == '__main__':
    main()
