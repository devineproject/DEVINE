#!/usr/bin/env python3
'''ROS node for features extraction'''

import sys
import os
from queue import Queue, Empty
from io import BytesIO

import tensorflow as tf
from tensorflow.contrib.slim.python.slim.nets import vgg
from PIL import Image
import numpy as np

import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import CompressedImage

ROOT_DIR = sys.path[0]
VGG16_NTW_PATH = os.path.join(ROOT_DIR, '../../vgg_16.ckpt')

FEATURES_TOPIC = '/vgg16_features'
IMAGE_TOPIC = '/devine/image'

IMAGE_SIZE = 224
CHANNEL_MEAN = np.array([123.68, 116.779, 103.939])

if __name__ == '__main__':
    rospy.init_node('features_extraction')
    features = rospy.Publisher(FEATURES_TOPIC, Float64MultiArray, queue_size=1)

    image_queue = Queue(2)

    def image_callback(data):
        '''Callback for image topic'''
        if image_queue.full():
            rospy.logwarn('features_extraction: image receiving rate is too high !')
            image_queue.get()
        image_queue.put(data.data)

    rospy.Subscriber(IMAGE_TOPIC, CompressedImage, image_callback)

    holder = tf.placeholder(tf.float32,
                            [None, IMAGE_SIZE, IMAGE_SIZE, len(CHANNEL_MEAN)],
                            name='image')
    _, end_points = vgg.vgg_16(holder, is_training=False, dropout_keep_prob=1.0)

    with tf.Session() as sess:
        saver = tf.train.Saver()
        saver.restore(sess, VGG16_NTW_PATH)

        while not rospy.is_shutdown():
            try:
                img = image_queue.get(timeout=1)

                rospy.loginfo('Processing frame')
                img = Image.open(BytesIO(img))
                img = img.resize((IMAGE_SIZE, IMAGE_SIZE), resample=Image.BILINEAR)
                img = np.array(img, dtype=np.float32)
                img -= channel[None, None, :]

                ft_output = end_points['vgg_16/fc8']
                feat = sess.run(ft_output, feed_dict={holder: np.array([img])})

                features.publish(Float64MultiArray(data=feat[0].tolist()))
                rospy.loginfo('Frame processed')
            except Empty:
                pass
