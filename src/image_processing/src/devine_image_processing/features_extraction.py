#!/usr/bin/env python3
""" ROS node for features extraction """

from queue import Queue, Empty
from io import BytesIO
from PIL import Image
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import CompressedImage
from devine_config import topicname
from devine_common import ros_utils
from tensorflow.contrib.slim.python.slim.nets import vgg
import tensorflow as tf
import numpy as np
import rospy

VGG16_NTW_PATH = ros_utils.get_fullpath(__file__, '../../vgg_16.ckpt')

FEATURES_TOPIC = topicname('image_features')
IMAGE_TOPIC = topicname('features_extraction_image')

IMAGE_SIZE = 224
CHANNEL_MEAN = np.array([123.68, 116.779, 103.939])

FEATURES_PUB = rospy.Publisher(FEATURES_TOPIC, Float64MultiArray, queue_size=1, latch=True)


def proccess_frame(img, sess, end_points, holder):
    """ Process a frame when a new image gets in the queue """
    rospy.loginfo('Processing frame')
    img = Image.open(BytesIO(img)).convert('RGB')
    img = img.resize((IMAGE_SIZE, IMAGE_SIZE), resample=Image.BILINEAR)
    img = np.array(img, dtype=np.float32)
    img -= CHANNEL_MEAN[None, None, :]

    feat = sess.run(end_points['vgg_16/fc8'], feed_dict={holder: np.array([img])})

    FEATURES_PUB.publish(Float64MultiArray(data=feat[0].tolist()))
    rospy.loginfo('Frame processed')


def main():
    """ Extract features from images """
    rospy.init_node('features_extraction')
    image_queue = Queue(2)

    def image_callback(data):
        """ Callback for image topic """
        if image_queue.full():
            rospy.logwarn('features_extraction: image receiving rate is too high !')
            image_queue.get()
        image_queue.put(data.data)

    rospy.Subscriber(IMAGE_TOPIC, CompressedImage, image_callback)

    holder = tf.placeholder(tf.float32,
                            [None, IMAGE_SIZE, IMAGE_SIZE, len(CHANNEL_MEAN)],
                            name='image')
    _, end_points = vgg.vgg_16(holder, is_training=False, dropout_keep_prob=1.0)

    tf_config = tf.ConfigProto(log_device_placement=False)
    tf_config.gpu_options.allow_growth = True

    with tf.Session(config=tf_config) as sess:
        saver = tf.train.Saver()
        saver.restore(sess, VGG16_NTW_PATH)

        while not rospy.is_shutdown():
            try:
                proccess_frame(image_queue.get(timeout=1), sess, end_points, holder)
            except Empty:
                pass


if __name__ == '__main__':
    main()
