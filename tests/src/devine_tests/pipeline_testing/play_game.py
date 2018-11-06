#! /usr/bin/env python2
"""import mscoco images to validate pipeline"""

import argparse
import random
import rospy
from devine_config import topicname
import cv2
from sensor_msgs.msg import CompressedImage
import numpy as np
from time import sleep
import os
from std_msgs.msg import String
from devine_common.image_utils import image_to_ros_msg


FEATURE_IMAGE_TOPIC = topicname('features_extraction_image')
SEGMENTATION_IMAGE_TOPIC = topicname('segmentation_image')
RESTART_TOPIC = topicname('end_of_game')
TTS_ANSWER_TOPIC = topicname('tts_answer')
TTS_QUERY_TOPIC = topicname('tts_query')
CATEGORY_TOPIC = topicname('guess_category')


class ImageTest:
    """ Test class that loads images and runs a game of GW?! """

    def __init__(self, image_folder):
        self.image_folder = image_folder
        self.outputs = []
        self.image = None
        rospy.init_node('mscoco_test')
        self.rate = rospy.Rate(1)
        self.image_feat_pub = rospy.Publisher(FEATURE_IMAGE_TOPIC, CompressedImage, queue_size=1)
        self.image_seg_pub = rospy.Publisher(SEGMENTATION_IMAGE_TOPIC, CompressedImage, queue_size=1)
        sleep(1)

    def load_test_image(self, image_name=None):
        """ Loads test data and images """
        if image_name is None:
            image_list = os.listdir(self.image_folder)
            random.shuffle(image_list)
            image_name = image_list[0]
        image_path = os.path.join(self.image_folder, image_name)
        self.image = cv2.imread(image_path, cv2.IMREAD_COLOR)
        self.outputs = []

    def _send_compressed_image(self, image):
        """ Sends compressed image to feature and seg nodes """
        msg = image_to_ros_msg(image)
        self.image_seg_pub.publish(msg)
        self.image_feat_pub.publish(msg)

    def play_game(self, image=np.array([])):
        """ Plays a game of GW?! """
        if len(image) is 0:
            image = self.image

        self._send_compressed_image(image)
        guess = rospy.wait_for_message(CATEGORY_TOPIC, String)
        self.outputs.append(guess)

    def crop_image(self, x1, y1, width, height):
        """ Crops an image """
        return self.image[y1: y1 + height, x1:x1 + width]

    def mask_image(self, x1, y1, width, height):
        """ Masks around a cropped image """
        mask = np.zeros(self.image.shape).astype(np.uint8)
        print(np.array(self.crop_image(x1, y1, width, height)).shape)
        mask[y1: y1+height, x1: x1 + width, :] = np.asarray(self.crop_image(x1, y1, width, height), dtype='uint8')
        return mask

    def mask_test(self, crop_array):
        """ Plays a game with a masked image """
        [x1, y1, w, h] = crop_array
        return self.play_game(self.mask_image(x1, y1, w, h))

    def crop_test(self, crop_array):
        """ Plays a game with a cropped image """
        [x1, y1, w, h] = crop_array
        return self.play_game(self.crop_image(x1, y1, w, h))

    def image_test(self, crop_array=None, image_name=None):
        """ Runs a test on an image  """
        self.load_test_image(image_name)
        if crop_array == None:
            self.play_game()
        else:
            self.play_game()
            self.crop_test(crop_array)
            self.mask_test(crop_array)


def main(args):
    """ Plays n games and displays their outputs  """
    # Parse arguments
    # print(args.crop_array)
    test = ImageTest(args.dataset)
    for i in range(0, int(args.num)):
        test.image_test(args.crop_array, args.image_name)
        print(test.outputs)


def parser():
    """ Command Line Parser """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    arg_parser = argparse.ArgumentParser(formatter_class=arg_fmt, description=main.__doc__)
    image_options = arg_parser.add_argument_group('Image options')
    image_options.add_argument('-d', '--dataset', required=True, help='Where is the data set you want to play with')
    # turn this into an option which will load the entire dataset
    image_options.add_argument('-n', '--num', required=True, help='How many games to play')
    image_options.add_argument('-c', '--crop_array', nargs='+', type=int,  default=None, help='Crop and mask sizes')
    image_options.add_argument('-i', '--image_name', type=str, default=None, help='Name of image to be used')
    arguments = arg_parser.parse_args(rospy.myargv()[1:])
    return arguments


if __name__ == '__main__':
    main(parser())
