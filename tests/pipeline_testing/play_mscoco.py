#! /usr/bin/env python2
'''import mscoco images to validate pipeline'''

import argparse
import random
import rospy
from devine_config import topicname
from Queue import Queue, Empty
import json
import cv2
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Bool
from devine_dialog.msg import TtsQuery
from devine_dialog import TTSAnswerType, send_speech
from time import sleep
import os
# How to load images from mscoco

FEATURE_IMAGE_TOPIC = topicname('features_extraction_image')
SEGMENTATION_IMAGE_TOPIC = topicname('segmentation_image')
RESTART_TOPIC = topicname('end_of_game')
TTS_ANSWER_TOPIC = topicname('tts_answer')
TTS_QUERY_TOPIC = topicname('tts_query')
CATEGORY_TOPIC = topicname('guess_category')

class MSCOCOTest:
    ''' Test class that loads images and runs a game of GW?! '''
    def __init__(self,image_folder):
        self.image_folder = image_folder
        self.image = None 
        rospy.init_node('mscoco_test')
        self.rate = rospy.Rate(1)
        self.image_feat_pub = rospy.Publisher(FEATURE_IMAGE_TOPIC, CompressedImage, queue_size=1)
        self.image_seg_pub = rospy.Publisher(SEGMENTATION_IMAGE_TOPIC, CompressedImage, queue_size=1)
        sleep(1)
        
    def _load_test_image(self, image_name):
        """ Loads test data and images """
        if image_name is None:
            image_list = os.listdir(self.image_folder)
            random.shuffle(image_list)
            image_name = image_list[0]
        image_path = os.path.join(self.image_folder,image_name)
        self.image = cv2.imread(image_path,cv2.IMREAD_COLOR)

    def _send_compressed_image(self):
        ''' Sends compressed image to feature and seg nodes '''
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "png"
        msg.data = np.array(cv2.imencode('.png', self.image)[1]).tostring()
        self.image_seg_pub.publish(msg) 
        self.image_feat_pub.publish(msg)
        
    def play_game(self,image_name = None):
        ''' Plays a game of GW?! '''
        self._load_test_image(image_name)
        self._send_compressed_image()
        guess = rospy.wait_for_message(CATEGORY_TOPIC, String)
        return guess
        
def main(args):
    ''' Plays n games and their outputs  '''
    # Parse arguments
    test = MSCOCOTest(args.dataset)
    for i in range(0,int(args.num)):
         print(test.play_game())

 
def parser():
    ''' Command Line Parser'''
    arg_fmt = argparse.RawDescriptionHelpFormatter
    arg_parser = argparse.ArgumentParser(formatter_class=arg_fmt, description=main.__doc__)
    image_options = arg_parser.add_argument_group('Image options')
    image_options.add_argument('-d', '--dataset', required=True, help='Where is the data set you want to play with')
    image_options.add_argument('-n', '--num', required=True, help='How many games to play') # turn this into an option which will load the entire dataset
    arguments = arg_parser.parse_args(rospy.myargv()[1:])
    return arguments

if __name__ == '__main__':
    main(parser())