#!/usr/bin/env python3
'''ROS node for guesswhat'''

import json
import os
import sys
from queue import Queue, Empty

import rospy
from std_msgs.msg import String, Int32MultiArray, Float64MultiArray

import tensorflow as tf
import numpy as np

from guesswhat.models.guesser.guesser_network import GuesserNetwork
from guesswhat.models.qgen.qgen_lstm_network import QGenNetworkLSTM
from guesswhat.models.qgen.qgen_wrapper import QGenWrapper
from guesswhat.models.looper.basic_looper import BasicLooper
from guesswhat.data_provider.guesswhat_dataset import Game
from guesswhat.data_provider.guesswhat_tokenizer import GWTokenizer
from guesswhat.data_provider.looper_batchifier import LooperBatchifier

from modelwrappers import GuesserROSWrapper, OracleROSWrapper

ROOT_DIR = sys.path[0]
EVAL_CONF_PATH = os.path.join(ROOT_DIR, '../config/eval.json')
GUESS_CONF_PATH = os.path.join(ROOT_DIR, '../config/guesser.json')
QGEN_CONF_PATH = os.path.join(ROOT_DIR, '../config/qgen.json')
GUESS_NTW_PATH = os.path.join(ROOT_DIR, '../data/guesser.ckpt')
QGEN_NTW_PATH = os.path.join(ROOT_DIR, '../data/qgen.ckpt')
TOKENS_PATH = os.path.join(ROOT_DIR, '../data/tokens.json')

SEGMENTATION_TOPIC = '/rcnn_segmentation'
FEATURES_TOPIC = '/vgg16_features'
OBJECT_TOPIC = '/object_found'
CATEGORY_TOPIC = '/found_category'
STATE_TOPIC = '/guesswhat_state'

segmentations = Queue(1)
features = Queue(1)

class ImgFeaturesLoader():
    '''Loads image from memory'''
    def __init__(self, data):
        self.data = data

    def get_image(self, *args, **kwargs):
        '''get_image interface'''
        return self.data

class ImgFeaturesBuilder():
    '''Builds the image loader'''
    def __init__(self, data):
        self.data = data

    def build(self, *args, **kwargs):
        '''build interface'''
        return ImgFeaturesLoader(self.data)

class SingleGameIterator():
    '''Single game iterator'''
    def __init__(self, tokenizer, game):
        self.n_examples = 1
        self.batchifier = LooperBatchifier(tokenizer, generate_new_games=False)
        self.game = game

    def __iter__(self):
        '''Applies batching (and transforms into dict)'''
        # Futurely consider yielding games from the queue for the same looper
        return iter([self.batchifier.apply([self.game])])

def open_config(path):
    '''Reads json config'''
    with open(path) as conf:
        return json.load(conf)

def segmentation_callback(data):
    '''Callback for the segmantion topic'''
    if segmentations.full():
        segmentations.get()

    try:
        segmentations.put(json.loads(data.data))
    except json.JSONDecodeError:
        rospy.logerr('Garbage on {}, expects json'.format(SEGMENTATION_TOPIC))

def features_callback(data):
    '''Callback for the features topic'''
    if features.full():
        features.get()

    features.put(np.array(data.data))

if __name__ == '__main__':
    rospy.init_node('guesswhat')
    rospy.Subscriber(SEGMENTATION_TOPIC, String, segmentation_callback)
    rospy.Subscriber(FEATURES_TOPIC, Float64MultiArray, features_callback)
    object_found = rospy.Publisher(OBJECT_TOPIC, Int32MultiArray, queue_size=1)
    category = rospy.Publisher(CATEGORY_TOPIC, String, queue_size=1)
    state = rospy.Publisher(STATE_TOPIC, String, queue_size=1, latch=True)

    eval_config = open_config(EVAL_CONF_PATH)
    guesser_config = open_config(GUESS_CONF_PATH)
    qgen_config = open_config(QGEN_CONF_PATH)

    tokenizer = GWTokenizer(TOKENS_PATH)

    with tf.Session() as sess:
        guesser_network = GuesserNetwork(guesser_config['model'], num_words=tokenizer.no_words)
        guesser_var = [v for v in tf.global_variables() if 'guesser' in v.name]
        guesser_saver = tf.train.Saver(var_list=guesser_var)
        guesser_saver.restore(sess, GUESS_NTW_PATH)
        guesser_wrapper = GuesserROSWrapper(guesser_network)

        qgen_network = QGenNetworkLSTM(qgen_config['model'],
                                       num_words=tokenizer.no_words,
                                       policy_gradient=False)
        qgen_var = [v for v in tf.global_variables() if 'qgen' in v.name]
        qgen_saver = tf.train.Saver(var_list=qgen_var)
        qgen_saver.restore(sess, QGEN_NTW_PATH)
        qgen_network.build_sampling_graph(qgen_config['model'],
                                          tokenizer=tokenizer,
                                          max_length=eval_config['loop']['max_depth'])
        qgen_wrapper = QGenWrapper(qgen_network, tokenizer,
                                   max_length=eval_config['loop']['max_depth'],
                                   k_best=eval_config['loop']['beam_k_best'])

        oracle_wrapper = OracleROSWrapper(tokenizer)

        batchifier = LooperBatchifier(tokenizer, generate_new_games=False)

        state.publish('Waiting for image processing')
        while not rospy.is_shutdown():
            try:
                seg = segmentations.get(timeout=1)
                feats = features.get(timeout=1)
            except Empty:
                continue

            rospy.loginfo('Starting new game')
            img = {'id': 0, 'width': 640, 'height': 480, 'coco_url': ''}
            game = Game(id=0,
                        object_id=0,
                        objects=seg['objects'],
                        qas=[],
                        image=img,
                        status="false",
                        which_set=None,
                        image_builder=ImgFeaturesBuilder(feats),
                        crop_builder=None)

            looper = BasicLooper(eval_config,
                                 guesser_wrapper=guesser_wrapper,
                                 qgen_wrapper=qgen_wrapper,
                                 oracle_wrapper=oracle_wrapper,
                                 tokenizer=tokenizer,
                                 batch_size=1)

            iterator = SingleGameIterator(tokenizer, game)
            looper.process(sess, iterator, mode='greedy', store_games=True)

            storage = looper.storage[0]
            choice_index = storage['guess_object_index']
            choice_bbox = game.objects[choice_index].bbox
            object_found.publish(Int32MultiArray(data=[int(choice_bbox.x_center),
                                                       int(choice_bbox.y_center)]))
            rospy.loginfo('Game over')
            category.publish(seg['objects'][choice_index]['category'])

            state.publish('Waiting for image processing')
