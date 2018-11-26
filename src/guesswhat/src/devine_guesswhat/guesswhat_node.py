#!/usr/bin/env python3
# -*- coding: utf-8 -*-
""" ROS node for guesswhat """

import json
from queue import Queue, Empty
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
import tensorflow as tf
import numpy as np

from guesswhat.models.guesser.guesser_network import GuesserNetwork
from guesswhat.models.qgen.qgen_lstm_network import QGenNetworkLSTM
from guesswhat.models.qgen.qgen_wrapper import QGenWrapper
from guesswhat.models.looper.basic_looper import BasicLooper
from guesswhat.data_provider.guesswhat_dataset import Game
from guesswhat.data_provider.guesswhat_tokenizer import GWTokenizer
from guesswhat.data_provider.questioner_batchifier import QuestionerBatchifier

from modelwrappers import GuesserROSWrapper, OracleROSWrapper
from devine_config import topicname
from devine_common import ros_utils
from devine_image_processing.msg import SegmentedImage, VGG16Features

EVAL_CONF_PATH = ros_utils.get_fullpath(__file__, '../../config/eval.json')
GUESS_CONF_PATH = ros_utils.get_fullpath(__file__, '../../config/guesser.json')
QGEN_CONF_PATH = ros_utils.get_fullpath(__file__, '../../config/qgen.json')
GUESS_NTW_PATH = ros_utils.get_fullpath(__file__, '../../data/guesser.ckpt')
QGEN_NTW_PATH = ros_utils.get_fullpath(__file__, '../../data/qgen.ckpt')
TOKENS_PATH = ros_utils.get_fullpath(__file__, '../../data/tokens.json')

# topics
SEGMENTATION_TOPIC = topicname('objects')
FEATURES_TOPIC = topicname('image_features')
STATUS_TOPIC = topicname('guesswhat_status')
# TODO: merge these topics to one
OBJECT_TOPIC = topicname('guess_location_image')
CATEGORY_TOPIC = topicname('guess_category')


class ImgFeaturesLoader(object):
    """ Loads image from memory """

    def __init__(self, data):
        self.data = data

    def get_image(self, *_args, **_kwargs):
        """ get_image interface """
        return self.data


class ImgFeaturesBuilder(object):
    """ Builds the image loader """

    def __init__(self, data):
        self.data = data

    def build(self, *_args, **_kwargs):
        """build interface"""
        return ImgFeaturesLoader(self.data)


class SingleGameIterator(object):
    """ Single game iterator """

    def __init__(self, tokenizer, game):
        self.n_examples = 1
        #self.batchifier = LooperBatchifier(tokenizer, generate_new_games=False)
        self.batchifier = QuestionerBatchifier(tokenizer, None)
        self.game = game

    def __iter__(self):
        """ Applies batching (and transforms into dict) """
        # Futurely consider yielding games from the queue for the same looper
        return iter([self.batchifier.apply([self.game])])


class GuessWhatNode(object):
    """ The GuessWhat node """

    def __init__(self):
        self.segmentations = Queue(1)
        self.features = Queue(1)

        rospy.Subscriber(SEGMENTATION_TOPIC, SegmentedImage,
                         self._segmentation_callback)
        rospy.Subscriber(FEATURES_TOPIC, VGG16Features,
                         self._features_callback)
        self.object_found = rospy.Publisher(
            OBJECT_TOPIC, PointStamped, queue_size=1)
        self.category = rospy.Publisher(CATEGORY_TOPIC, String, queue_size=1)
        self.status = rospy.Publisher(
            STATUS_TOPIC, String, queue_size=1, latch=True)

        self.eval_config = self._load_config(EVAL_CONF_PATH)
        self.guesser_config = self._load_config(GUESS_CONF_PATH)
        self.qgen_config = self._load_config(QGEN_CONF_PATH)

        self.tokenizer = GWTokenizer(TOKENS_PATH)

        self.tf_config = tf.ConfigProto(log_device_placement=False)
        self.tf_config.gpu_options.allow_growth = True

        self.image_dim = ros_utils.get_image_dim()

    def start_session(self):
        """ Launch the tensorflow session and start the GuessWhat loop """
        with tf.Session(config=self.tf_config) as sess:
            guesser_network = GuesserNetwork(
                self.guesser_config['model'], num_words=self.tokenizer.no_words)
            guesser_var = [v for v in tf.global_variables()
                           if 'guesser' in v.name]
            guesser_saver = tf.train.Saver(var_list=guesser_var)
            guesser_saver.restore(sess, GUESS_NTW_PATH)
            guesser_wrapper = GuesserROSWrapper(guesser_network)

            qgen_network = QGenNetworkLSTM(self.qgen_config['model'],
                                           num_words=self.tokenizer.no_words,
                                           policy_gradient=False)
            qgen_var = [v for v in tf.global_variables() if 'qgen' in v.name]
            qgen_saver = tf.train.Saver(var_list=qgen_var)
            qgen_saver.restore(sess, QGEN_NTW_PATH)
            qgen_network.build_sampling_graph(self.qgen_config['model'],
                                              tokenizer=self.tokenizer,
                                              max_length=self.eval_config['loop']['max_depth'])
            qgen_wrapper = QGenWrapper(qgen_network, self.tokenizer,
                                       max_length=self.eval_config['loop']['max_depth'],
                                       k_best=self.eval_config['loop']['beam_k_best'])

            oracle_wrapper = OracleROSWrapper(self.tokenizer)

            self.loop(sess, guesser_wrapper, qgen_wrapper, oracle_wrapper)

    def segmented_image_to_img_obj(self, id_seg):
        """ Transform a segmented image obj to the image object expected for GW """
        (index, segmented_image) = id_seg
        bbox = segmented_image.bounding_box
        mask = np.array(segmented_image.mask_array).reshape(
            (segmented_image.mask_height, segmented_image.mask_width))
        return {
            'id': index,
            'category': segmented_image.category_name,
            'category_id': segmented_image.category_id,
            'bbox': [bbox.x_offset, bbox.y_offset, bbox.width, bbox.height],
            'area': mask,
            'segment': None
        }

    def loop(self, sess, guesser_wrapper, qgen_wrapper, oracle_wrapper):
        """ The GuessWhat game loop """
        wait_for_img_status_published = False
        while not rospy.is_shutdown():
            if not wait_for_img_status_published:
                self.status.publish('Waiting for image processing')
                wait_for_img_status_published = True
            try:
                seg = self.segmentations.get(timeout=1)
                feats = self.features.get(timeout=1)
                wait_for_img_status_published = False
            except Empty:
                continue

            self.status.publish('Starting new game')

            objects = list(map(self.segmented_image_to_img_obj,
                               enumerate(seg.objects)))
            game = Game(id=0,
                        object_id=0,
                        objects=objects,
                        qas=[],
                        image={'id': 0, 'width': self.image_dim[0],
                               'height': self.image_dim[1], 'coco_url': ''},
                        status='false',
                        which_set=None,
                        image_builder=ImgFeaturesBuilder(feats),
                        crop_builder=None)

            looper = BasicLooper(self.eval_config,
                                 guesser_wrapper=guesser_wrapper,
                                 qgen_wrapper=qgen_wrapper,
                                 oracle_wrapper=oracle_wrapper,
                                 tokenizer=self.tokenizer,
                                 batch_size=1)

            iterator = SingleGameIterator(self.tokenizer, game)
            looper.process(sess, iterator,
                           mode='greedy', store_games=True) #beam_search, sampling or greedy

            self._resolve_choice(seg.header, looper)

    def _load_config(self, path):
        """ Reads json config """
        with open(path) as conf:
            return json.load(conf)

    def _segmentation_callback(self, data):
        """ Callback for the segmantion topic """
        if self.segmentations.full():
            self.segmentations.get()

        self.segmentations.put(data)

    def _features_callback(self, data):
        """ Callback for the features topic """
        if self.features.full():
            self.features.get()

        self.features.put(np.array(data.data))

    def _resolve_choice(self, msg_header, looper):
        """ Resolve what object GuessWhat chose """
        storage = looper.storage[0]
        choice = next(obj for obj in storage['game'].objects
                      if obj.id == storage['guess_object_id'])

        object_location = PointStamped()
        object_location.header = msg_header
        object_location.point.x = int(choice.bbox.x_center)
        object_location.point.y = int(choice.bbox.y_center)
        self.object_found.publish(object_location)
        self.status.publish('Object guessed')
        self.category.publish(choice.category)


def main():
    """ Entry point of this file """
    rospy.init_node('guesswhat')
    node = GuessWhatNode()
    node.start_session()

if __name__ == '__main__':
    main()
