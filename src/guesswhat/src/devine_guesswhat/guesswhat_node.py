#!/usr/bin/env python3
""" ROS node for guesswhat """

import json
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
from devine_config import topicname
from devine_common import ros_utils

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


class ImgFeaturesLoader():
    """ Loads image from memory """

    def __init__(self, data):
        self.data = data

    def get_image(self, *_args, **_kwargs):
        """ get_image interface """
        return self.data


class ImgFeaturesBuilder():
    """ Builds the image loader """

    def __init__(self, data):
        self.data = data

    def build(self, *_args, **_kwargs):
        """build interface"""
        return ImgFeaturesLoader(self.data)


class SingleGameIterator():
    """ Single game iterator """

    def __init__(self, tokenizer, game):
        self.n_examples = 1
        self.batchifier = LooperBatchifier(tokenizer, generate_new_games=False)
        self.game = game

    def __iter__(self):
        """ Applies batching (and transforms into dict) """
        # Futurely consider yielding games from the queue for the same looper
        return iter([self.batchifier.apply([self.game])])


class GuessWhatNode():
    """ The GuessWhat node """

    def __init__(self):
        self.segmentations = Queue(1)
        self.features = Queue(1)

        rospy.Subscriber(SEGMENTATION_TOPIC, String, self._segmentation_callback)
        rospy.Subscriber(FEATURES_TOPIC, Float64MultiArray, self._features_callback)
        self.object_found = rospy.Publisher(OBJECT_TOPIC, Int32MultiArray, queue_size=1)
        self.category = rospy.Publisher(CATEGORY_TOPIC, String, queue_size=1)
        self.status = rospy.Publisher(STATUS_TOPIC, String, queue_size=1, latch=True)

        self.eval_config = self._load_config(EVAL_CONF_PATH)
        self.guesser_config = self._load_config(GUESS_CONF_PATH)
        self.qgen_config = self._load_config(QGEN_CONF_PATH)

        self.tokenizer = GWTokenizer(TOKENS_PATH)

        self.tf_config = tf.ConfigProto(log_device_placement=True)
        self.tf_config.gpu_options.allow_growth = True

    def start_session(self):
        """ Launch the tensorflow session and start the GuessWhat loop """
        with tf.Session(config=self.tf_config) as sess:
            guesser_network = GuesserNetwork(self.guesser_config['model'], num_words=self.tokenizer.no_words)
            guesser_var = [v for v in tf.global_variables() if 'guesser' in v.name]
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

            # batchifier = LooperBatchifier(tokenizer, generate_new_games=False)
            self.loop(sess, guesser_wrapper, qgen_wrapper, oracle_wrapper)

    def loop(self, sess, guesser_wrapper, qgen_wrapper, oracle_wrapper):
        """ The GuessWhat game loop """
        while not rospy.is_shutdown():
            self.status.publish('Waiting for image processing')
            try:
                seg = self.segmentations.get(timeout=1)
                feats = self.features.get(timeout=1)
            except Empty:
                continue

            self.status.publish('Starting new game')

            objects = []
            for obj in seg['objects']:
                # Adapting bounding box to GuessWhat
                bbox = obj['bbox']
                objects.append([bbox[1], bbox[0], bbox[3] - bbox[1], bbox[2] - bbox[0]])

            game = Game(id=0,
                        object_id=0,
                        objects=objects,
                        qas=[],
                        image={'id': 0, 'width': 640, 'height': 480, 'coco_url': ''},
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
            looper.process(sess, iterator, mode='beam_search', store_games=True)

            self._resolve_choice(looper)

    def _load_config(self, path):
        """ Reads json config """
        with open(path) as conf:
            return json.load(conf)

    def _segmentation_callback(self, data):
        """ Callback for the segmantion topic """
        if self.segmentations.full():
            self.segmentations.get()

        try:
            self.segmentations.put(json.loads(data.data))
        except json.JSONDecodeError:
            rospy.logerr('Garbage on %s, expects json', SEGMENTATION_TOPIC)

    def _features_callback(self, data):
        """ Callback for the features topic """
        if self.features.full():
            self.features.get()

        self.features.put(np.array(data.data))

    def _resolve_choice(self, looper):
        """ Resolve what object GuessWhat chose """
        storage = looper.storage[0]
        choice = next(obj for obj in storage['game'].objects
                      if obj.id == storage['guess_object_id'])

        self.object_found.publish(Int32MultiArray(data=[int(choice.bbox.x_center),
                                                        int(choice.bbox.y_center)]))
        self.status.publish('Object guessed')
        self.category.publish(choice.category)


def main():
    """ Entry point of this file """
    rospy.init_node('guesswhat')
    node = GuessWhatNode()
    node.start_session()


if __name__ == '__main__':
    main()
