#!/usr/bin/env python3
""" ROS module for image_segmentation """

import sys
import os
import datetime
import rospy
from std_msgs.msg import String
from bson import json_util
from devine_common import ros_utils

sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../Mask_RCNN'))

import coco
import model as modellib
import tensorflow as tf
from keras.backend.tensorflow_backend import set_session
from devine_config import topicname
from ros_image_processor import ImageProcessor, ROSImageProcessingWrapper

# Paths
COCO_MODEL_PATH = ros_utils.get_fullpath(__file__, '../../mask_rcnn_coco.h5')
MODEL_DIR = ros_utils.get_fullpath(__file__, 'logs')

# Topics
IMAGE_TOPIC = topicname('segmentation_image')
SEGMENTATION_TOPIC = topicname('objects')

SEGMENTATION_THRESHOLD = 0.8

class RCNNSegmentation(ImageProcessor):
    """ RCNN segmentation wrapper of Mask_RCNN for use in guesswhat """
    class_names = ['BG', 'person', 'bicycle', 'car', 'motorcycle', 'airplane',
                   'bus', 'train', 'truck', 'boat', 'traffic light',
                   'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird',
                   'cat', 'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear',
                   'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie',
                   'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball',
                   'kite', 'baseball bat', 'baseball glove', 'skateboard',
                   'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup',
                   'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
                   'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza',
                   'donut', 'cake', 'chair', 'couch', 'potted plant', 'bed',
                   'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote',
                   'keyboard', 'cell phone', 'microwave', 'oven', 'toaster',
                   'sink', 'refrigerator', 'book', 'clock', 'vase', 'scissors',
                   'teddy bear', 'hair drier', 'toothbrush']

    class InferenceConfig(coco.CocoConfig):
        """ Configuration class """
        GPU_COUNT = 1
        IMAGES_PER_GPU = 1

    def __init__(self):
        self.config = self.InferenceConfig()
        config = tf.ConfigProto(log_device_placement=True)
        config.gpu_options.allow_growth = True
        set_session(tf.Session(config=config))
        self.model = modellib.MaskRCNN(mode='inference', model_dir=MODEL_DIR, config=self.config)
        self.model.load_weights(COCO_MODEL_PATH, by_name=True)

    def process(self, image, _):
        """ Actual segmentation of the image """
        rospy.logdebug('Starting segmentation')
        height, width, _ = image.shape
        timestamp = '{:%Y-%b-%d %H:%M:%S}'.format(datetime.datetime.now())
        results = self.model.detect([image])

        if len(results) != 1:
            rospy.logerr('Couldn\'t segment image')
            return '{}'

        rospy.logdebug('Segmentation done')
        result = results[0]
        result_obj = {
            'timestamp': timestamp,
            'image': {
                'height': height,
                'width': width
            },
            'objects': []
        }
        object_array = []

        for current_id, (class_id, bounding_box, score, mask) in enumerate(zip(result['class_ids'], result['rois'], result['scores'], result['masks'])):
            if score < SEGMENTATION_THRESHOLD:
                continue
            current_object = {
                'category_id': int(class_id),
                'bbox': bounding_box.tolist(),
                'category': self.class_names[class_id],
                'segment': [],
                'id' : current_id,
                'area': mask.tolist()
            }
            object_array.append(current_object)

        result_obj['objects'] = object_array
        return json_util.dumps(result_obj)

def main():
    """ Entry point of this file """
    processor = ROSImageProcessingWrapper(RCNNSegmentation, IMAGE_TOPIC)
    publisher = rospy.Publisher(SEGMENTATION_TOPIC, String, queue_size=10, latch=False)
    processor.loop(lambda processor_output : publisher.publish(processor_output))

if __name__ == '__main__':
    main()
