#!/usr/bin/env python3
""" ROS module for image_segmentation """

import sys
import os
import datetime
import rospy
from std_msgs.msg import String
from bson import json_util
from devine_common import ros_utils
from cv_bridge import CvBridge

sys.path.append(os.path.join(os.path.dirname(
    os.path.realpath(__file__)), '../../Mask_RCNN'))

import coco
import model as modellib
import tensorflow as tf
from keras.backend.tensorflow_backend import set_session
from devine_config import topicname
from ros_image_processor import ImageProcessor, ROSImageProcessingWrapper
from devine_image_processing.msg import SegmentedImage, SceneObject

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
        config = tf.ConfigProto(log_device_placement=False)
        config.gpu_options.allow_growth = True
        set_session(tf.Session(config=config))
        self.model = modellib.MaskRCNN(
            mode='inference', model_dir=MODEL_DIR, config=self.config)
        self.model.load_weights(COCO_MODEL_PATH, by_name=True)
        self._bridge = CvBridge()

    def process(self, image, image_payload):
        """ Actual segmentation of the image """
        rospy.logdebug('Starting segmentation')
        results = self.model.detect([image])

        output = SegmentedImage()
        output.header = image_payload.header

        if len(results) != 1:
            rospy.logerr('Couldn\'t segment image')
            return output

        rospy.logdebug('Segmentation done')
        result = results[0]

        for (class_id, bounding_box, score, mask) in zip(result['class_ids'], result['rois'], result['scores'], result['masks']):
            if score < SEGMENTATION_THRESHOLD:
                rospy.logdebug(
                    'Discarding object "%s" because of score: %i', self.class_names[class_id], score)
                continue

            scene_object = SceneObject()
            scene_object.category_id = int(class_id)
            scene_object.category_name = self.class_names[class_id]
            [top, left, bottom, right] = bounding_box
            scene_object.bounding_box.x_offset = left
            scene_object.bounding_box.y_offset = top
            scene_object.bounding_box.height = bottom - top
            scene_object.bounding_box.width = right - left
            scene_object.mask = self._bridge.cv2_to_imgmsg(
                mask, encoding="passthrough")
            scene_object.mask_array = mask.flatten().tolist()
            (scene_object.mask_height, scene_object.mask_width) = mask.shape
            scene_object.score = score
            output.objects.append(scene_object)

        return output


def main():
    """ Entry point of this file """
    processor = ROSImageProcessingWrapper(RCNNSegmentation, IMAGE_TOPIC)
    publisher = rospy.Publisher(
        SEGMENTATION_TOPIC, SegmentedImage, queue_size=1)
    processor.loop(
        lambda processor_output: publisher.publish(processor_output))


if __name__ == '__main__':
    main()
