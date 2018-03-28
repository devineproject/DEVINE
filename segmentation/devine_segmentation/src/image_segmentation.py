#!/usr/bin/env python3
'''ROS module for image_segmentation'''

import sys
import os
import datetime
import queue
#import pickle

import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

import numpy as np
import skimage.io
from bson import json_util

sys.path.append(os.path.join(sys.path[0], '../../Mask_RCNN'))
import coco
import model as modellib

#paths
ROOT_DIR = sys.path[0]
COCO_MODEL_PATH = os.path.join(ROOT_DIR, "../../mask_rcnn_coco.h5")
MODEL_DIR = os.path.join(ROOT_DIR, "logs")

#topics
IMAGE_TOPIC = '/camera/rgb/image_color/compressed'
SEGMENTATION_TOPIC = '/rcnn_segmentation'

class RCNNSegmentation(object):
    '''RCNN segmentation wrapper of Mask_RCNN for use in guesswhat'''
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
        '''Configuration class'''
        GPU_COUNT = 1
        IMAGES_PER_GPU = 1

    def __init__(self):
        self.config = self.InferenceConfig()
        #config.display()
        self.model = modellib.MaskRCNN(mode="inference", model_dir=MODEL_DIR, config=self.config)
        self.model.load_weights(COCO_MODEL_PATH, by_name=True) #blocking I/O in constructor!

    def correct_array(self, bbox, im_height):
        '''correct_array'''
        correct_boxes = np.zeros(bbox.shape)
        for counter, (original_array, new_array) in enumerate(zip(bbox, correct_boxes)):
            width = original_array[3] - original_array[1]
            height = original_array[2] - original_array[0]
            left = original_array[1]
            top = im_height - original_array[2]
            correct_boxes[counter][:] = np.array([left, top, height, width])
        return correct_boxes

    def segment(self, img):
        '''Actual segmentation of the image'''
        #TODO: Don't use disk i/o...
        image_name = '/tmp/segmentedimage.jpg'
        image_file = open(image_name, 'wb')
        image_file.write(img)
        image = skimage.io.imread(image_name)
        height, width, depth = image.shape
        results = self.model.detect([image])
        result1 = results[0]
        result_obj = {
            "status": "success" if result1 else "failure",
            "qas": [],
            "questioner_id": 2,
            "timestamp": '{:%Y-%b-%d %H:%M:%S}'.format(datetime.datetime.now()),
            "image": {
                "file_name": image_name,
                "coco_url": None,
                "height": height,
                "width": width,
                "flickr_url": None,
                "id": 1
            },
            "objects": [],
            "object_id": 0,
            "id": 1
        }
        corrected_bounding_box = self.correct_array(result1['rois'], height)
        object_array = []

        # Debug file dump
        # with open('id.pkl', 'wb') as pickle_file:
        #     pickle.dump(result1['class_ids'], pickle_file)
        # with open('scores.pkl', 'wb') as pickle_file:
        #     pickle.dump(result1['scores'], pickle_file)
        # with open('masks.pkl', 'wb') as pickle_file:
        #     pickle.dump(result1['masks'], pickle_file)
        # with open('rois.pkl','wb') as pickle_file:
        #     result1['rois'].dump(pickle_file)

        for current_id, (class_id, bounding_box) in enumerate(zip(result1['class_ids'], corrected_bounding_box)):
            object_area = 0 # figure out if we ever use the area
            # According the MsCoco api this seems to be the mask area
            # (check if maskrcnn can produce this)
            current_object = {
                "category_id": int(class_id),
                "bbox": bounding_box.tolist(),
                "category": self.class_names[class_id],
                "segment": [],
                "id" : current_id,
                "area": object_area
            }
            object_array.append(current_object)

            # Currently there is an issue with Numpy integers
            # (probably orginating from the bounding box)
            # Also make sure the bounding box being created is correct

        result_obj['objects'] = object_array
        return json_util.dumps(result_obj)

class ROSRCNNSegmentation(RCNNSegmentation):
    '''ROS Wrapper of RCNN Segmentation'''
    image_queue = queue.Queue(2)

    def __init__(self):
        super(ROSRCNNSegmentation, self).__init__()
        rospy.init_node('image_segmentation')
        rospy.Subscriber(IMAGE_TOPIC, CompressedImage,
                         self.image_received_callback, queue_size=2)
        self.publisher = rospy.Publisher(SEGMENTATION_TOPIC, String, queue_size=10)

    def image_received_callback(self, data):
        '''Callback when a new image is received from the topic'''
        if self.image_queue.full():
            self.image_queue.get()
        self.image_queue.put(data.data)
        rospy.sleep(10.) #in seconds

    def loop(self):
        '''Looping method to segment every image'''
        rate = rospy.Rate(0.1) #0.1 Hz publishing rate
        while True:
            img = self.image_queue.get() #blocking
            json = self.segment(img)
            self.publisher.publish(json)
            rate.sleep()

if __name__ == '__main__':
    ros_segmentation = ROSRCNNSegmentation()
    ros_segmentation.loop()
