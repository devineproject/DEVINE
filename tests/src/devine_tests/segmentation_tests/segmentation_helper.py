#! /usr/bin/env python2
""" Test to validate segmentation rates """
import json
import rospy
import utils
from collections import Counter
from sensor_msgs.msg import CompressedImage
from devine_config import topicname

IMAGE_TOPIC = topicname("segmentation_image")
SEGMENTATION_IMAGE_TOPIC = topicname("objects")

IMAGE_PUB = rospy.Publisher(IMAGE_TOPIC, CompressedImage, queue_size=1)

IMAGE_MSG = "image_msg"
EXPECTED_OBJECTS = "expected_objects"

def load_test_images(test_filepath):
    """ Loads test data and images"""
    test_images = []
    with open(test_filepath) as json_data:
        file_data = json.load(json_data)

    for test in file_data["tests"]:
        test_images.append({
            IMAGE_MSG: utils.image_file_to_ros_msg(test["imageName"]),
            EXPECTED_OBJECTS: test["refData"]["objects"]
        })

    return test_images


def get_segmented_objets(data):
    """ Extract a list of the segmented objects found in the given ros data """
    json_data = json.loads(data.data)
    return [object["category"] for object in json_data["objects"]]


def get_missed_objects(expected_objects, actual_objects):
    """ Return the list of all objects that were not detected """
    return list((Counter(expected_objects) - Counter(actual_objects)).elements())
