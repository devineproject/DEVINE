#! /usr/bin/env python2
""" Test to validate segmentation rates """
from Queue import Queue, Empty
import argparse
import json
import os
import rospy
import cv2
import numpy as np
from collections import Counter
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from devine_config import topicname

IMAGE_TOPIC = topicname('segmentation_image')
SEGMENTATION_IMAGE_TOPIC = topicname('objects')

IMAGE_PUB = rospy.Publisher(IMAGE_TOPIC, CompressedImage, queue_size=1)

IMAGE_MSG = "image_msg"
EXPECTED_OBJECTS = "expected_objects"


def image_file_to_ros_msg(image_path):
    """ Convert an image file to a ros readable data message """
    img = cv2.imread(get_fullpath(image_path), cv2.IMREAD_COLOR)

    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "png"
    msg.data = np.array(cv2.imencode('.png', img)[1]).tostring()

    return msg


def load_test_images(test_filepath):
    """ Loads test data and images"""
    test_images = []
    with open(test_filepath) as json_data:
        file_data = json.load(json_data)

    for test in file_data["tests"]:
        test_images.append({
            IMAGE_MSG: image_file_to_ros_msg(test["imageName"]),
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


def test_segmentation_should_find_most_of_the_objects(test_image):
    """ Evaluates  segmentation rate for a single image """
    expected_objects = test_image[EXPECTED_OBJECTS]

    # send over node
    IMAGE_PUB.publish(test_image[IMAGE_MSG])
    # receive data
    data = rospy.wait_for_message(SEGMENTATION_IMAGE_TOPIC, String)
    rospy.logwarn("Message timestamp: %s", json.loads(data.data)['timestamp'])
    objects_found = get_segmented_objets(data)

    missed_objects = get_missed_objects(expected_objects, objects_found)
    other_found_objects = get_missed_objects(objects_found, expected_objects)

    rospy.loginfo("Number of missed detections: %d", len(missed_objects))
    rospy.loginfo("Number of other detected objects: %d", len(other_found_objects))
    objects_missed_ratio = float(len(missed_objects)) / len(expected_objects)
    rospy.loginfo("Percentage of objects missed: %.02f", objects_missed_ratio)
    if objects_missed_ratio >= 0.5:
        rospy.logwarn("The ratio of objects missed if greater than 50%.")


def get_fullpath(relative_file):
    """ Return the full path of a file in a directory relative to this one """
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), relative_file)


def main():
    """ Loads images and posts the corresponding segmentation rates """
    # Start ROS node
    rospy.init_node("image_seg_test")
    test_images = load_test_images(get_fullpath("test.json"))

    pos = 0
    while not rospy.is_shutdown() and pos < len(test_images):
        rospy.loginfo("### Test image %d ###", pos + 1)
        test_segmentation_should_find_most_of_the_objects(test_images[pos])
        pos += 1


if __name__ == "__main__":
    main()
