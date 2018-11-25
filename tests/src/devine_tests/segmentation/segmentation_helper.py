__author__ = "Jordan Prince Tremblay, Ismael Balafrej, Felix Labelle, Félix Martel-Denis, Eric Matte, Adam Letourneau, Julien Chouinard-Beaupré, Antoine Mercier-Nicol"
__copyright__ = "Copyright 2018, DEVINE Project"
__credits__ = ["Simon Brodeur", "François Ferland", "Jean Rouat"]
__license__ = "BSD"
__version__ = "1.0.0"
__email__ = "devine.gegi-request@listes.usherbrooke.ca"
__status__ = "Production"

import json
from collections import Counter
from sensor_msgs.msg import CompressedImage
from devine_config import topicname
import rospy
from devine_common import image_utils, ros_utils
from devine_image_processing.msg import SegmentedImage

IMAGE_TOPIC = topicname("segmentation_image")
SEGMENTATION_IMAGE_TOPIC = topicname("objects")

IMAGE_PUB = rospy.Publisher(IMAGE_TOPIC, CompressedImage, queue_size=1, latch=True)

IMAGE_MSG = "image_msg"
FILENAME = "filename"
EXPECTED_OBJECTS = "expected_objects"
LAST_STAMP = None


def segment_image(image):
    global LAST_STAMP
    # send over node
    IMAGE_PUB.publish(image[IMAGE_MSG])
    # receive data
    while True:
        data = rospy.wait_for_message(SEGMENTATION_IMAGE_TOPIC, SegmentedImage)
        stamp = data.header.stamp.to_nsec()
        if LAST_STAMP is None or stamp != LAST_STAMP:
            LAST_STAMP = stamp
            break
        else:
            sleep(0.5)
    return data


def load_test_images(file, test_filepath):
    """ Loads test data and images"""
    test_images = []
    with open(test_filepath) as json_data:
        file_data = json.load(json_data)

    for test in file_data["tests"]:
        test_images.append({
            IMAGE_MSG: image_utils.image_file_to_ros_msg(ros_utils.get_fullpath(file, test["imageName"])),
            FILENAME: test["imageName"],
            EXPECTED_OBJECTS: test["refData"]["objects"]
        })

    return test_images


def get_segmented_objets(data):
    """ Extract a list of the segmented objects found in the given ros data """
    return [obj.category_name for obj in data.objects]


def get_missed_objects(expected_objects, actual_objects):
    """ Return the list of all objects that were not detected """
    return list((Counter(expected_objects) - Counter(actual_objects)).elements())
