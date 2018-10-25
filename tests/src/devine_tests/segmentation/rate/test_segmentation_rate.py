#! /usr/bin/env python2
import json
import unittest
import rospy
from devine_common import image_utils
import devine_tests.segmentation.segmentation_helper as helper

class TestSegmentationRate(unittest.TestCase):
    """ Test to validate segmentation rates """

    def test_segmentation_rate_on_two_frames(self):
        """ Loads images and posts the corresponding segmentation rates """
        test_images = helper.load_test_images(__file__, image_utils.get_fullpath(__file__, "test.json"))

        for image in test_images:
            self.assertFalse(rospy.is_shutdown())

            rospy.loginfo("### Test image %s ###", image[helper.FILENAME])
            self.segmentation_should_find_most_of_the_objects(image)

    def segmentation_should_find_most_of_the_objects(self, test_image):
        """ Evaluates  segmentation rate for a single image """
        expected_objects = test_image[helper.EXPECTED_OBJECTS]

        data = helper.segment_image(test_image)
        rospy.logwarn("Message timestamp: %s", json.loads(data.data)['timestamp'])
        objects_found = helper.get_segmented_objets(data)

        missed_objects = helper.get_missed_objects(expected_objects, objects_found)
        other_found_objects = helper.get_missed_objects(objects_found, expected_objects)

        rospy.loginfo("Number of missed detections: %d", len(missed_objects))
        rospy.loginfo("Number of other detected objects: %d", len(other_found_objects))
        objects_missed_ratio = float(len(missed_objects)) / len(expected_objects)
        rospy.loginfo("Percentage of objects missed: %.02f", objects_missed_ratio)
        if objects_missed_ratio >= 0.5:
            rospy.logwarn("The ratio of objects missed if greater than 50%.")

        self.assertTrue(objects_missed_ratio < 0.7, "The object missed ratio is too high.")


if __name__ == "__main__":
    rospy.init_node("test")
    unittest.main()
