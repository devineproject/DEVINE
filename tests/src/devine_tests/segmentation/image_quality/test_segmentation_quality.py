#! /usr/bin/env python2
import unittest
import rospy
from devine_tests import utils
import devine_tests.segmentation.segmentation_helper as helper

class TestSegmentationQuality(unittest.TestCase):
    """ Test to validate segmentation quality """

    def test_image_quality_should_not_affect_segmentation(self):
        """ Tests image quality """
        test_images = helper.load_test_images(__file__, utils.get_fullpath(__file__, "test1.json"))

        expected_objects = test_images[0][helper.EXPECTED_OBJECTS]
        images_objects_found = []

        for image in test_images:
            self.assertFalse(rospy.is_shutdown())
            images_objects_found.append(self.get_objects_found(image))

        kinect_missed_objects = helper.get_missed_objects(expected_objects, images_objects_found[0])
        s8_missed_objects = helper.get_missed_objects(expected_objects, images_objects_found[1])

        self.assertAlmostEqual(kinect_missed_objects, s8_missed_objects, delta=1)
        self.assertLessEqual(kinect_missed_objects, 1)
        self.assertLessEqual(s8_missed_objects, 1)

    def test_image_size_should_not_affect_segmentation_too_much(self):
        """ Tests image size """
        test_images = helper.load_test_images(__file__, utils.get_fullpath(__file__, "test2.json"))

        expected_objects = test_images[0][helper.EXPECTED_OBJECTS]
        images_objects_missed = []

        for image in test_images:
            self.assertFalse(rospy.is_shutdown())
            object_found = self.get_objects_found(image)
            images_objects_missed.append(helper.get_missed_objects(expected_objects, object_found))

        for object_missed in images_objects_missed:
            self.assertLessEqual(len(object_missed), 1, "Too much expected objects were not detected.")

        objects_missed_from_smaller_image = images_objects_missed[0]
        for i in range(2, len(images_objects_missed)):
            self.assertGreaterEqual(objects_missed_from_smaller_image, len(images_objects_missed[i]) + 1)


    def get_objects_found(self, test_image):
        """ Trigger the segmentation on a given image """
        rospy.loginfo("Loading objects from %s...", test_image[helper.FILENAME])
        return helper.get_segmented_objets(helper.segment_image(test_image))


if __name__ == "__main__":
    rospy.init_node("test")
    unittest.main()
