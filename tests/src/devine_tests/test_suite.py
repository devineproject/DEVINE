#! /usr/bin/env python2
""" Run the DEVINE test suite """

import sys
import unittest
import rospy

from devine_tests.segmentation.rate.test_segmentation_rate import TestSegmentationRate
from devine_tests.segmentation.image_quality.test_segmentation_quality import TestSegmentationQuality

TEST_PACKAGE = 'tests'

if __name__ == '__main__':
    rospy.init_node('test')

    if sys.argv and sys.argv[0] == '--verbose':
        # Will run all the previously imported tests using Python unittest and display logs.
        unittest.main()
    else:
        import rosunit
        rosunit.unitrun(TEST_PACKAGE, 'Test suite', TestSegmentationRate)
        rosunit.unitrun(TEST_PACKAGE, 'Test suite', TestSegmentationQuality)
