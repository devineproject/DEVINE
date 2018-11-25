#! /usr/bin/env python2
""" Run the DEVINE test suite """
__author__ = "Jordan Prince Tremblay, Ismael Balafrej, Felix Labelle, Félix Martel-Denis, Eric Matte, Adam Letourneau, Julien Chouinard-Beaupré, Antoine Mercier-Nicol"
__copyright__ = "Copyright 2018, DEVINE Project"
__credits__ = ["Simon Brodeur", "François Ferland", "Jean Rouat"]
__license__ = "BSD"
__version__ = "1.0.0"
__email__ = "devine.gegi-request@listes.usherbrooke.ca"
__status__ = "Production"

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
