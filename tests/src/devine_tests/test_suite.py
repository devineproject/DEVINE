#! /usr/bin/env python2
import rosunit
from devine_tests.segmentation.rate.test_segmentation_rate import TestSegmentationRate

TEST_PACKAGE = "tests"

if __name__ == "__main__":
    rosunit.unitrun(TEST_PACKAGE, "Segmentation Rate", TestSegmentationRate)
