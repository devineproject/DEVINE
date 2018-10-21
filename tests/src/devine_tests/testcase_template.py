#! /usr/bin/env python2
import unittest
import rospy

class TemplateTestCase(unittest.TestCase):
    """ Template test case file """

    def test_should_succeed(self):
        """ Simple test """
        self.assertEqual(1, 1)


if __name__ == "__main__":
    rospy.init_node("test")
    unittest.main()
