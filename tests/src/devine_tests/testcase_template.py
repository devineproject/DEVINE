#! /usr/bin/env python2
__author__ = "Jordan Prince Tremblay, Ismael Balafrej, Felix Labelle, Felix Martel-Denis, Eric Matte, Adam Letourneau, Julien Chouinard-Beaupre, Antoine Mercier-Nicol"
__copyright__ = "Copyright 2018, DEVINE Project"
__credits__ = ["Simon Brodeur", "Francois Ferland", "Jean Rouat"]
__license__ = "BSD"
__version__ = "1.0.0"
__email__ = "devine.gegi-request@listes.usherbrooke.ca"
__status__ = "Production"

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
