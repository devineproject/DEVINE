#! /usr/bin/env python2
# -*- coding: utf-8 -*-
""" Node to visualize object in RViz and broadcast object_frame """
__author__ = "Jordan Prince Tremblay, Ismael Balafrej, Felix Labelle, Félix Martel-Denis, Eric Matte, Adam Letourneau, Julien Chouinard-Beaupré, Antoine Mercier-Nicol"
__copyright__ = "Copyright 2018, DEVINE Project"
__credits__ = ["Simon Brodeur", "François Ferland", "Jean Rouat"]
__license__ = "BSD"
__version__ = "1.0.0"
__email__ = "devine.gegi-request@listes.usherbrooke.ca"
__status__ = "Production"

import rospy
from devine_irl_control.objects import ObjectMaker
from devine_config import topicname

TOPIC_OBJECT_LOCATION = topicname('guess_location_world')


def main():
    """ Init ROS Node to create marker and broadcast TF at postion /object_location """
    node_name = 'devine_irl_control_object'
    rospy.init_node(node_name)
    rospy.loginfo('Running node \'%s\'', node_name)

    ObjectMaker(TOPIC_OBJECT_LOCATION)
    rospy.spin()


if __name__ == '__main__':
    main()
