#!/usr/bin/env python2
# -*- coding: utf-8 -*-
""" Simple test of the human finder """
__author__ = "Jordan Prince Tremblay, Ismael Balafrej, Felix Labelle, Felix Martel-Denis, Eric Matte, Adam Letourneau, Julien Chouinard-Beaupre, Antoine Mercier-Nicol"
__copyright__ = "Copyright 2018, DEVINE Project"
__credits__ = ["Simon Brodeur", "Francois Ferland", "Jean Rouat"]
__license__ = "BSD"
__version__ = "1.0.0"
__email__ = "devine.gegi-request@listes.usherbrooke.ca"
__status__ = "Production"

import rospy

import actionlib

from devine_head_coordinator.msg import LookAtHumanAction, LookAtHumanGoal

def get_result():
    """ Call the human finder actionlib and get the result """
    client = actionlib.SimpleActionClient('human_finder', LookAtHumanAction)
    
    rospy.loginfo('Waiting for server...')
    client.wait_for_server()

    rospy.loginfo('Sending goal')
    client.send_goal(LookAtHumanGoal(period=rospy.rostime.Duration(0)))

    rospy.loginfo('Waiting for results')
    client.wait_for_result()

    return client.get_result() 

if __name__ == '__main__':
    rospy.init_node('test_human_client')
    result = get_result()
    rospy.loginfo('Result: %s', str(result))
