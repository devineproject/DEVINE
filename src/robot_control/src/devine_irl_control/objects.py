""" MarkerArray to see object in RViz, to calculate TF and angle errors """
__author__ = "Jordan Prince Tremblay, Ismael Balafrej, Felix Labelle, Félix Martel-Denis, Eric Matte, Adam Letourneau, Julien Chouinard-Beaupré, Antoine Mercier-Nicol"
__copyright__ = "Copyright 2018, DEVINE Project"
__credits__ = ["Simon Brodeur", "François Ferland", "Jean Rouat"]
__license__ = "BSD"
__version__ = "1.0.0"
__email__ = "devine.gegi-request@listes.usherbrooke.ca"
__status__ = "Production"

import random
import rospy

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


class ObjectMaker(object):
    """ On topic_name changes, marker array is updated """

    def __init__(self, topic_name):
        self.object_location = None
        self.markers = Markers()
        rospy.Subscriber(topic_name, PoseStamped, self.object_location_callback)

    def object_location_callback(self, msg):
        """ Create marker array at /object_location position """
        if self.object_location != msg:
            position = msg.pose.position
            rospy.loginfo('New object location: %s', position)
            self.object_location = msg
            self.markers.set_marker_array([[position.x, position.y, position.z]])
            self.markers.publish()


class Markers(object):
    """ Wrapper of MarkerArray """

    def __init__(self, markers_position=None):
        self._pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
        if markers_position:
            self.set_marker_array(markers_position)

    def publish(self):
        """ Publisher for MarkerArray """
        self._pub.publish(self.marker_array)

    def set_marker_array(self, markers_position):
        """ Create random color MarkerArray from reference frame """
        self.marker_array = MarkerArray()
        marker_id = 0
        for marker_position in markers_position:
            marker = Marker()
            marker.header.frame_id = '/base_link'
            marker.type = marker.SPHERE
            marker.action = marker.ADD

            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1

            marker.color.a = 1.0
            marker.color.r = random.random()
            marker.color.g = random.random()
            marker.color.b = random.random()

            marker.pose.position.x = marker_position[0]
            marker.pose.position.y = marker_position[1]
            marker.pose.position.z = marker_position[2]

            marker.id = marker_id
            marker_id += 1

            self.marker_array.markers.append(marker)
