#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from marker import Markers
import tf

TOPIC_OBJECT_LOCATION = "/object_location"

class ObjectMaker(object):
    def __init__(self):
        self.object_location = None
        self.markers = Markers()
        rospy.Subscriber(TOPIC_OBJECT_LOCATION, Float32MultiArray, self.object_location_callback)

    def object_location_callback(self, msg):
        if self.object_location != msg.data:
            rospy.loginfo('New object location: %s', msg.data)
            self.object_location = msg.data
            self.markers.set_marker_array([msg.data])
            self.markers.publish()

def main():
    node_name = 'irl_control_marker'
    rospy.init_node(node_name)
    rospy.loginfo('Running node \'' + node_name + '\'')
    
    object_marker = ObjectMaker()
    rospy.spin()

if __name__ == '__main__':
    main()
