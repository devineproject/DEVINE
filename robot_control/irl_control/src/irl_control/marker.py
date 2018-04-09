#!/usr/bin/env python

import argparse
import math
import random
import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import tf

class Markers(object):
    def __init__(self, markers_position=None):
        topic = 'visualization_marker_array'
        self._pub = rospy.Publisher(topic, MarkerArray, queue_size=10)
        if markers_position:
            self.set_marker_array(markers_position)

    def publish(self):
        self._pub.publish(self.marker_array)

    def set_marker_array(self, markers_position):
        self.marker_array = MarkerArray()
        marker_id = 0
        for marker_position in markers_position:
            marker = Marker()
            marker.header.frame_id = "/base_link"
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

def main(args):
    point = [float(i) for i in args.point.split(',')]

    markers_position = [point]

    rospy.init_node('makers_tf')
    rospy.loginfo('Running node \'' + 'markers_tf' + '\' with: ' + str(markers_position))

    markers = Markers(markers_position)
    tf_broadcaster = tf.TransformBroadcaster()
    tf_listener = tf.TransformListener()

    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        markers.publish()
        tf_broadcaster.sendTransform(markers_position[0], (0.0, 0.0, 0.0, 1.0), rospy.Time.now(), "obj", "base_link")
        try:
            (trans1, rot1) = tf_listener.lookupTransform('/R_frame_tool_link', '/obj', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        rospy.loginfo('Orientation from top: %s', math.floor(math.sin(trans1[1]/trans1[0]) * 180 / math.pi))
        rospy.loginfo('Orientation from side: %s', math.floor(math.sin(trans1[2]/trans1[0]) * 180 / math.pi))
        rate.sleep()

if __name__ == '__main__':
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt, description=main.__doc__)
    required = parser.add_argument_group('required arguments')
    required.add_argument('-p', '--point', required=True, help='What 3D point to point?')
    args = parser.parse_args(rospy.myargv()[1:])

    main(args)
