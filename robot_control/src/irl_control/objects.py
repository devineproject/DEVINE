''' MarkerArray to see object in RViz, to calculate TF and angle errors '''

import random
import rospy
import tf

from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


class ObjectTf(object):
    ''' Object TF '''

    def __init__(self, topic_object_pos, topic_object_frame, topic_from_ref):
        self.object_location = [0, 0, 0]

        self.topic_object_pos = topic_object_pos
        self.topic_object_frame = topic_object_frame
        self.topic_from_ref = topic_from_ref

        self.tf_broadcaster = tf.TransformBroadcaster()
        rospy.Subscriber(topic_object_pos, Float32MultiArray, self.object_location_callback)

    def object_location_callback(self, msg):
        ''' On /object_location changes, broadcast TF frame '''

        if self.object_location != msg.data:
            rospy.loginfo(msg.data)
            self.object_location = msg.data
            self.brodcast_tf()

    def brodcast_tf(self):
        ''' Broadcast transformation between /base_link and /object_frame '''

        self.tf_broadcaster.sendTransform(self.object_location,
                                          (0.0, 0.0, 0.0, 1.0),
                                          rospy.Time.now(),
                                          self.topic_object_frame,
                                          self.topic_from_ref)

class ObjectMaker(object):
    ''' On topic_name changes, marker array is updated '''

    def __init__(self, topic_name):
        self.object_location = None
        self.markers = Markers()
        rospy.Subscriber(topic_name, Float32MultiArray, self.object_location_callback)

    def object_location_callback(self, msg):
        ''' Create marker array at /object_location position '''

        if self.object_location != msg.data:
            rospy.loginfo('New object location: %s', msg.data)
            self.object_location = msg.data
            self.markers.set_marker_array([msg.data])
            self.markers.publish()

class Markers(object):
    ''' Wrapper of MarkerArray '''

    def __init__(self, markers_position=None):
        topic = 'visualization_marker_array'
        self._pub = rospy.Publisher(topic, MarkerArray, queue_size=10)
        if markers_position:
            self.set_marker_array(markers_position)

    def publish(self):
        ''' Publisher for MarkerArray '''

        self._pub.publish(self.marker_array)

    def set_marker_array(self, markers_position):
        ''' Create random color MarkerArray from reference frame '''

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
