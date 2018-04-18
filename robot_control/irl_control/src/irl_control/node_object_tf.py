#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
import tf

TOPIC_ROBOT_BASE_FRAME = 'base_link'
TOPIC_OBJECT_LOCATION = "/object_location"
TOPIC_OBJECT_FRAME = "/object_frame"

class Object(object):
    def __init__(self):
        self.object_location = [0, 0, 0]
        self.tf_broadcaster = tf.TransformBroadcaster()
        rospy.Subscriber(TOPIC_OBJECT_LOCATION, Float32MultiArray, self.object_location_callback)

    def object_location_callback(self, msg):
        if self.object_location != msg.data:
            rospy.loginfo(msg.data)
            self.object_location = msg.data
            self.brodcast_tf()

    def brodcast_tf(self):
        self.tf_broadcaster.sendTransform(self.object_location, (0.0, 0.0, 0.0, 1.0), rospy.Time.now(), TOPIC_OBJECT_FRAME, TOPIC_ROBOT_BASE_FRAME)

def main():
    node_name = 'irl_control_object_tf'
    rospy.init_node(node_name)
    rospy.loginfo('Running node \'' + node_name + '\'')

    obj = Object()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        obj.brodcast_tf()
        rate.sleep()

if __name__ == '__main__':
    main()
