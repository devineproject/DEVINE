#!/usr/bin/env python

import math
import rospy
import tf

TOPIC_ROBOT_POINT_RIGHT_FRAME = "/R_frame_tool_link"
TOPIC_OBJECT_FRAME = "/object_frame"

def main():
    node_name = 'irl_control_error'
    rospy.init_node(node_name)
    rospy.loginfo('Running node \'' + node_name + '\'')

    tf_listener = tf.TransformListener()
    rate = rospy.Rate(5)

    try:
        tf_listener.waitForTransform(TOPIC_ROBOT_POINT_RIGHT_FRAME, TOPIC_OBJECT_FRAME, rospy.Time(), rospy.Duration(4))
    except tf.Exception as err:
        rospy.logerr(err)
        rospy.signal_shutdown(err)

    while not rospy.is_shutdown():
        try:
            now = rospy.Time().now()
            tf_listener.waitForTransform(TOPIC_ROBOT_POINT_RIGHT_FRAME, TOPIC_OBJECT_FRAME, now, rospy.Duration(4))
            (trans, rot) = tf_listener.lookupTransform(TOPIC_ROBOT_POINT_RIGHT_FRAME, TOPIC_OBJECT_FRAME, now)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as err:
            rospy.logerr(err)
            rospy.signal_shutdown(err)

        rospy.loginfo('Orientation from top: %s', math.floor(math.sin(trans[1]/trans[0]) * 180 / math.pi))
        rospy.loginfo('Orientation from side: %s', math.floor(math.sin(trans[2]/trans[0]) * 180 / math.pi))
        # TODO Add Publisher for error
        rate.sleep()

if __name__ == '__main__':
    main()
