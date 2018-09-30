#!/usr/bin/env python2

''' Calculates the error in real-time '''

import math
import rospy
import tf

from irl_control import irl_constant

TOPIC_ROBOT_POINT_RIGHT_FRAME = irl_constant.ROBOT_LINK['r_frame_tool']
TOPIC_OBJECT_FRAME = '/object_frame'

def main():
    ''' Compute error when pointing with arms '''

    node_name = 'irl_control_error'
    rospy.init_node(node_name)
    rospy.loginfo('Running node \'' + node_name + '\'')

    tf_listener = tf.TransformListener()
    duration = rospy.Duration(4)

    try:
        tf_listener.waitForTransform(TOPIC_ROBOT_POINT_RIGHT_FRAME,
                                     TOPIC_OBJECT_FRAME,
                                     rospy.Time(),
                                     duration)
    except tf.Exception as err:
        rospy.logerr(err)
        rospy.signal_shutdown(err)

    while not rospy.is_shutdown():
        try:
            now = rospy.Time().now()
            tf_listener.waitForTransform(TOPIC_ROBOT_POINT_RIGHT_FRAME,
                                         TOPIC_OBJECT_FRAME,
                                         now,
                                         duration)
            (trans, _) = tf_listener.lookupTransform(TOPIC_ROBOT_POINT_RIGHT_FRAME,
                                                     TOPIC_OBJECT_FRAME,
                                                     now)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as err:
            rospy.logerr(err)
            rospy.signal_shutdown(err)

        rospy.loginfo('Orientation from top: %s', angle(trans[0], trans[1]))
        rospy.loginfo('Orientation from side: %s', angle(trans[0], trans[2]))
        # TODO Add Publisher for error

def angle(pos_x, pos_y):
    ''' Angle in degrees of 2D point '''

    return math.floor(math.sin(pos_y/pos_x) * 180 / math.pi)


if __name__ == '__main__':
    main()
