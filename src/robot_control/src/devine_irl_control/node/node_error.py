#!/usr/bin/env python2
""" Calculates the error in real-time """
__author__ = "Jordan Prince Tremblay, Ismael Balafrej, Felix Labelle, Félix Martel-Denis, Eric Matte, Adam Letourneau, Julien Chouinard-Beaupré, Antoine Mercier-Nicol"
__copyright__ = "Copyright 2018, DEVINE Project"
__credits__ = ["Simon Brodeur", "François Ferland", "Jean Rouat"]
__license__ = "BSD"
__version__ = "1.0.0"
__email__ = "devine.gegi-request@listes.usherbrooke.ca"
__status__ = "Production"

import math
import rospy
import tf

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
from devine_irl_control import irl_constant
from devine_config import topicname

# IN
TOPIC_OBJECT_LOCATION = topicname('guess_location_world')

# OUT
TOPIC_POINT_ERR = topicname('robot_err_pointing')


class ErrorPoint(object):
    """ Compute error when pointing with arms """

    def __init__(self):
        self.pose_stamp = None
        self.tf_listener = tf.TransformListener()
        rospy.Subscriber(TOPIC_OBJECT_LOCATION, PoseStamped, self.pose_stamp_callback)
        self.pub_err_top = rospy.Publisher(TOPIC_POINT_ERR, Float64MultiArray, queue_size=1)

    def compute_err(self):
        """ Compute and publish error """
        if self.pose_stamp:
            try:
                self.pose_stamp.header.stamp = self.tf_listener.getLatestCommonTime(
                    irl_constant.ROBOT_LINK['r_frame_tool'],
                    irl_constant.ROBOT_LINK['base'])

                tf_pose_stamp = self.tf_listener.transformPose(
                    irl_constant.ROBOT_LINK['r_frame_tool'],
                    self.pose_stamp)
                tf_position = tf_pose_stamp.pose.position

                err_top = angle(tf_position.x, tf_position.y)
                err_side = angle(tf_position.x, tf_position.z)

                ros_packet = Float64MultiArray()
                ros_packet.data = [err_top, err_side]
                self.pub_err_top.publish(ros_packet)

                rospy.loginfo('Orientation error in degree (top, side): %s, %s',
                              round(err_top, 2), round(err_side, 2))

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as err:
                rospy.logerr(err)

    def pose_stamp_callback(self, msg):
        """ Callback PoseStamped """
        self.pose_stamp = msg


def angle(pos_x, pos_y):
    """ Angle in degrees of 2D point """
    angle_rad = math.atan(abs(pos_y/pos_x))
    angle_degree = math.degrees(angle_rad)

    return angle_degree


def main():
    """ Start Node """
    node_name = 'devine_irl_control_error'
    rospy.init_node(node_name)
    rospy.loginfo('Running node \'%s\'', node_name)
    err = ErrorPoint()
    while not rospy.is_shutdown():
        err.compute_err()
        rospy.sleep(rospy.Duration(0.5))


if __name__ == '__main__':
    main()
