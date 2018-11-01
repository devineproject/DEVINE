#!/usr/bin/env python2
"""
Position library for DEVINE using openni

ROS Topics
"""
from __future__ import division  # python 2 float division support
import rospy
import tf
import message_filters
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import PoseStamped

from devine_config import topicname
from devine_common import math_utils, ros_utils

# Topics
IMAGE_DEPTH_TOPIC = topicname('image_depth')
OBJECT_IMAGE_LOCATION_TOPIC = topicname('guess_location_image')
OBJECT_WORLD_LOCATION_TOPIC = topicname('guess_location_world')

ROBOT_BASE_LINK_FRAME = '/base_link'
CAMERA_BASE_LINK_FRAME = '/openni_base_link'

ROS_PUBLISHER = rospy.Publisher(OBJECT_WORLD_LOCATION_TOPIC, PoseStamped, queue_size=10)


def plot_openni_3d_data_matrix(data):
    """ Plot the matrix in 3D. Return the list of points. """
    return math_utils.plot_3d_matrix(deserialize(data))


def deserialize(pcloud):
    """ Deserialize a cloudpoit into an array of (x, y, z) tuples """
    return list(point_cloud2.read_points(pcloud, field_names=('x', 'y', 'z'), skip_nans=True))


class PosLib(object):
    """ Callback executed when a depth image is received from openni """

    def __init__(self, depth_topic, obj_pos_topic):
        self.trans_camera = 0
        self.rot_camera = 0
        self.tf_listener = tf.TransformListener()

        message_filters.ApproximateTimeSynchronizer([
            message_filters.Subscriber(obj_pos_topic, Int32MultiArray),
            message_filters.Subscriber(depth_topic, PointCloud2)
        ], 1, 0.5).registerCallback(self.synchronized_callback)

    def synchronized_callback(self, obj_pos_data, depth_data):
        """ Callback executed when a depth image and an object pos are received """
        rospy.loginfo('Received a new pointcloud and 2D position, image size: %ix%i',
                      depth_data.width, depth_data.height)

        self.do_point_transform(obj_pos_data.data, depth_data)

    def do_point_transform(self, position_to_transform, point_cloud):
        """ Do the 2d -> 3d transformation if we got the necessary information """
        [x, y] = position_to_transform
        center_point = math_utils.upper_left_to_zero_center(x, y, point_cloud.width, point_cloud.height)
        [z] = next(point_cloud2.read_points(point_cloud, field_names='z',
                                            skip_nans=False, uvs=[(x, y)]))

        (trans, rot) = self.get_trans(ROBOT_BASE_LINK_FRAME, CAMERA_BASE_LINK_FRAME)
        position = math_utils.calc_geometric_location(center_point[0], center_point[1], z,
                                                      point_cloud.width, point_cloud.height,
                                                      trans, rot)

        ros_packet = ros_utils.pose_stamped(position[0], position[1], position[2])
        ROS_PUBLISHER.publish(ros_packet)
        rospy.loginfo('Object 3D position calculated: (%.2f, %.2f, %.2f)',
                      position[0], position[1], position[2])

    def get_trans(self, target_frame, source_frame):
        """ Get translation between 2 frames """
        time = self.tf_listener.getLatestCommonTime(target_frame, source_frame)
        (trans, rot) = self.tf_listener.lookupTransform(target_frame, source_frame, time)
        return (trans, rot)


def main():
    """ Init node """
    node_name = 'devine_pos_lib'
    rospy.init_node(node_name)
    PosLib(IMAGE_DEPTH_TOPIC, OBJECT_IMAGE_LOCATION_TOPIC)
    rospy.loginfo('Running node \'%s\'', node_name)
    rospy.spin()


if __name__ == '__main__':
    main()
