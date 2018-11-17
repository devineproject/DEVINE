#!/usr/bin/env python2
"""
Position library for DEVINE using openni

ROS Topics
"""
from __future__ import division  # python 2 float division support
import rospy
import tf
import message_filters
from sensor_msgs.msg import PointCloud2, CompressedImage
from sensor_msgs import point_cloud2
from geometry_msgs.msg import PoseStamped, PointStamped

from devine_config import topicname, constant
from devine_common import math_utils, ros_utils

CAM_FRAME_OPTICAL = constant('cam_frame_optical')

# Topics
IMAGE_DEPTH_TOPIC = topicname('image_depth')
OBJECT_IMAGE_LOCATION_TOPIC = topicname('guess_location_image')
OBJECT_WORLD_LOCATION_TOPIC = topicname('guess_location_world')
SEG_IMAGE_TOPIC = topicname('segmentation_image')

ROS_PUBLISHER = rospy.Publisher(OBJECT_WORLD_LOCATION_TOPIC, PoseStamped, queue_size=1)


def plot_openni_3d_data_matrix(data):
    """ Plot the matrix in 3D. Return the list of points. """
    return math_utils.plot_3d_matrix(deserialize(data))


def deserialize(pcloud):
    """ Deserialize a cloudpoint into an array of (x, y, z) tuples """
    return list(point_cloud2.read_points(pcloud, field_names=('x', 'y', 'z'), skip_nans=True))


class PosLib(object):
    """ Callback executed when a depth image is received from openni """

    def __init__(self, depth_topic, obj_pos_topic):
        self.trans_camera = 0
        self.rot_camera = 0
        self.tf_listener = tf.TransformListener()

        message_filters.ApproximateTimeSynchronizer([
            message_filters.Subscriber(SEG_IMAGE_TOPIC, CompressedImage),
            message_filters.Subscriber(depth_topic, PointCloud2)
        ], 1, 0.5).registerCallback(self._save_point_cloud)

        rospy.Subscriber(obj_pos_topic, PointStamped, self.do_point_transform, queue_size=1)
        self.point_cloud = None

    def _save_point_cloud(self, _, point_cloud):
        self.point_cloud = point_cloud

    def do_point_transform(self, position_to_transform):
        """ Do the 2d -> 3d transformation if we got the necessary information """

        point_cloud = self.point_cloud
        if point_cloud is None:
            rospy.logerr('Error while getting the point cloud')
            return

        rospy.loginfo('Received a new pointcloud and 2D position, image size: %ix%i',
                      point_cloud.width, point_cloud.height)

        x = int(position_to_transform.point.x)
        y = int(position_to_transform.point.y)
        center_points = math_utils.upper_left_to_zero_center(x, y, point_cloud.width, point_cloud.height)
        [z] = next(point_cloud2.read_points(point_cloud, field_names='z',
                                            skip_nans=False, uvs=[(x, y)]))

        pos_xyz = math_utils.pixel_to_meters(center_points[0], center_points[1],
                                             z, point_cloud.width)

        ros_packet = ros_utils.pose_stamped(pos_xyz[0], pos_xyz[1], pos_xyz[2])
        ros_packet.header = position_to_transform.header
        ros_packet.header.frame_id = CAM_FRAME_OPTICAL
        ROS_PUBLISHER.publish(ros_packet)
        rospy.loginfo('Object 3D position calculated: (%.2f, %.2f, %.2f)',
                      pos_xyz[0], pos_xyz[1], pos_xyz[2])

        self.point_cloud = None


def main():
    """ Init node """
    node_name = 'devine_pos_lib'
    rospy.init_node(node_name)
    PosLib(IMAGE_DEPTH_TOPIC, OBJECT_IMAGE_LOCATION_TOPIC)
    rospy.loginfo('Running node \'%s\'', node_name)
    rospy.spin()


if __name__ == '__main__':
    main()
