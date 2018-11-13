""" ROS Utils """
import os
import sys
import rospy
from sensor_msgs.msg import CameraInfo
from devine_config import topicname

IS_PYTHON2 = sys.version_info[0] < 3

if IS_PYTHON2:
    import tf
    from geometry_msgs.msg import PoseStamped

    def pose_stamped(x, y, z, roll=0, pitch=0, yaw=0, ref_frame='base_link', stamp=None):
        """ Convert x, y, z, roll, pitch, yaw to PoseStamped """

        pose = PoseStamped()
        pose.header.stamp = stamp or rospy.Time.now() - rospy.rostime.Duration(0.1)
        pose.header.frame_id = ref_frame
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]

        return pose


def get_fullpath(file_name, relative_file):
    """ Return the full path of a file in a directory relative to this one
    Inputs:
    - file_name: the __file__ variable of the python file to get the reference path from
    - relative_file: the file relative path to __file__
    """
    return os.path.join(os.path.dirname(os.path.abspath(file_name)), relative_file)


def get_image_dim(topic_name=topicname('camera_info')):
    """ Return the image width and height for an image topic (blocking) """
    cam_info = rospy.wait_for_message(topic_name, CameraInfo, None)
    return (cam_info.width, cam_info.height)
