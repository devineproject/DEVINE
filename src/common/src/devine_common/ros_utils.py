""" ROS Utils """
__author__ = "Jordan Prince Tremblay, Ismael Balafrej, Felix Labelle, Félix Martel-Denis, Eric Matte, Adam Letourneau, Julien Chouinard-Beaupré, Antoine Mercier-Nicol"
__copyright__ = "Copyright 2018, DEVINE Project"
__credits__ = ["Simon Brodeur", "François Ferland", "Jean Rouat"]
__license__ = "BSD"
__version__ = "1.0.0"
__email__ = "devine.gegi-request@listes.usherbrooke.ca"
__status__ = "Production"

import os
import sys
import rospy
from sensor_msgs.msg import CameraInfo
from devine_config import topicname
from threading import Lock

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


class TopicBlocker(object):
    """ Blocks until a topic message is received """
    def __init__(self, topic_name, topic_type):
        rospy.Subscriber(topic_name, topic_type, self._topic_callback, queue_size=1)
        self.topic_data = None
        self._mutex = Lock()
        self._mutex.acquire()

    def wait_for_message(self):
        """ Blocks until data is received from the topic """
        self._mutex.acquire()
        return self.topic_data

    def _topic_callback(self, topic_data):
        """ Callback when data is received from the topic """
        self.topic_data = topic_data
        self._mutex.release()


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
