''' ROS Utils '''

import rospy
import tf
from geometry_msgs.msg import PoseStamped

def pose_stamped(x, y, z, roll=0, pitch=0, yaw=0):
    ''' Convert x, y, z, roll, pitch, yaw to PoseStamp '''
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = 'base_link'
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z

    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    pose.pose.orientation.x = quaternion[0]
    pose.pose.orientation.y = quaternion[1]
    pose.pose.orientation.z = quaternion[2]
    pose.pose.orientation.w = quaternion[3]

    return pose
