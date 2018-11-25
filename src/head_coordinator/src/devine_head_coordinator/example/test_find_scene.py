#! /usr/bin/env python2
""" Example to find scene """
__author__ = "Jordan Prince Tremblay, Ismael Balafrej, Felix Labelle, Félix Martel-Denis, Eric Matte, Adam Letourneau, Julien Chouinard-Beaupré, Antoine Mercier-Nicol"
__copyright__ = "Copyright 2018, DEVINE Project"
__credits__ = ["Simon Brodeur", "François Ferland", "Jean Rouat"]
__license__ = "BSD"
__version__ = "1.0.0"
__email__ = "devine.gegi-request@listes.usherbrooke.ca"
__status__ = "Production"

import rospy
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint
from devine_config import topicname

ZONE_DETECTION_TOPIC = topicname('zone_detection')
TOPIC_HEAD_JOINT_STATE = topicname('robot_head_joint_traj_point')


def main():
    """ Example to look for a scene """
    # Start ROS node
    node_name = 'devine_common_find_scene_example'
    rospy.init_node(node_name)

    pub_zone_detection = rospy.Publisher(ZONE_DETECTION_TOPIC,
                                         String,
                                         queue_size=10)
    pub_neck_ctrl = rospy.Publisher(TOPIC_HEAD_JOINT_STATE,
                                    JointTrajectoryPoint, queue_size=1)

    while not rospy.is_shutdown():
        #TODO: Revamp
        
        time = 1
        # Init head
        rospy.loginfo('Init')
        ros_packet = JointTrajectoryPoint(positions=[0, 0],
                                          time_from_start=rospy.Duration(time))
        pub_neck_ctrl.publish(ros_packet)

        rospy.sleep(3)

        # Zone Dectection Not Found
        rospy.loginfo('Zone Dectection Not Found: Head should sweep')
        payload = "{\"top_left_corner\": [-1, -1], \"bottom_right_corner\": [-1, -1]}"
        ros_packet = String(payload)
        pub_zone_detection.publish(ros_packet)

        rospy.sleep(3)

        # Zone Dectection Found
        rospy.loginfo('Zone Dectection Found')
        payload = "{\"top_left_corner\": [353.5, 188.25], \"bottom_right_corner\": [353.5, 188.25]}"
        ros_packet = String(payload)
        pub_zone_detection.publish(ros_packet)

        rospy.sleep(3)

        # Move Head to play game
        rospy.loginfo('Move Head to play game')
        ros_packet = JointTrajectoryPoint(positions=[1, 0],
                                          time_from_start=rospy.Duration(time))
        pub_neck_ctrl.publish(ros_packet)

        rospy.sleep(3)

        # Go back to known zone
        rospy.loginfo('Go back to known zone')
        payload = '{"top_left_corner": [-1, -1], "bottom_right_corner": [-1, -1]}'
        ros_packet = String(payload)
        pub_zone_detection.publish(ros_packet)

        rospy.sleep(3)

        # Known zone is gone
        rospy.loginfo('Known zone is gone: Head should sweep')
        payload = '{"top_left_corner": [-1, -1], "bottom_right_corner": [-1, -1]}'
        ros_packet = String(payload)
        pub_zone_detection.publish(ros_packet)

        rospy.sleep(3)

        # Known zone is not found
        rospy.loginfo('Known zone is not found: Head should sweep')
        payload = '{"top_left_corner": [-1, -1], "bottom_right_corner": [-1, -1]}'
        ros_packet = String(payload)
        pub_zone_detection.publish(ros_packet)

        rospy.sleep(3)

        rospy.loginfo('Test done')


if __name__ == '__main__':
    main()
