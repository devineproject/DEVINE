#!/usr/bin/env python2
"""
Position library for DEVINE using openni

ROS Topics
"""
from __future__ import division #python 2 float division support
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Float32MultiArray, Int32MultiArray
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import math

IMAGE_DEPTH_TOPIC = "/camera/depth/points"
OBJECT_POSITION_TOPIC = "/object_found"
ROS_PUBLISHER = rospy.Publisher('/object_location', Float32MultiArray, queue_size=10)

class PosLib(object):
    """
    Callback executed when a depth image is received from openni
    """
    def __init__(self, depth_topic, obj_pos_topic):
        self.subscription = rospy.Subscriber(depth_topic, 
                                            PointCloud2, self.openni_depth_callback)
        rospy.Subscriber(obj_pos_topic, Int32MultiArray, self.object_position_callback)
        self.current_2d_position = [320, 240] #in pixel

    def openni_depth_callback(self, data):
        """
        Callback executed when a depth image is received from openni
        """
        #self.subscription.unregister() #only do it once
        #points = self.deserialize(data)
        #self.draw(points)
        rospy.loginfo("Image size: %i x %i", data.width, data.height)
        val = next(point_cloud2.read_points(data, field_names='z', skip_nans=False, uvs=[tuple(self.current_2d_position)]))
        position = self.calc_geometric_location(self.current_2d_position[0], self.current_2d_position[1], val[0], data.width)
        rospy.loginfo("Object position found: (%.2f, %.2f, %.2f)", position[0], position[1], position[2])
        ros_packet = Float32MultiArray()
        ros_packet.data = position
        ROS_PUBLISHER.publish(ros_packet)

    def object_position_callback(self, data):
        """
        Callback executed when an object is found by guesswhat and its position is broadcasted
        """
        self.current_2d_position = data.data


    def calc_geometric_location(self, x_pixel, y_pixel, kinect_z, width):
        f = width / (2 * math.tan(math.radians(57/2))) #57 = fov angle in kinect spects
        d = kinect_z / f
        x = d * x_pixel
        y = d * y_pixel
        return [x, y, kinect_z]


    def deserialize(self, pcloud):
        """
        Deserialize a cloudpoit into an array of (x, y, z) tuples
        """
        return list(point_cloud2.read_points(pcloud, field_names=("x", "y", "z"), skip_nans=True))


    def draw(self, points):
        """
        Draw an array of (x, y, z) tuples with matplotlib
        """
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        vects = zip(*points)
        xdata = list(vects[0])
        ydata = list(vects[1])
        zdata = list(vects[2])
        ax.scatter(xdata, ydata, zdata, c=zdata, cmap='Greens')
        plt.show()


if __name__ == '__main__':
    rospy.init_node('devine_kinect')
    PosLib(IMAGE_DEPTH_TOPIC, OBJECT_POSITION_TOPIC) 
    rospy.loginfo("Node initialized")
    rospy.spin()
