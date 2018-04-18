#!/usr/bin/env python2
"""
Position library for DEVINE using openni

ROS Topics
"""
from __future__ import division #python 2 float division support
from threading import Lock
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Float32MultiArray, Int32MultiArray
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import math
import tf


IMAGE_DEPTH_TOPIC = "/camera/depth/points"
OBJECT_POSITION_TOPIC = "/object_found"

ROS_PUBLISHER = rospy.Publisher('/object_location', Float32MultiArray, queue_size=10)

(trans, rot) = (0, 0)

class PosLib(object):
    """
    Callback executed when a depth image is received from openni
    """
    def __init__(self, depth_topic, obj_pos_topic):
        self.subscription = rospy.Subscriber(depth_topic, 
                                            PointCloud2, self.openni_depth_callback)
        rospy.Subscriber(obj_pos_topic, Int32MultiArray, self.object_position_callback)
        self.position_to_transform = None #[320, 240] #in pixel
        self.current_point_cloud = None
        self.mutex = Lock() #Lock mutex to sync up point_cloud and 2D position calculation

    def openni_depth_callback(self, data):
        """
        Callback executed when a depth image is received from openni
        """
        #self.subscription.unregister() #only do it once
        rospy.loginfo("Received a new pointcloud, image size: %ix%i", data.width, data.height)
        self.mutex.acquire()
        self.current_point_cloud = data
        self.mutex.release()
        self.do_point_transform()
        #points = self.draw(self.deserialize(data))

    def object_position_callback(self, data):
        """
        Callback executed when an object is found by guesswhat and its position is broadcasted
        """
        rospy.loginfo("Received a new 2D point to transform")
        self.mutex.acquire()
        self.position_to_transform = data.data
        self.mutex.release()
        self.do_point_transform()

    def do_point_transform(self):
        '''Do the 2d -> 3d transformation if we got the necessary information'''
        self.mutex.acquire()
        try :
            if self.current_point_cloud is not None and self.position_to_transform is not None:
                [x, y] = self.position_to_transform
                print(x, y)
                pc = self.current_point_cloud
                center_point = self.upper_left_to_zero_center(x, y, pc.width, pc.height)
                [z] = next(point_cloud2.read_points(pc, field_names='z', skip_nans=False, uvs=[(x,y)]))
                position = self.calc_geometric_location(center_point[0], center_point[1], z, pc.width, pc.height)
                ros_packet = Float32MultiArray()
                ros_packet.data = position
                ROS_PUBLISHER.publish(ros_packet)
                self.current_point_cloud = None
                self.position_to_transform = None
                rospy.loginfo("Object 3D position calculated: (%.2f, %.2f, %.2f)", position[0], position[1], position[2])
        finally:
            self.mutex.release()

    def upper_left_to_zero_center(self, x, y, width, height):
        '''Change referential from center to upper left'''
        return (x - int(width/2), y - int(height/2))

    def calc_geometric_location(self, x_pixel, y_pixel, kinect_z, width, height):
        f = width / (2 * math.tan(math.radians(57/2))) #57 = fov angle in kinect spects
        d = kinect_z / f
        x = d * x_pixel
        y = d * y_pixel

        [transz, transx, transy] = trans
        point = [kinect_z + transz, x + transx, y + transy]
        return self.point_rotation_by_quaternion(point, rot)

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

    # https://math.stackexchange.com/questions/40164/how-do-you-rotate-a-vector-by-a-unit-quaternion
    def quaternion_mult(self, q,r):
        '''Quaternion multiplication'''
        return [r[0]*q[0]-r[1]*q[1]-r[2]*q[2]-r[3]*q[3],
            r[0]*q[1]+r[1]*q[0]-r[2]*q[3]+r[3]*q[2],
            r[0]*q[2]+r[1]*q[3]+r[2]*q[0]-r[3]*q[1],
            r[0]*q[3]-r[1]*q[2]+r[2]*q[1]+r[3]*q[0]]

    def point_rotation_by_quaternion(self, point, q):
        '''Point rotation by a quaternion'''
        [x, y, z] = point
        r = [0, x, -1*y, -1*z]
        q_conj = [q[0],1*q[1],1*q[2],1*q[3]]
        return self.quaternion_mult(self.quaternion_mult(q,r),q_conj)[1:]


if __name__ == '__main__':
    rospy.init_node('devine_kinect')
    PosLib(IMAGE_DEPTH_TOPIC, OBJECT_POSITION_TOPIC) 
    rospy.loginfo("Node initialized")

    tf_listener = tf.TransformListener()
    rate = rospy.Rate(5)

    try:
        tf_listener.waitForTransform("/base_link", "/openni_base_link", rospy.Time(), rospy.Duration(4))
    except tf.Exception as err:
        rospy.logerr(err)
        rospy.signal_shutdown(err)

    while not rospy.is_shutdown():
        try:
            now = rospy.Time().now()
            tf_listener.waitForTransform("/base_link", "/openni_base_link", now, rospy.Duration(4))
            (trans, rot) = tf_listener.lookupTransform("/base_link", "/openni_base_link", now)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as err:
            rospy.logerr(err)
            rospy.signal_shutdown(err)

        print(trans)
        rate.sleep()

    rospy.spin()
