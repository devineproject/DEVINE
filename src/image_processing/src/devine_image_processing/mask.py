#!/usr/bin/env python2

''' Depth mask of images based on a distance threshold '''

from threading import Lock
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from devine_config import topicname
from sensor_msgs.msg import CompressedImage
import numpy as np
from sensor_msgs.msg import Image as ROS_Image
from devine_common.image_utils import image_to_ros_msg
from PIL import Image
from io import BytesIO
from cv_bridge import CvBridge, CvBridgeError

 #Topics
IMAGE_DEPTH_TOPIC = '/camera/depth/image' #topicname('image_depth')
IMAGE_TOPIC = topicname('raw_image')
IMAGE_PUB_TOPIC =  topicname('segmentation_image') #"/devine/image/depth_masked" #topicname('depth_masked_image')
DEPTH_THRESHOLD = 1.5 # meters of depth

 
class DepthMask(object):
    ''' Callback executed when a depth image is received from openni '''
    def __init__(self):
        self.depth_subscription = rospy.Subscriber(IMAGE_DEPTH_TOPIC, ROS_Image, self.openni_depth_callback)
        self.img_subscription = rospy.Subscriber(IMAGE_TOPIC, CompressedImage, self.img_callback)
        self.mutex = Lock() #Lock mutex to sync up point_cloud and 2D position calculation
        self.current_point_cloud = None
        self.current_img = None
        self.depth_mask_publisher = rospy.Publisher(IMAGE_PUB_TOPIC, CompressedImage, queue_size=10)
       
    def img_callback(self, data):
        self.mutex.acquire()
        self.current_img = data
        self.mutex.release()
        self.do_mask()
        
    def openni_depth_callback(self, data):
        ''' Callback executed when a depth image is received from openni '''
        self.mutex.acquire()
        self.current_point_cloud = data
        self.mutex.release()
        self.do_mask()
        
    def do_mask(self):
         if self.current_img is not None and self.current_point_cloud is not None:
             self.mutex.acquire()
             #np_img = np.fromstring(self.current_img.data, np.uint8)
             np_img = np.array(Image.open(BytesIO(self.current_img.data))).astype(np.uint8)
             coordinates = np.array(np.meshgrid(np.array(range(0,481)), np.array(range(0,641)))).T.reshape(-1,2)
             #gen = point_cloud2.read_points(self.current_point_cloud, field_names=('z'),uvs = coordinates,  skip_nans=True)
             coordinates = CvBridge().imgmsg_to_cv2(self.current_point_cloud.data, desired_encoding="passthrough")
             print(coordinates.shape)
             self.mutex.release() 	
             #print(np_img.shape)
             #print(sum([1 for _ in gen]))
             for item in gen:
                 print(item)
             for [u, v, z] in gen:
                 if z > DEPTH_THRESHOLD:
                     print(x,y,z)
                     np_img[x,y,:] = 0
             print(np_img.shape)
             msg = image_to_ros_msg(np_img)
             self.depth_mask_publisher.publish(msg)
             self.current_img = None
             self.current_point_cloud = None
            
            
            
def main():
    ''' Init node '''
    rospy.init_node('devine_depth_mask')
    DepthMask()
    rospy.loginfo("Node initialized")
    rospy.spin()
    
if __name__ == '__main__':
    main()
