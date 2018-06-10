#!/usr/bin/env python2
'''ROS module for body tracking'''

import sys
import os
import Queue
from io import BytesIO

import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

import numpy as np
from PIL import Image
from bson import json_util

from tf_pose.estimator import TfPoseEstimator
from tf_pose.networks import get_graph_path

import cv2

#paths
ROOT_DIR = sys.path[0]
MODEL_DIR = os.path.join(ROOT_DIR, "../../mobilenet_thin.pb") # originally in './models/graph/mobilenet_thin/graph_opt.pb'

#topics
IMAGE_TOPIC = '/camera/rgb/image_color/compressed' # or /devine/image
PUBLISH_TOPIC = '/body_tracking'

class BodyTracking(object):
    '''Body Tracking wrapper of tf_pose for use in guesswhat'''
    body_parts_names = [
        "Nose", "Neck", "RShoulder", "RElbow", "RWrist", "LShoulder", "LElbow", "LWrist", "RHip", "RKnee",
        "RAnkle", "LHip", "LKnee", "LAnkle", "REye", "LEye", "REar", "LEar", "Background"
    ]
    def __init__(self):
        self.estimator = TfPoseEstimator(MODEL_DIR, target_size=(432, 368)) #downscale image 

    def track(self, img):
        '''Actual body tracking of the image'''
        rospy.logdebug("Starting body tracking")
        image = cv2.cvtColor(np.array(Image.open(BytesIO(img))), cv2.COLOR_RGB2BGR)
        humans = self.estimator.inference(image, resize_to_default=True, upsample_size=4.0) #True if resize is not the same size
        
        #Draw humans on image
        #image = TfPoseEstimator.draw_humans(image, humans, imgcopy=False)
        #Show image
        #cv2.imshow("result", image)
        #cv2.waitKey(1)
        
        rospy.logdebug("Body tracking done")
        result_obj = []
        for hum in humans:
            result_human = {
                "body_parts": []
            }
            for (_, bp) in hum.body_parts.iteritems():
                result_human["body_parts"].append({
                    "name": self.body_parts_names[bp.part_idx],
                    "index": bp.part_idx,
                    "x": bp.x,
                    "y": bp.y,
                    "score": bp.score
                })
            result_obj.append(result_human)
        return json_util.dumps(result_obj)

class ROSBodyTracking(BodyTracking):
    '''ROS Wrapper of BodyTracking'''
    image_queue = Queue.Queue(2) #Tracking must be on the main thread

    def __init__(self):
        super(ROSBodyTracking, self).__init__()
        rospy.init_node('body_tracking')
        rospy.Subscriber(IMAGE_TOPIC, CompressedImage,
                         self.image_received_callback, queue_size=1)
        self.publisher = rospy.Publisher(PUBLISH_TOPIC, String, queue_size=10, latch=True)

    def image_received_callback(self, data):
        '''Callback when a new image is received from the topic'''
        if self.image_queue.full():
            rospy.logwarn("body_tracking: image receiving rate is too high !")
            self.image_queue.get()
        self.image_queue.put(data.data)

    def loop(self):
        '''Looping method to track every image'''
        while True:
            img = self.image_queue.get() #blocking
            json = self.track(img)
            self.publisher.publish(json)

def main():
    '''Entry point of this file'''
    ros_body_tracking = ROSBodyTracking()
    ros_body_tracking.loop()

if __name__ == '__main__':
    main()
