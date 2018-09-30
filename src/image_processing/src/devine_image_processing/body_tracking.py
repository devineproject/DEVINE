#!/usr/bin/env python2
'''ROS module for body tracking'''

import sys
import os

import rospy
from std_msgs.msg import String

from bson import json_util

from tf_pose.estimator import TfPoseEstimator
from tf_pose.networks import get_graph_path

import cv2

from ros_image_processor import ImageProcessor, ROSImageProcessingWrapper
from devine_config import topicname

#paths
ROOT_DIR = os.path.dirname(os.path.realpath(__file__))
MODEL_DIR = os.path.join(ROOT_DIR, "../../mobilenet_thin.pb") # originally in './models/graph/mobilenet_thin/graph_opt.pb'

#topics
IMAGE_TOPIC = topicname('body_tracking_image')
PUBLISH_TOPIC = topicname('body_tracking')

class BodyTracking(ImageProcessor):
    '''Body Tracking wrapper of tf_pose for use in guesswhat'''
    body_parts_names = [
        "Nose", "Neck", "RShoulder", "RElbow", "RWrist", "LShoulder", "LElbow", "LWrist", "RHip", "RKnee",
        "RAnkle", "LHip", "LKnee", "LAnkle", "REye", "LEye", "REar", "LEar", "Background"
    ]
    def __init__(self):
        self.estimator = TfPoseEstimator(MODEL_DIR, target_size=(432, 368)) #downscale image 

    def process(self, img):
        '''Actual body tracking of the image'''
        rospy.logdebug("Starting body tracking")
        image = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
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

def main():
    '''Entry point of this file'''
    processor = ROSImageProcessingWrapper(BodyTracking, IMAGE_TOPIC)
    publisher = rospy.Publisher(PUBLISH_TOPIC, String, queue_size=10, latch=True)
    processor.loop(lambda processor_output : publisher.publish(processor_output))

if __name__ == '__main__':
    main()
