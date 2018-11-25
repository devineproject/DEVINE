#!/usr/bin/env python3
""" ROS module for body tracking """
__author__ = "Jordan Prince Tremblay, Ismael Balafrej, Felix Labelle, Félix Martel-Denis, Eric Matte, Adam Letourneau, Julien Chouinard-Beaupré, Antoine Mercier-Nicol"
__copyright__ = "Copyright 2018, DEVINE Project"
__credits__ = ["Simon Brodeur", "François Ferland", "Jean Rouat"]
__license__ = "BSD"
__version__ = "1.0.0"
__email__ = "devine.gegi-request@listes.usherbrooke.ca"
__status__ = "Production"

import rospy
import cv2
import tensorflow as tf
from std_msgs.msg import String
from bson import json_util
from tf_pose.estimator import TfPoseEstimator
from keras.backend.tensorflow_backend import set_session
from devine_config import topicname
from devine_common import ros_utils
from ros_image_processor import ImageProcessor, ROSImageProcessingWrapper

# Originally in './models/graph/mobilenet_thin/graph_opt.pb'
MODEL_DIR = ros_utils.get_fullpath(__file__, '../../mobilenet_thin.pb')

# Topics
IMAGE_TOPIC = topicname('body_tracking_image')
PUBLISH_TOPIC = topicname('body_tracking')

class BodyTracking(ImageProcessor):
    """ Body Tracking wrapper of tf_pose for use in guesswhat """

    body_parts_names = [
        'Nose', 'Neck', 'RShoulder', 'RElbow', 'RWrist', 'LShoulder', 'LElbow', 'LWrist', 'RHip', 'RKnee',
        'RAnkle', 'LHip', 'LKnee', 'LAnkle', 'REye', 'LEye', 'REar', 'LEar', 'Background'
    ]

    def __init__(self):
        config = tf.ConfigProto(log_device_placement=False)
        config.gpu_options.allow_growth = True
        set_session(tf.Session(config=config))
        self.estimator = TfPoseEstimator(MODEL_DIR, target_size=(432, 368))  # downscale image

    def process(self, img, img_payload):
        """ Actual body tracking of the image """
        rospy.logdebug('Starting body tracking')
        image = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        # True if resize is not the same size
        humans = self.estimator.inference(image, resize_to_default=True, upsample_size=4.0)
        # # Draw humans on image
        # image = TfPoseEstimator.draw_humans(image, humans, imgcopy=False)
        # # Show image
        # cv2.imshow("result", image)
        # cv2.waitKey(1)

        rospy.logdebug('Body tracking done')
        result_obj = []

        for hum in humans:
            result_human = {
                'body_parts': []
            }
            for (_, bp) in hum.body_parts.items():
                result_human['body_parts'].append({
                    'name': self.body_parts_names[bp.part_idx],
                    'index': bp.part_idx,
                    'x': bp.x,
                    'y': bp.y,
                    'score': bp.score,
                    'stamp': img_payload.header.stamp.to_nsec()
                })
            result_obj.append(result_human)
        return json_util.dumps(result_obj)


def main():
    """ Entry point of this file """
    processor = ROSImageProcessingWrapper(BodyTracking, IMAGE_TOPIC)
    publisher = rospy.Publisher(PUBLISH_TOPIC, String, queue_size=1)
    processor.loop(lambda processor_output: publisher.publish(processor_output))

if __name__ == '__main__':
    main()
