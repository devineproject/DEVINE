#! /usr/bin/env python
''' Test to validate segmentation rates '''
import copy
import argparse
import rospy
from std_msgs.msg import Float32MultiArray
from devine_config import topicname
from Queue import Queue, Empty
import json
import cv2
import os
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from std_msgs.msg import String

IMAGE_TOPIC = topicname('segmentation_image')
SEGMENTATION_IMAGE_TOPIC = topicname('objects')

def list_diff(x1,x2):
    """ Returns the difference between two lists"""
    return [[x for x in x1 if x not in x2],[x for x in x2 if x not in x1]]
    
def load_test_data(test_name):
    """ Loads test data and images"""	
    imgs = []
    data = []
    with open(test_name + '.json') as json_data:
    	test_data = json.load(json_data)

    for test in test_data["tests"]:
    	imgs.append(cv2.imread(test["imageName"],cv2.IMREAD_COLOR))
    	data.append(test["refData"]["objects"])
    return imgs,data

def flatten_json(data):
    """ Takes input json object and returns a list"""
    return [object["category"] for object in data["objects"]]

def segmentation_call_back(data):
    '''Callback for segmentation topic'''
    if seg_queue.full():
    	seg_queue.get()
    try:
    	seg_queue.put(flatten_json(json.loads(data.data)))
    except Exception as e:
    	rospy.logerr(e)

def run_test(image,ref_data):
    """ Evaluates  segmentation rate for a single image """
    #send over node
    segmentation_pub = rospy.Publisher(IMAGE_TOPIC, CompressedImage, queue_size=1)
    rospy.Subscriber(SEGMENTATION_IMAGE_TOPIC, String, segmentation_call_back)
    #is_blurry = is_image_blurry(image.data)
    image_message = CvBridge().cv2_to_imgmsg(image)	
    ### Create CompressedIamge ####
    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "png"
    msg.data = np.array(cv2.imencode('.png', image)[1]).tostring()
    rospy.sleep(1)
    segmentation_pub.publish(msg)
    rospy.sleep(1)
    #recieve data
    try:
    	results = seg_queue.get()
    	[missed_detections,false_detections] = list_diff(ref_data,results)
    	missed_detection_count = len(missed_detections)
    	false_detection_count = len(false_detections)
    except Empty:
    	missed_detection_count = -1
    	false_detection_count = -1
    	pass
    #extract and comp dat
    return [missed_detection_count,false_detection_count]
    

def main(args):
    ''' Publish on the 3D position from /base_link to point '''
    # Parse arguments
    test_name = args.test
    # Start ROS node
    rospy.init_node('image_seg_test')
    global seg_queue
    seg_queue  = Queue(1)
    [imgs,data] = load_test_data(test_name)
    pos = 0
    rate = rospy.Rate(1)
    while not rospy.is_shutdown() and pos < len(imgs):
    	[missed_detection_count,false_detection_count] = run_test(imgs[pos],data[pos])
    	pos += 1
    	rospy.loginfo("Num missed detections" + str(missed_detection_count))
    	rospy.loginfo("Num false detections: " + str(false_detection_count))
    	rate.sleep()
    	print("Percentage of objects missed: " + str(float(missed_detection_count)/len(data[pos-1])))
    	print("Number of objects falsely detected: ",false_detection_count)

def parser():
    ''' Command Line Parser'''
    arg_fmt = argparse.RawDescriptionHelpFormatter
    arg_parser = argparse.ArgumentParser(formatter_class=arg_fmt, description=main.__doc__)
    image_options = arg_parser.add_argument_group('Image options')
    image_options.add_argument('-t', '--test', required=True, help='What test to run')
    arguments = arg_parser.parse_args(rospy.myargv()[1:])
    return arguments

if __name__ == '__main__':
    main(parser())
    rospy.spin()
