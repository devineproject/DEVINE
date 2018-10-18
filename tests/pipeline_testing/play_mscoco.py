t to validate segmentation rates '''
import argparse
import random
import rospy
from devine_config import topicname
from Queue import Queue, Empty
import json
import cv2
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Bool
from devine_dialog.msg import TtsQuery
from devine_dialog import TTSAnswerType, send_speech

# How to load images from mscoco

IMAGE_TOPIC = topicname('raw_image')
RESTART_TOPIC = topicname('end_of_game')
GUESSED_OBJECT_TOPIC = topic('guess_category')
TTS_ANSWER_TOPIC = topicname('tts_answer')
TTS_QUERY_TOPIC = topicname('tts_query')
CATEGORY_TOPIC = topicname('guess_category')

class MSCOCOTest:
    def __init__(self,image_folder,num_games):
        self.images = []
        self.image_folder = image_folder
        self.num_games = num_games
        self._load_test_data()
		self.question = ""
        self.guessed_category = ""
        
        rospy.init_node('mscoco_test')
        self.rate = rospy.Rate(1)
        self.image_pub = rospy.Publisher(IMAGE_TOPIC, CompressedImage, queue_size=1)
        self.restart_pub = rospy.Publisher(RESTART_TOPIC, Bool, queue_size=1)
		self.answer_pub = rospy.Publisher(TTS_ANSWER_TOPIC, TtsQuery, queue_size=1)
		rospy.Subscriber(CATEGORY_TOPIC, String, self.category_callback)
		rospy.Subscriber(TTS_QUERY_TOPIC, TtsQuery, self.recieve_question)
        rospy.Subscriber(GUESSED_OBJECT_TOPIC, String, self.guessed_category_callback) 
        rospy.sleep(1)
        
    def category_callback(self,category):
		self.guessed_category = category.data

    def _load_test_data(self):
        """ Loads test data and images"""
        image_pathes = [self.image_folder + img for img in random.shuffle(os.listdir(self.image_folder))[0:self.num_games]]
        for image_path in image_pathes: 
            self.images.append(cv2.imread(image_path,cv2.IMREAD_COLOR))

    def guessed_category_callback(self,guess):
        '''Callback for segmentation topic'''
        self.guessed_category = guess.data
    
    def _send_compressed_image(self,img):
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "png"
        msg.data = np.array(cv2.imencode('.png', img)[1]).tostring()
        self.image_pub.publish(msg)
		
	def _send_answer(self):
		answer_to_question = raw_input(self.question + ": ")
		tts_query_object = {text: answer_to_question, uid: 1, answer_type: 0}
		self.answer_pub.pub(json.dumps(data))
	
	def recieve_question(self, question):
		self.question = question.data
    
    def play_game(self,img):
        self._send_compressed_image(self,img)
		for i in range(0,5):
			self._send_answer()
			self.rate.sleep()
		return self.guessed_category
		
    def play_games(self)
        return [self.play_game(self.images[i]) for i in range(0,self.num_games)]
    


    

def main(args):
    '''Loads images and posts the corresponding segmentation rates'''
    # Parse arguments
    test = MSCOCOTest(args.dataset,args.num)
    print(test.play_games())

 
def parser():
    ''' Command Line Parser'''
    arg_fmt = argparse.RawDescriptionHelpFormatter
    arg_parser = argparse.ArgumentParser(formatter_class=arg_fmt, description=main.__doc__)
    image_options = arg_parser.add_argument_group('Image options')
    image_options.add_argument('-d', '--dataset', required=True, help='Where is the data set you want to play with')
    image_options.add_argument('-n', '--num', required=True, help='How many games to play') # turn this into an option which will load the entire dataset
    arguments = arg_parser.parse_args(rospy.myargv()[1:])
    return arguments

if __name__ == '__main__':
    main(parser())
    rospy.spin()
