#! /usr/bin/env python2
''' Announce selected object once IRL-1 has pointed to it'''

import rospy
from Queue import Queue, Empty
from std_msgs.msg import Bool
from std_msgs.msg import String
from devine_config import topicname


NODE_NAME = 'announce_object'
TOPIC_OBJECT_CATEGORY = topicname('guess_category')
TOPIC_SNIPS = topicname('tts_answer')
TOPIC_IS_POINTING_OBJECT = topicname('is_pointing_object') # add to paramters.json
TOPIC_IS_END_OF_GAME = topicname('end_of_game') # add to parameters.json
global current_category 
global pointing_state
def category_callback(data):
	'''Callback for the category topic'''
	current_category = data.data
	
def is_pointing_callback(data):
	'''Callback for the is_pointing topic'''
	print(data.data)
	pointing_state = data.data

def main():
	''' Checks if selected object has been pointed to and announces its category '''
	rospy.init_node(NODE_NAME)
	current_category = ""
	pointing_state = False
	rospy.Subscriber(TOPIC_OBJECT_CATEGORY, String, category_callback)
	rospy.Subscriber(TOPIC_IS_POINTING_OBJECT, Bool, is_pointing_callback)
	end_of_game = rospy.Publisher(TOPIC_IS_END_OF_GAME, Bool, queue_size=1)
	snips = rospy.Publisher(TOPIC_SNIPS, String, queue_size=1)
	print("Here")
	rospy.sleep(1)
	rate = rospy.Rate(1) # TBD
	while not rospy.is_shutdown():
		if pointing_state == True:
			print("Cond met")
			snips.publish(current_category)
			rospy.sleep(1)
			end_of_game.publish(True)
		print("Loopy")
		rate.sleep()
	print("Loop exited")


if __name__ == '__main__':
	main()
	rospy.spin()
