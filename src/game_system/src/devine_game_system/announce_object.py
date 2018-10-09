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
TOPIC_IS_POINTING_OBJECT = topicname('is_pointing_object')
TOPIC_IS_END_OF_GAME = topicname('end_of_game') 

class announce_node:

	def __init__(self):
		self.current_category = ""
		self.pointing_state = False

	def category_callback(self,data):
		'''Callback for the category topic'''
		self.current_category = data.data
	
	def is_pointing_callback(self,data):
		'''Callback for the is_pointing topic'''
		self.pointing_state = data.data

	def run_node(self):
		''' Checks if selected object has been pointed to and announces its category '''
		rospy.init_node(NODE_NAME)
		rospy.Subscriber(TOPIC_OBJECT_CATEGORY, String, self.category_callback)
		rospy.Subscriber(TOPIC_IS_POINTING_OBJECT, Bool, self.is_pointing_callback)
		end_of_game = rospy.Publisher(TOPIC_IS_END_OF_GAME, Bool, queue_size=1)
		snips = rospy.Publisher(TOPIC_SNIPS, String, queue_size=1)
		rospy.sleep(1)
		rate = rospy.Rate(1) # TBD
		while not rospy.is_shutdown():
			if self.pointing_state == True:
				snips.publish(self.current_category)
				#rospy.sleep(1)
				end_of_game.publish(True)
				self.pointing_state = False
			rate.sleep()


if __name__ == '__main__':
	node_inst = announce_node()
	node_inst.run_node()
	rospy.spin()
