#! /usr/bin/env python2
''' Announce selected object once IRL-1 has pointed to it'''

import rospy
from queue import Queue, Empty
from std_msgs.msg import Bool
from std_msgs.msg import String
from devine_config import topicname


NODE_NAME = 'announce_object'
TOPIC_OBJECT_CATEGORY = topicname('guess_category')
TOPIC_SNIPS = topicname('tts_answer')
TOPIC_IS_POINTING_OBJECT = topicname('is_pointing_object') # add to paramters.json
TOPIC_IS_END_OF_GAME = topicname('end_of_game') # add to parameters.json

def category_callback(data):
	'''Callback for the category topic'''
    if categories.full():
        categories.get()

    categories.put(data.data)
	
def is_pointing_callback(data):
	'''Callback for the is_pointing topic'''
    if is_pointing.full():
        is_pointing.get()

    is_pointing.put(data.data)

def main(args):
    ''' Checks if selected object has been pointed to and announces its category '''
	rospy.init_node(NODE_NAME)
	global is_pointing = Queue(1)
	global categories = Queue(1)
	rospy.Subscriber(TOPIC_OBJECT_CATEGORY, String, category_callback)
	rospy.Subscriber(TOPIC_IS_POINTING_OBJECT, Bool, is_pointing_callback)
    end_of_game = rospy.Publisher(TOPIC_OBJECT_CATEGORY, Bool, queue_size=1)
	snips = rospy.Publisher(TOPIC_SNIPS, String, queue_size=1)

    rate = rospy.Rate(1) # TBD
    while not rospy.is_shutdown():
        if is_pointing.get() == True:
            object_type = categories.get()
			snips.publish(object_type)
			rospy.sleep(1)
			end_of_game.publish(True)

		rate.sleep()


if __name__ == '__main__':
    main()
	rospy.spin()
