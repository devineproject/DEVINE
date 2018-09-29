#!/usr/bin/env python2
'''Example of how to trigger different states on the State Machine'''
import rospy
from std_msgs.msg import String
from game_system import GameSystem
from devine_config import topicname

#Topics
GAME_SYSTEM_STATE = topicname('game_system_state')

if __name__ == "__main__":

    pub = rospy.Publisher(GAME_SYSTEM_STATE, String, queue_size=10)
    rospy.init_node('game_system', anonymous=False)
    # pylint: disable=no-member
    gamesystem = GameSystem("DEVINE State Machine", pub)

    gamesystem.initialisation()

    gamesystem.ready()

    gamesystem.move_to_scene()

    gamesystem.take_picture()

    gamesystem.turn_head()

    gamesystem.ask_question()

    gamesystem.analyse(ready_to_answer=True)

    gamesystem.point()

    gamesystem.say_object()

    gamesystem.listen_final_answer()

    gamesystem.emotion()

    gamesystem.move_back()

    rospy.signal_shutdown('End of game')
