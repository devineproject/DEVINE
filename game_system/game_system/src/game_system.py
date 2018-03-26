#!/usr/bin/env python2
'''State Machine to be used for DEVINE Game System.'''

from enum import Enum
import rospy
from std_msgs.msg import String
from transitions import Machine


class States(Enum):
    # pylint: disable=too-few-public-methods
    '''The possibles states of the machine'''
    __order__ = 'INITIALISATION SHOWING_INSTRUCTIONS MOVING_TO_SCENE TAKING_PICTURE TURNING_HEAD ' \
                'ASKING_QUESTION LISTENING ANALYSING GUESSING POINTING SAYING_GUESSED_OBJECT ' \
                'LISTENING_TO_FINAL_ANSWER SHOWING_EMOTION MOVING_BACK_TO_PLAYER'

    INITIALISATION = 'initialisation'
    SHOWING_INSTRUCTIONS = 'showing_instructions'
    MOVING_TO_SCENE = 'moving_to_scene'
    TAKING_PICTURE = 'taking_picture'
    TURNING_HEAD = 'turning_head'
    ASKING_QUESTION = 'asking_question'
    LISTENING = 'listening'
    ANALYSING = 'analysing'
    GUESSING = 'guessing'
    POINTING = 'pointing'
    SAYING_GUESSED_OBJECT = 'saying_guessed_object'
    LISTENING_TO_FINAL_ANSWER = 'listening_to_final_answer'
    SHOWING_EMOTION = 'showing_emotion'
    MOVING_BACK_TO_PLAYER = 'moving_back_to_player'

class GameSystem(object):
    '''The state machine itself'''
    states = []
    name = ''

    def __init__(self, name, pub):
        self.name = name
        self.pub = pub
        for state in States:
            self.states.append(state.value)

        self.machine = Machine(model=self, states=GameSystem.states,
                               send_event=True, initial='initialisation')

        # must use same states names
        self.machine.on_enter_showing_instructions('show_instructions')
        self.machine.on_enter_moving_to_scene('move_robot_to_scene')
        self.machine.on_enter_taking_picture('take_the_picture')
        self.machine.on_enter_turning_head('turn_head_towards_player')
        self.machine.on_enter_asking_question('ask_a_question')
        self.machine.on_enter_listening('listen_to_answer')
        self.machine.on_enter_analysing('analyse_answer')
        self.machine.on_enter_pointing('point_object')
        self.machine.on_enter_saying_guessed_object('say_guessed_object')
        self.machine.on_enter_listening_to_final_answer('listen_to_final_answer')
        self.machine.on_enter_showing_emotion('show_emotion')
        self.machine.on_enter_moving_back_to_player('move_back_to_player')

        self.machine.add_transition(trigger='ready', source='*',
                                    dest=States.SHOWING_INSTRUCTIONS.value)
        self.machine.add_transition(trigger='move_to_scene', source=States.SHOWING_INSTRUCTIONS.value,
                                    dest=States.MOVING_TO_SCENE.value)
        self.machine.add_transition(trigger='take_picture', source=States.MOVING_TO_SCENE.value,
                                    dest=States.TAKING_PICTURE.value)
        self.machine.add_transition(trigger='turn_head', source=States.TAKING_PICTURE.value,
                                    dest=States.TURNING_HEAD.value)
        self.machine.add_transition(trigger='ask_question', source=States.TURNING_HEAD.value,
                                    dest=States.ASKING_QUESTION.value)
        self.machine.add_transition(trigger='listen_answer', source=States.ASKING_QUESTION.value,
                                    dest=States.LISTENING.value)
        self.machine.add_transition(trigger='analyse', source=States.LISTENING.value,
                                    dest=States.ANALYSING.value)
        self.machine.add_transition(trigger='guess', source=States.ANALYSING.value,
                                    dest=States.GUESSING.value)
        self.machine.add_transition(trigger='point', source=States.GUESSING.value,
                                    dest=States.POINTING.value)
        self.machine.add_transition(trigger='say_object', source=States.POINTING.value,
                                    dest=States.SAYING_GUESSED_OBJECT.value)
        self.machine.add_transition(trigger='listen_final_answer',
                                    source=States.SAYING_GUESSED_OBJECT.value,
                                    dest=States.LISTENING_TO_FINAL_ANSWER.value)
        self.machine.add_transition(trigger='emotion',
                                    source=States.LISTENING_TO_FINAL_ANSWER.value,
                                    dest=States.SHOWING_EMOTION.value)
        self.machine.add_transition(trigger='move_back', source=States.SHOWING_EMOTION.value,
                                    dest=States.MOVING_BACK_TO_PLAYER.value)
        
    def initialisation(self):
        """ Initial state of the Machine, waiting everything is ready to go! """
        self.print_diagnostic("Booting DEVINE state Machine...")
        # add stuff here to make sure everything is ready

        self.print_diagnostic("All systems are ready to go!")

    def show_instructions(self, event):
        # pylint: disable=unused-argument
        """ Showing instructions and waiting for trigger to start the game """
        self.print_diagnostic("""\nWelcome to DEVINE GuessWhat?! Please follow the instructions above:
            1- Have fun
            2- (...)
        """)
        user_input = None

        try:
            user_input = raw_input
        except NameError:
            user_input = input

        answer = user_input("To start a game, please type 'start'")

        while answer.strip() != 'start':
            self.print_diagnostic("'"+ answer + "'" + " is not a valid entry...")
            answer = user_input("\nTo start a game, please type 'start'")

        self.print_diagnostic("Starting a new game!")


    def move_robot_to_scene(self, event):
        # pylint: disable=unused-argument
        """ Prepares the robot to take a picture of the scene """
        self.print_diagnostic("Moving to scene...")

    def take_the_picture(self, event):
        # pylint: disable=unused-argument
        """ Takes a picture of the scene """
        self.print_diagnostic("Taking picture...")

    def turn_head_towards_player(self, event):
        # pylint: disable=unused-argument
        """ Turns head towards player """
        self.print_diagnostic("Turning head to player...")

    def ask_a_question(self, event):
        # pylint: disable=unused-argument
        # pylint: disable=no-member
        """ Asks a question """
        self.print_diagnostic("Asking a question to player...")
        self.listen_answer()

    def listen_to_answer(self, event):
        # pylint: disable=unused-argument
        """ Waits for the answer """
        self.print_diagnostic("Listening to player...")

    def analyse_answer(self, event):
        """ Analysing the answers and take action depending if is ready or not to guess """
        self.print_diagnostic("Analysing answer...")
        ready = event.kwargs.get('ready_to_answer', False)
        self.print_diagnostic("Analysing done!")

        if ready:
            self.print_diagnostic("Ready to guess!")
            self.machine.set_state('guessing')
        else:
            self.print_diagnostic("I want to ask another question!")
            self.machine.set_state('asking_question')
            self.ask_a_question(None)

    def point_object(self, event):
        # pylint: disable=unused-argument
        """ Points the guessed object"""
        self.print_diagnostic("Pointing Object...")

    def say_guessed_object(self, event):
        # pylint: disable=unused-argument
        """ Say the guessed object to played """
        self.print_diagnostic("I guess object X")

    def listen_to_final_answer(self, event):
        # pylint: disable=unused-argument
        """ Listens to final answer """
        self.print_diagnostic("Listening to final answer")

    def show_emotion(self, event):
        # pylint: disable=unused-argument
        """ Showing emotion depending on final answer """
        self.print_diagnostic("Showing happy face!")

    def move_back_to_player(self, event):
        # pylint: disable=unused-argument
        """ Moving back to robot to player """
        self.print_diagnostic("Moving back to player...")

    def print_diagnostic(self, text):
        if not rospy.is_shutdown():
            try:
                message = text + " - %s" % rospy.get_time()
                rospy.loginfo(message)
                self.pub.publish(message)
            except rospy.ROSInterruptException:
                pass
