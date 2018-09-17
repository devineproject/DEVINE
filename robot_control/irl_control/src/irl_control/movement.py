import rospy

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool

TOPIC_GUESSWHAT_CONFIDENCE = '/confidence'
TOPIC_GUESSWHAT_SUCCEED = '/is_guesswhat_succeed'

class Movement(object):
    def __init__(self, controller):
        self.controller = controller
        self.confidence_max = None
        self.is_guesswhat_succeed = None

        # TODO verify if mutex needed
        rospy.Subscriber(TOPIC_GUESSWHAT_CONFIDENCE, Float32MultiArray, self.confidence_callback)
        rospy.Subscriber(TOPIC_GUESSWHAT_SUCCEED, Bool, self.is_guesswhat_succeed_callback)


    def confidence_callback(self, msg):
        rospy.loginfo(msg.data)
        if msg.data:
            self.confidence_max = max(msg.data)


    def is_guesswhat_succeed_callback(self, msg):
        rospy.loginfo(msg.data)
        if msg.data != None:
            self.is_guesswhat_succeed = msg.data
            self.choose_move()


    def choose_move(self):
        seuil = 0.75
        if not self.is_guesswhat_succeed and self.confidence_max >= seuil:
            rospy.loginfo('Sad')
            self.head_down_shoulder_in()
            rospy.sleep(1)

        elif not self.is_guesswhat_succeed and self.confidence_max < seuil:
            rospy.loginfo('Disappointed')
            self.head_shake_hand_up()

        elif self.is_guesswhat_succeed and self.confidence_max < seuil:
            rospy.loginfo('Satisfied')
            self.head_up_arm_up()

        elif self.is_guesswhat_succeed and self.confidence_max >= seuil:
            rospy.loginfo('Happy')
            self.dab_left()
            rospy.sleep(3)
            self.dab_right()
            rospy.sleep(3)

        self.controller.move_init(5)


    def head_down_shoulder_in(self):
        right_joints_position = ui_to_traj([-0.22, 0.47, 0.18, -0.4])
        left_joints_position = ui_to_traj([-0.70, -0.62, -0.31, -0.67])
        head_joints_position = [-0.15, 0.79]
        time = 4

        self.controller.move({'head': head_joints_position,
                              'arm_left': left_joints_position,
                              'arm_right': right_joints_position
                             },
                             time)

        rospy.sleep(1)

        time = 2
        left_joints_position = ui_to_traj([-0.35, -0.62, -0.31, -0.67])
        self.controller.move({'arm_left': left_joints_position}, time)

        rospy.sleep(0.1)

        left_joints_position = ui_to_traj([-0.70, -0.62, -0.31, -0.67])
        self.controller.move({'arm_left': left_joints_position}, time)

        rospy.sleep(0.1)

        left_joints_position = ui_to_traj([-0.35, -0.62, -0.31, -0.67])
        self.controller.move({'arm_left': left_joints_position}, time)

        rospy.sleep(0.1)


    def head_shake_hand_up(self):
        right_joints_position = ui_to_traj([-0.75, -0.31, 0.31, -1.07])
        left_joints_position = ui_to_traj([-0.75, 0.31, -0.31, -1.07])
        head_joints_position = [0.4, 0.47]
        time = 3

        self.controller.move({'head': head_joints_position,
                              'arm_left': left_joints_position,
                              'arm_right': right_joints_position
                             },
                             time)

        time = 2
        head_joints_position = [-0.4, 0.47]
        self.controller.move({'head': head_joints_position}, time)

        rospy.sleep(0.1)

        head_joints_position = [0.4, 0.47]
        self.controller.move({'head': head_joints_position}, time)

        rospy.sleep(0.1)

        head_joints_position = [-0.4, 0.47]
        self.controller.move({'head': head_joints_position}, time)

        rospy.sleep(0.1)


    def head_up_arm_up(self):
        right_joints_position = ui_to_traj([-0.34, -0.22, 0, -2.48])
        left_joints_position = ui_to_traj([-0.34, 0.22, 0, -2.48])
        head_joints_position = [-0.5, -0.17]
        time = 5

        self.controller.move({'head': head_joints_position,
                              'arm_left': left_joints_position,
                              'arm_right': right_joints_position
                             },
                             time)
        time = 2

        head_joints_position = [0.5, -0.17]
        self.controller.move({'head': head_joints_position}, time)

        rospy.sleep(0.5)

        head_joints_position = [-0.5, -0.17]
        self.controller.move({'head': head_joints_position}, time)

        rospy.sleep(0.5)

        head_joints_position = [0.5, -0.17]
        self.controller.move({'head': head_joints_position}, time)

        rospy.sleep(0.5)


    def dab_left(self):
        right_joints_position = [-1.57, -1.87, 0, -0.16]
        left_joints_position = [0, -2.0, 1.57, 1.22]
        head_joints_position = [0.31, 0.79]
        time = 3

        self.controller.move({'head': head_joints_position,
                              'arm_left': left_joints_position,
                              'arm_right': right_joints_position
                             },
                             time)

    def dab_right(self):
        left_joints_position = [1.57, -1.87, 0, -0.16]
        right_joints_position = [0, -2, -1.57, 1.22]
        head_joints_position = [-0.31, 0.79]
        time = 3

        self.controller.move({'head': head_joints_position,
                              'arm_left': left_joints_position,
                              'arm_right': right_joints_position
                             },
                             time)

def ui_to_traj(joints_position):
    # in: UI Joints Position
    # L_elbow_tilt_joint, L_shoulder_pan_joint, L_shoulder_roll_joint, L_shoulder_tilt_joint
    # out: Trajectory Client
    # L_shoulder_pan_joint, L_shoulder_tilt_joint, L_shoulder_roll_joint, L_elbow_tilt_joint
    joints_position_out = [0, 0, 0, 0]
    joints_position_out[0] = joints_position[1]
    joints_position_out[1] = joints_position[3]
    joints_position_out[2] = joints_position[2]
    joints_position_out[3] = joints_position[0]

    return joints_position_out
