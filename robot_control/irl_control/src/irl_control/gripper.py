from commandPublisher import CommandPublisher

class Gripper(object):
    def __init__(self, robotName, gripper):
        if gripper is 'left':
            self._gripperUp = CommandPublisher(robotName, 'left_gripper_up_controller')
            self._gripperDown = CommandPublisher(robotName, 'left_gripper_down_controller')
        if gripper is 'right':
            self._gripperUp = CommandPublisher(robotName, 'right_gripper_up_controller')
            self._gripperDown = CommandPublisher(robotName, 'right_gripper_down_controller')
    
    def close(self, percentage=1):
        self._gripperDown.publish(percentage * 2.1 -1.7)
        self._gripperUp.publish(percentage * 2.1 - 1.7)
  
    def open(self, percentage=1):
        self._gripperDown.publish(percentage * -2.1 + 0.4)
        self._gripperUp.publish(percentage * -2.1 + 0.4)
