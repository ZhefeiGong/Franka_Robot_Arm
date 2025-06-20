#@time: Jan.13th 2025
#@func: the controller for franka gripper
#@copyright: 
# - https://github.com/rail-berkeley/serl 
# - Zhefei(Jeffrey) Gong

import rospy
import time
from franka_gripper.msg import GraspActionGoal, MoveActionGoal
from sensor_msgs.msg import JointState


class FrankaGripperController():
    def __init__(self):
        super().__init__()
        self.grippermovepub = rospy.Publisher(
            "/franka_gripper/move/goal", MoveActionGoal, queue_size=1
        )
        self.grippergrasppub = rospy.Publisher(
            "/franka_gripper/grasp/goal", GraspActionGoal, queue_size=1
        )

    def open(self):
        """Open the gripper"""
        msg = MoveActionGoal()
        msg.goal.width = 0.09 # 0.09->single-object | 0.075->multi-object
        msg.goal.speed = 0.3
        self.grippermovepub.publish(msg)
    
    def close(self):
        """Close the gripper"""
        msg = GraspActionGoal()
        msg.goal.width = 0.01
        msg.goal.speed = 0.3
        msg.goal.epsilon.inner = 1
        msg.goal.epsilon.outer = 1
        msg.goal.force = 130
        self.grippergrasppub.publish(msg)

    def move(self, position: int):
        """Move the gripper to a specific position in range [0, 255]"""
        msg = MoveActionGoal()
        msg.goal.width = float(position / (255 * 10))  # width in [0, 0.1]m
        msg.goal.speed = 0.3
        self.grippermovepub.publish(msg)


def test():
    """Test the gripper controller"""
    rospy.init_node('franka_gripper_controller')
    gripper_controller = FrankaGripperController()
    
    time.sleep(2)
    print('close')
    gripper_controller.close()
    time.sleep(2)
    print('open')
    gripper_controller.open()
    time.sleep(2)
    print('close to 100')
    gripper_controller.move(100)
    time.sleep(2)
    print('open')
    gripper_controller.open()


if __name__ == "__main__":
    test()



