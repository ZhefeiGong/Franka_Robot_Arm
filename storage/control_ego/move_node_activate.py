import sys
import rospy
import numpy as np
from absl import app, flags
from std_msgs.msg import String  # 用于抓夹控制
import os
from move_control import FrankaController

# 设置 FLAGS 参数
FLAGS = flags.FLAGS
flags.DEFINE_string("robot_ip", None, "IP address of the robot.", required=True)
flags.DEFINE_string("load_gripper", 'false', "Whether or not to load the gripper.")

def main(_):
    try:
        # 读取参数
        robot_ip = FLAGS.robot_ip
        load_gripper = FLAGS.load_gripper

        # 创建 FrankaController 实例
        franka = FrankaController(robot_ip, load_gripper)

        # 启动控制器和初始化 ROS
        franka.init_node()

        # 移动到初始位置
        # input("\033[33m\nPress enter to move the robot to the initial position.\033[0m")
        initial_position = [0.5, 0, 0.2]
        initial_orientation = [np.pi, 0, np.pi / 2]
        # franka.move_to(initial_position, initial_orientation)

        # 控制抓夹开合
        #franka.gripper_action("open")
        #rospy.sleep(1)
        # franka.gripper_action("close")

        # # 设置参考限制值
        # franka.set_reference_limits()

        # 演示移动控制和抓夹动作
        input("\033[33mPress enter to start moving the robot arm up and control the gripper.\033[0m")
        for i in range(10):
            position = [0.5, 0, 0.2 + i * 0.02]
            franka.move_to(position, initial_orientation)
            rospy.sleep(0.2)
        franka.gripper_action("open")

        # 重置机械臂位置
        input("\033[33m\nPress enter to reset the robot arm back to the initial pose.\033[0m")
        for i in range(10):
            position = [0.5, 0, 0.4 - i * 0.02]
            franka.move_to(position, initial_orientation)
            rospy.sleep(0.1)

        # 停止控制器并退出
        input("\033[33m\nPress enter to exit the test and stop the controller.\033[0m")
        franka.stop_controller()
        sys.exit()

    except Exception as e:
        rospy.logerr(f"程序出错: {e}")
        sys.exit(1)


if __name__ == "__main__":
    app.run(main)