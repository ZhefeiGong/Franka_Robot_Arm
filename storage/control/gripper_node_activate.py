#!/usr/bin/env python

import rospy
import subprocess
from gripper_control import GripperControl
from std_msgs.msg import String  # 用于订阅其他节点的消息
import sys
import time


class GripperNode:
    def __init__(self, robot_ip):
        """初始化 GripperNode"""
        # 启动 franka_gripper.launch
        self.start_franka_gripper(robot_ip)

        # 初始化 GripperControl 实例
        self.gripper = GripperControl()

        # 等待发布器连接
        rospy.sleep(1)

        # 订阅控制指令主题
        self.subscriber = rospy.Subscriber("/gripper_control", String, self.control_callback)

    def start_franka_gripper(self, robot_ip):
        """启动 franka_gripper.launch"""
        try:
            rospy.loginfo(f"启动 franka_gripper.launch，robot_ip: {robot_ip}")
            subprocess.Popen(
                ["roslaunch", "franka_gripper", "franka_gripper.launch", f"robot_ip:={robot_ip}"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            rospy.sleep(5)  # 等待 launch 脚本完成启动
        except Exception as e:
            rospy.logerr(f"启动 franka_gripper.launch 失败: {e}")
            sys.exit(1)

    def control_callback(self, msg):
        """接收其他节点的控制指令回调函数"""
        command = msg.data
        rospy.loginfo(f"接收到指令: {command}")

        try:
            if command == "open":
                self.gripper.open()
            elif command == "close":
                self.gripper.close()
            elif command.startswith("move"):
                parts = command.split()
                if len(parts) == 2 and parts[1].isdigit():
                    position = int(parts[1])
                    self.gripper.move(position)
                else:
                    rospy.logwarn("无效的 move 指令。格式应为: move <0-255>")
            else:
                rospy.logwarn("未知指令！可用指令: open, close, move <0-255>")
        except Exception as e:
            rospy.logerr(f"执行指令时出错: {e}")


def main():
    # 初始化 ROS 节点
    rospy.init_node('gripper_control_node', anonymous=True)

    # 从参数服务器获取 robot_ip
    robot_ip = rospy.get_param("~robot_ip", "192.168.3.20")  # 默认 IP 为 192.168.3.20

    # 初始化 GripperNode
    gripper_node = GripperNode(robot_ip)

    # 保持节点运行
    rospy.loginfo("Gripper 控制节点已启动。等待指令...")
    rospy.spin()


if __name__ == "__main__":
    main()