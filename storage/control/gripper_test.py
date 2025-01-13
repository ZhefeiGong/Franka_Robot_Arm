#!/usr/bin/env python

import rospy
from gripper_control import GripperControl  # 确保路径正确
import sys
import time

def main():
    # 初始化 ROS 节点
    rospy.init_node('gripper_control_node', anonymous=True)
    
    # 创建 GripperControl 实例
    gripper = GripperControl()
    
    # 等待发布器连接
    rospy.sleep(1)
    
    # 示例操作：打开、关闭、移动
    try:
        while not rospy.is_shutdown():
            command = input("请输入命令 (open, close, move <0-255>, exit): ")
            if command == "open":
                gripper.open()
            elif command == "close":
                gripper.close()
            elif command.startswith("move"):
                parts = command.split()
                if len(parts) == 2 and parts[1].isdigit():
                    position = int(parts[1])
                    gripper.move(position)
                else:
                    print("无效的 move 命令。使用: move <0-255>")
            elif command == "exit":
                print("退出节点。")
                break
            else:
                print("未知命令。可用命令: open, close, move <0-255>, exit")
            rospy.sleep(1)  # 等待命令执行
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()