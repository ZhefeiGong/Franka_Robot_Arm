#@time: May.28th 2025
#func: execute the teleoperation system
#@copyright: 
# - Zhefei(Jeffrey) Gong

import time
import rospy
import numpy as np
from franka_ctrl.joints_ctrl import FrankaJointsController
from franka_ctrl.gripper_ctrl import FrankaGripperController
from franka_ctrl.state_lstn import FrankaStateListener
from space_mouse import SpaceMouse
from utils import quat_to_axis, axis_to_euler
# import keyboard


GP_OPEN = 0
GP_CLOSE = 1
is_euler = True
V_AUG = 2.5
W_AUG = 8


def speedL(joints_controller, motion_state, robot_state, acceleration = 0.05, span = 0.05, dt=0.01, is_euler=True):
    steps = int(span / dt)
    pos = np.array(robot_state[:3])
    quat = np.array(robot_state[3:])
    euler = axis_to_euler(quat_to_axis(quat))
    v = np.array(motion_state[:3]) * V_AUG
    w = np.array(motion_state[3:]) * W_AUG
    for _ in range(steps):
        pos += v * dt
        euler += w * dt
        joints_controller.move(np.concatenate((pos, euler)), is_euler)
        time.sleep(dt)


def teleop():
    
    ## Space Mouse
    print("\033[33m[INFO] Starting Space Mouse ... \033[0m")
    sp_mouse = SpaceMouse()
    sp_mouse.start()

    ## Controller and Listener
    print("\033[33m[INFO] Starting Robor Server ... \033[0m")
    joints_controller = FrankaJointsController()
    gripper_controller = FrankaGripperController()
    gripper_controller.open()
    gripper_status = GP_OPEN
    state_listener = FrankaStateListener()
    time.sleep(2)

    try:
        ## Begin to tele-operate
        print("\033[33m[INFO] Teleoperating ... \033[0m")
        # print("[INFO] Teleoperating and Press ESC to terminate...")
        while True:
            
            # Read motion state from SpaceMouse
            motion_state = sp_mouse.get_motion_state_transformed()
            # print("Current motion state" , motion_state)
            robot_state = state_listener.get_cartesian_pose()
            # print("Current robot state" , robot_state)
            
            # Send command to robot 
            speedL(
                joints_controller, 
                motion_state, 
                robot_state, 
                acceleration = 0.05, 
                span = 0.05, 
                dt = 0.01, 
                is_euler=True,
            )
            
            # Send command to grasp
            if sp_mouse.is_button_pressed(0) and gripper_status == GP_CLOSE:
                gripper_controller.open()
                gripper_status = GP_OPEN
            elif sp_mouse.is_button_pressed(1) and gripper_status == GP_OPEN:
                gripper_controller.close()
                gripper_status = GP_CLOSE
            
            # # Monitor the exist
            # if keyboard.is_pressed("esc"):
            #     print("[INFO] Stopping robot")
            #     sp_mouse.stop()
            #     break
            
            # Wait awhile before proceeding 
            time.sleep(1/100)
            
    except KeyboardInterrupt:
        # Handle graceful shutdown here
        print("\033[33m[INFO] Stopping robot \033[0m")
        sp_mouse.stop()


if __name__ == "__main__":
    rospy.init_node('robot_teleop_info_pub_node', anonymous=True)
    teleop()
