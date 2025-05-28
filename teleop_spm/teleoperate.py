#!/usr/bin/env python

# from rtde_control import RTDEControlInterface
# from rtde_receive import RTDEReceiveInterface
# from rtde_io import RTDEIOInterface as RTDEIO
# import time
# import rospy
# import numpy as np
# from std_msgs.msg import Float64MultiArray
# import tele_ctrl_jeff
# from robotiq_gripper import RobotiqGripper
# from space_mouse import SpaceMouse
# from utils import axis_to_euler,axis_to_quat
# # define robot parameters
# ROBOT_HOST = "192.168.2.6"

from franka_ctrl.joints_ctrl import FrankaJointsController
from franka_ctrl.gripper_ctrl import FrankaGripperController
from franka_ctrl.state_lstn import FrankaStateListener
from space_mouse import SpaceMouse
from utils import euler_to_axis, quat_to_axis

GP_OPEN = 0
GP_CLOSE = 1
is_euler = True

def run():
    
    ## Node
    rospy.init_node('robot_teleop_info_pub_node', anonymous=True)

    ## Space Mouse
    print("[INFO] Starting Space Mouse...")
    sp_mouse = SpaceMouse()
    sp_mouse.start()

    ## Controller and Listener
    print("[INFO] Starting Robor Server...")
    joints_controller = FrankaJointsController()
    gripper_controller = FrankaGripperController()
    state_listener = FrankaStateListener()
    time.sleep(1)
    
    # pose = [0.5, 0, 0.2+i*0.02, np.pi, 0, np.pi/2]
    # joints_controller.move(pose, is_euler)
    
    # ## Publisher
    # pose_pub = rospy.Publisher('robot/pose', Float64MultiArray, queue_size=10)
    
    try:
        while True:

            # if rtde_rcv.getRobotMode() == 7:

            # Read motion state from SpaceMouse
            motion_state = sp_mouse.get_motion_state_transformed()
            # print("Current motion state" , motion_state)

            robot_state = state_listener.get_cartesian_pose()
            print("Current robot state" , robot_state)
            
            # # send command to robot 
            # rtde_ctl.speedL(motion_state, acceleration = 0.05, time = 0.05) # adjust the acceleration if required | 1.5 | 1.0
            
            # # get tcp velocity of robot
            # actual_velocity = rtde_rcv.getActualTCPSpeed()
            # actual_velocity = [0 if abs(x) < 0.01 else x for x in actual_velocity] #filter out extremely small numbers

            # # get tcp pose of robot
            # actual_pose = rtde_rcv.getActualTCPPose()
            # axis_cart = np.array(actual_pose[:3])
            # axis_angle = np.array(actual_pose[3:])
            # euler = axis_to_euler(axis_angle)
            # quat = axis_to_quat(axis_angle)
            # print("Current pose | euler" , np.stack([axis_cart,euler]))
            
            if sp_mouse.is_button_pressed(0):
                gripper_status = GP_OPEN
                gripper_controller.open()
            if sp_mouse.is_button_pressed(1):
                gripper_status = GP_CLOSE
                gripper_controller.close()
            
            # print("Gripper Position (0 to 255): ", gripper.get_current_position())
            
            # pose_msg = Float64MultiArray()
            # pose_msg.data = axis_cart.tolist() + quat.tolist() + list([float(gripper_status)])
            # pose_pub.publish(pose_msg)
            
            # wait awhile before proceeding 
            time.sleep(1/100)
            
            # else:
            #     print("[WARNING] Robot is not ready.")
            #     time.sleep(1)  # Wait longer if robot is not ready
    
    except KeyboardInterrupt:
        # Handle graceful shutdown here
        print("Stopping robot")
        sp_mouse.stop()

if __name__ == "__main__":
    run()
