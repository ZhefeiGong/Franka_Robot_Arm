#@time: May.28th 2025
#func: test the teleoperation system
#@copyright: 
# - Zhefei(Jeffrey) Gong

import sys
import rospy
import time
import subprocess
from absl import app, flags
from teleop_spm.teleop import teleop

FLAGS = flags.FLAGS
flags.DEFINE_string("robot_ip", None, "IP address of the robot.", required=True)

def main(_):
    try:
        input("\033[33m[INFO] Press enter to start roscore and the impedance controller \033[0m")
        try:
            roscore = subprocess.Popen('roscore')
            time.sleep(2)
        except:
            pass
        impedence_controller = subprocess.Popen(['roslaunch', 'serl_franka_controllers', 'impedance.launch',
                                                f'robot_ip:={FLAGS.robot_ip}', f'load_gripper:=True'],
                                                stdout=subprocess.PIPE)
        rospy.init_node('franka_teleop_api')
        time.sleep(2)
        
        input("\033[33m[INFO] Press enter to begin teleoperation \033[0m")
        teleop()
        
        impedence_controller.terminate()
        roscore.terminate()
        sys.exit()
    except:
        rospy.logerr("\033[33m[INFO] Error occured. Terminating the controller \033[0m")
        impedence_controller.terminate()
        roscore.terminate()
        sys.exit()

if __name__ == "__main__":
    app.run(main)