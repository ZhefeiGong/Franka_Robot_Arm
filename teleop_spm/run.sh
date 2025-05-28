#@time: May.28th 2025
#@func: initialize the environment for teleoperation
#@copyright: 
# - Zhefei(Jeffrey) Gong


###################################################################################################################

### Get into the folder
source /path/to/miniforge3/bin/activate
conda activate robot_infra
cd /path/to/Franka_Robot_Arm

### Build a ROS node
roscore

###################################################################################################################

### Activate the Cartesian Impedance Controller (by `serl_franka_controllers`) -> to execute
### template: roslaunch serl_franka_controllers impedance.launch robot_ip:=<RobotIP> load_gripper:=<true/false>
roslaunch serl_franka_controllers impedance.launch robot_ip:=192.168.3.20 load_gripper:=True

# ### Activate the Joint Position Controller (by `serl_franka_controllers`) -> to reset
# rosparam set /target_joint_positions '[2.85704382, 0.00647726, -2.86199697, -2.37292058, 0.02808961, 2.3696481, -0.29082154]'
# roslaunch serl_franka_controllers joint.launch robot_ip:=192.168.3.20 load_gripper:=False

### Activate the gripper of the franka
roslaunch franka_gripper franka_gripper.launch  robot_ip:=192.168.3.20

###################################################################################################################

python teleop_spm/teleoperate.py

