#@time: Jan.13th 2025
#@func: initialize the environment for executing
#@copyright: 
# - Zhefei(Jeffrey) Gong


###################################################################################################################

### Get into the folder
source /home/franka/jeffrey/miniforge3/bin/activate
conda activate robot_infra
cd /home/franka/jeffrey/Franka_Robot_Arm

### Build a ROS node
roscore

###################################################################################################################

### Activate the Cartesian Impedance Controller (by `serl_franka_controllers`) -> to execute
### template: roslaunch serl_franka_controllers impedance.launch robot_ip:=<RobotIP> load_gripper:=<true/false>
roslaunch serl_franka_controllers impedance.launch robot_ip:=192.168.3.20 load_gripper:=True

### Activate the Joint Position Controller (by `serl_franka_controllers`) -> to reset
rosparam set /target_joint_positions '[2.85704382, 0.00647726, -2.86199697, -2.37292058, 0.02808961, 2.3696481, -0.29082154]'
roslaunch serl_franka_controllers joint.launch robot_ip:=192.168.3.20 load_gripper:=False

### Activate the gripper of the franka
roslaunch franka_gripper franka_gripper.launch  robot_ip:=192.168.3.20

###################################################################################################################

### Test gripper controller
python /home/franka/jeffrey/Franka_Robot_Arm/franka_ctrl/gripper_ctrl.py

### Test cartesian impedance controller
python /home/franka/jeffrey/Franka_Robot_Arm/franka_ctrl/joints_ctrl.py

### Test franka state listener
python /home/franka/jeffrey/Franka_Robot_Arm/franka_ctrl/state_lstn.py

###################################################################################################################






