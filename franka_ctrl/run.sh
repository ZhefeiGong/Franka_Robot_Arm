#@time: Jan.13th 2025
#@func: initialize the environment for executing
#@copyright: 
# - Zhefei(Jeffrey) Gong


###################################################################################################################

### Get into the folder
source /path/to/miniforge3/bin/activate
conda activate robot_infra
cd /path/to/Franka_Robot_Arm
source catkin_ws/devel/setup.bash
export PYTHONPATH=$PYTHONPATH:/path/to/Franka_Robot_Arm/
clear

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
rostopic pub /franka_gripper/move franka_gripper/MoveActionGoal "{goal: {width: 0.08, speed: 0.05}}" # seem to activate the gripper
rostopic pub /franka_gripper/move franka_gripper/MoveActionGoal "{goal: {width: 0.8, speed: 0.05}}" # seem to activate the gripper

###################################################################################################################

### Test gripper controller
python franka_ctrl/gripper_ctrl.py

### Test cartesian impedance controller
python franka_ctrl/joints_ctrl.py

### Test franka state listener
python franka_ctrl/state_lstn.py

###################################################################################################################






