#@time: May.28th 2025
#@func: initialize the environment for teleoperation
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

################################################ Launch Seperately ################################################

### Build a ROS node
roscore

### Activate the Cartesian Impedance Controller (by `serl_franka_controllers`) -> to execute
### template: roslaunch serl_franka_controllers impedance.launch robot_ip:=<RobotIP> load_gripper:=<true/false>
roslaunch serl_franka_controllers impedance.launch robot_ip:=192.168.3.20 load_gripper:=True

### Launch teleoperation
python teleop_spm/teleop.py


################################################ Launch in One File ################################################

### Lanch in one file
python teleop_spm/test.py --robot_ip=192.168.3.20


