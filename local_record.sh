### init
source /home/franka/jeffrey/miniforge3/bin/activate
conda activate robot_infra
cd /home/franka/jeffrey/Franka_Robot_Arm
source catkin_ws/devel/setup.bash
export PYTHONPATH=$PYTHONPATH:/home/franka/jeffrey/Franka_Robot_Arm/
clear

###
roscore

###
roslaunch serl_franka_controllers impedance.launch robot_ip:=192.168.3.20 load_gripper:=True

###
roslaunch franka_gripper franka_gripper.launch  robot_ip:=192.168.3.20

### Test 
python franka_ctrl/test.py --robot_ip=192.168.3.20 --load_gripper=True

### Test gripper controller
python franka_ctrl/gripper_ctrl.py

### Test cartesian impedance controller
python franka_ctrl/joints_ctrl.py

### Test franka state listener
python franka_ctrl/state_lstn.py

### Launch teleoperation
source /home/franka/jeffrey/miniforge3/bin/activate
conda activate robot_infra
cd /home/franka/jeffrey/Franka_Robot_Arm
source catkin_ws/devel/setup.bash
export PYTHONPATH=$PYTHONPATH:/home/franka/jeffrey/Franka_Robot_Arm/
clear
python teleop_spm/teleop.py

### Launch teleoperation test in one file
killall -9 roscore
killall -9 rosmaster
source /home/franka/jeffrey/miniforge3/bin/activate
conda activate robot_infra
cd /home/franka/jeffrey/Franka_Robot_Arm
source catkin_ws/devel/setup.bash
export PYTHONPATH=$PYTHONPATH:/home/franka/jeffrey/Franka_Robot_Arm/
clear
python teleop_spm/test.py --robot_ip=192.168.3.20



