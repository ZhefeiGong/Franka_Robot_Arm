### init
source /home/user1/Jeffrey/miniforge3/bin/activate
conda activate robot_infra
cd /home/user1/Jeffrey/Franka_Robot_Arm
source catkin_ws/devel/setup.bash
clear


### test 
python franka_ctrl/test.py --robot_ip=192.168.3.20 --load_gripper=True

### Test gripper controller
python franka_ctrl/gripper_ctrl.py

### Test cartesian impedance controller
python franka_ctrl/joints_ctrl.py

### Test franka state listener
python franka_ctrl/state_lstn.py


