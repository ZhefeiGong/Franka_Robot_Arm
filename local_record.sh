### init
source /home/user1/Jeffrey/miniforge3/bin/activate
conda activate robot_infra
cd /home/user1/Jeffrey/Franka_Robot_Arm
source catkin_ws/devel/setup.bash

### test 
python franka_ctrl/test.py --robot_ip=192.168.3.20


