#@time: Jan.13th 2025
#@func: initialize the environment for executing
#@copyright: 
# - Zhefei(Jeffrey) Gong


### Build a ROS node
roscore


### Activate the Cartesian Impedance Controller (by `serl_franka_controllers`) -> to execute
### template: roslaunch serl_franka_controllers impedance.launch robot_ip:=<RobotIP> load_gripper:=<true/false>
roslaunch serl_franka_controllers impedance.launch robot_ip:=192.168.3.20 load_gripper:=True

# ### Activate the Joint Position Controller (by `serl_franka_controllers`) -> to reset
# rosparam set /target_joint_positions '[q1, q2, q3, q4, q5, q6, q7]'
# roslaunch serl_franka_controllers joint.launch robot_ip:=<RobotIP> load_gripper:=<true/false>

