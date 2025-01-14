# 🦾 Franka Emika Panda 🐼

### 🛠️ Installation

#### Install `libfranka` and `franka_ros`
build from the ROS repositories
```bash
sudo apt install ros-noetic-libfranka ros-noetic-franka-ros
```
build from source
* [libfranka](https://github.com/frankaemika/libfranka/blob/main/README.md)
* [franka_ros](https://frankaemika.github.io/docs/installation_linux.html#building-the-ros-packages)
#### Build `serl_franka_controllers`
```bash
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone git@github.com:rail-berkeley/serl_franka_controllers.git
cd catkin_ws
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 --pkg serl_franka_controllers
cd ..
source catkin_ws/devel/setup.bash
```
#### Build virtual env
```bash
cd robot_infra
conda create -n robot_infra python=3.9
conda activate robot_infra
pip install -e .
```
#### Check path
```bash
import package
print(package.__file__)
```

### 🚗 Run
* initialization
```bash
source /home/franka/jeffrey/miniforge3/bin/activate
conda activate robot_infra
cd /home/franka/jeffrey/Franka_Robot_Arm
```
* run
```bash
pytho test.py --robot_ip=192.168.3.20
```
* close
```bash
killall -9 roscore
killall -9 rosmaster
```


### 📖 Reference
* [fmb](https://github.com/rail-berkeley/fmb/tree/main)
* [serl_franka_controllers](https://github.com/rail-berkeley/serl_franka_controllers)
* [serl](https://github.com/rail-berkeley/serl)
* [hi-serl](https://github.com/rail-berkeley/hil-serl)



