# 🦾 Franka Emika Panda 🐼

### Install `libfranka` and `franka_ros`
build from the ROS repositories
```bash
sudo apt install ros-noetic-libfranka ros-noetic-franka-ros
```
build from source
* [libfranka](https://github.com/frankaemika/libfranka/blob/main/README.md)
* [franka_ros](https://frankaemika.github.io/docs/installation_linux.html#building-the-ros-packages)


### Build `serl_franka_controllers`
```bash
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone git@github.com:rail-berkeley/serl_franka_controllers.git
cd catkin_ws
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 --pkg serl_franka_controllers
cd ..
source catkin_ws/devel/setup.bash
```


### Check Path
```bash
import package
print(package.__file__)
```


### Run

```bash
source /home/franka/jeffrey/miniforge3/bin/activate
conda activate robot_infra
cd /home/franka/jeffrey/Franka_Robot_Arm/robot_infra

# python franka_server.py --robot_ip=192.168.3.20
pytho test.py --robot_ip=192.168.3.20

```

```bash
curl -X POST http://127.0.0.1:5000/activate_gripper                                 # Activate gripper
curl -X POST http://127.0.0.1:5000/close_gripper                                    # Close gripper
curl -X POST http://127.0.0.1:5000/open_gripper                                     # Open gripper

curl -X POST http://127.0.0.1:5000/getpos                                           # Print current end-effector pose
curl -X POST http://127.0.0.1:5000/getstate

curl -X POST http://127.0.0.1:5000/jointreset                                       # Perform joint reset
curl -X POST http://127.0.0.1:5000/precision_mode                                   # Change the impedance controller to precision mode
curl -X POST http://127.0.0.1:5000/compliance_mode                                  # Change the impedance controller to compliance mode
curl -X POST http://127.0.0.1:5000/stopimp                                          # Stop the impedance controller
curl -X POST http://127.0.0.1:5000/startimp                                         # Start the impedance controller (**Only run this after stopimp**)

```

```bash
killall -9 roscore
killall -9 rosmaster
```


### Reference
* [FMB](https://github.com/rail-berkeley/fmb/tree/main)
* [serl_franka_controllers](https://github.com/rail-berkeley/serl_franka_controllers)



