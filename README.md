# 🦾 Franka Emika Panda 🐼


## 🛠️ Franka Installation

### 🔧 Install Noetic for ubuntu20.04
set download source
```bash
sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.ustc.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'
```
add ros-key
```bash
sudo apt install curl 
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```
download noetic package
```bash
sudo apt update
sudo apt install ros-noetic-desktop-full
```
set global environment
```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc 
source ~/.bashrc
```
install tools for ros
```bash
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall generator python3-wstool build-essential
```
check the installation
```bash
roscore
rosrun turtlesim turtlesim_node
rosrun turtlesim turtle_teleop_key
```

### 🔧 Install `libfranka` and `franka_ros`
build from the ROS repositories
```bash
sudo apt-get update
sudo apt-get install ros-noetic-libfranka ros-noetic-franka-ros
```
build from source
* [libfranka](https://github.com/frankaemika/libfranka/blob/main/README.md)
* [franka_ros](https://frankaemika.github.io/docs/installation_linux.html#building-the-ros-packages)

### 🔧 Build `serl_franka_controllers`
```bash
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone git@github.com:rail-berkeley/serl_franka_controllers.git
cd catkin_ws
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 --pkg serl_franka_controllers
cd ..
source catkin_ws/devel/setup.bash
```

### 🔧 Build virtual env
```bash
cd robot_infra
conda create -n robot_infra python=3.9
conda activate robot_infra
pip install -e .
```

### 🔧 Check path
```bash
import package
print(package.__file__)
```


## 🚗 Franka Run
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


## 🛠️ Others Installation

### SpaceMouse-3Dconnexion

install the packages for SpaceMouse
```bash
pip install numpy termcolor atomics scipy
pip install git+https://github.com/cheng-chi/spnav
sudo apt install libspnav-dev spacenavd
sudo systemctl start spacenavd
```


### 📖 Reference
* [fmb](https://github.com/rail-berkeley/fmb/tree/main)
* [serl_franka_controllers](https://github.com/rail-berkeley/serl_franka_controllers)
* [serl](https://github.com/rail-berkeley/serl)
* [hil-serl](https://github.com/rail-berkeley/hil-serl)



## 🍀 Ubuntu Install w/ `real-time` Kernel


### 🔧 Download Source 

download the source from [here](https://www.kernel.org/pub/linux/kernel/)
```bash
# build foler
mkdir franka_env
cd franka_env
# download corresponding packages
curl -SLO https://www.kernel.org/pub/linux/kernel/v5.x/linux-5.15.76.tar.gz
curl -SLO https://www.kernel.org/pub/linux/kernel/projects/rt/5.15/older/patch-5.15.76-rt53.patch.gz
# unzip
tar xvzf linux-5.15.76.tar.gz
gunzip patch-5.15.76-rt53.patch.gz
# extract source code and insert the patch
cd linux-5.15.76
patch -p1 < ../patch-5.15.76-rt53.patch
```


### 🔧 Build Source

build the kernel from now on (refer to [here](https://blog.csdn.net/tiboyang/article/details/127700249))
```bash
# download neccessary packages
sudo apt-get install build-essential bc curl ca-certificates gnupg2 libssl-dev lsb-release libelf-dev bison flex dwarves zstd libncurses-dev
# borrow the config setting from current kernel
make olddefconfig
```
update the config of current kernel
```bash
make menuconig
```
* General Setup -> Preemption Model -> Fully Preemptible Kernel (Real-Time)
* Cryptographic API -> Certificates for signature checking -> Provide system-wide ring of trusted keys -> Additional X.509 keys for default system keyring -> remove  "debian/canonical-certs.pem" -> save to .config
open .config for further update
```bash
gedit .config
# change the following old commands
CONFIG_SYSTEM_TRUSTED_KEYS="debian/canonical-certs.pem"
CONFIG_SYSTEM_REVOCATION_KEYS="debian/canonical-revoked-certs.pem"
CONFIG_DEBUG_INFO_BTF=y
# to the following new commands
CONFIG_SYSTEM_TRUSTED_KEYS=""
CONFIG_SYSTEM_REVOCATION_KEYS=""
CONFIG_DEBUG_INFO_BTF=n
```
begin to build kernel locally
```bash
fakeroot make -j24 deb-pkg
```


### 🔧 Install Source

finally, install the built kernel with patch
```bash
# install
sudo dpkg -i ../linux-headers-5.15.76-rt53*.deb ../linux-image-5.15.76-rt53*.deb
# re-start the computer
sudo reboot
# check
uname -msr
```



