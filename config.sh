
### get miniforge
curl -L -O "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh"
bash Miniforge3-$(uname)-$(uname -m).sh

### activate miniforge
source /home/franka/jeffrey/miniforge3/bin/activate
source /home/Jeffrey/miniforge3/bin/activate

### add ssh key 
eval "$(ssh-agent -s)"
chmod 600 ~/.ssh/id_rsa_ubuntu
ssh-add ~/.ssh/id_rsa_ubuntu
git config --global user.email "zhefeigong@gmail.com"
git config --global user.name "zhefeigong"
git config user.name
git config user.email
git config --list

### ubuntu driver check
lspci | grep -i vga
ls /usr/src | grep nvidia
lsmod | grep nvidia

### ubuntu driver install
ubuntu-drivers devices
sudo add-apt-repository ppa:graphics-drivers/ppa
sudo apt-get update
sudo apt-get install nvidia-driver-535
sudo reboot

### image check
dpkg --get-selections | grep linux-image
uname -a
uname -r

### 'Shift' -> get into initial ubuntu panel -> to exchange the following kernels
reboot
linux-image-5.15.0-139-generic # for nvidia
linux-image-5.15.76-rt53 # for franka

### libfranka and franka_ros check
dpkg -l | grep libfranka
dpkg -l | grep franka_ros
ldconfig -p | grep libfranka
ros-noetic-libfranka
ros-noetic-franka-ros
sudo apt-get remove "*franka*"
apt-cache policy ros-noetic-libfranka




