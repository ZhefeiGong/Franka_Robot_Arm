

### get miniforge
curl -L -O "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh"
bash Miniforge3-$(uname)-$(uname -m).sh

### activate miniforge
source /home/franka/jeffrey/miniforge3/bin/activate

### add ssh key 
eval "$(ssh-agent -s)"
chmod 600 ~/.ssh/id_rsa_ubuntu
ssh-add ~/.ssh/id_rsa_ubuntu
git config --global user.email "zhefeigong@gmail.com"
git config --global user.name "zhefeigong"
git config user.name
git config user.email
git config --list



