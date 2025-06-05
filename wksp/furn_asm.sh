

# refer to https://github.com/ankile/robust-rearrangement
### install Isaac-Gym
conda create -n furn_asm python=3.8 -y
conda activate furn_asm
wget --no-check-certificate https://iai-robust-rearrangement.s3.us-east-2.amazonaws.com/packages/IsaacGym_Preview_4_Package.tar.gz
tar -xzf IsaacGym_Preview_4_Package.tar.gz
pip install -e /home/dingpengxiang/jeffrey/isaacgym/python --no-cache-dir --force-reinstall
### install furniture-bench
git clone --recursive git@github.com:ankile/robust-rearrangement.git
cd robust-rearrangement/furniture-bench
pip install -e .
pip install ipdb # extra
### test furniture bench
python -m furniture_bench.scripts.run_sim_env --furniture one_leg --scripted
python -m furniture_bench.scripts.run_sim_env --furniture lamp --scripted
python -m furniture_bench.scripts.run_sim_env --furniture round_table --scripted
python -m furniture_bench.scripts.run_sim_env --furniture one_leg --no-action
python -m furniture_bench.scripts.run_sim_env --furniture one_leg --input-device keyboard
### install robust-rearrangement
cd ..
pip install -e .


# refer to https://clvrai.github.io/furniture-bench/docs/getting_started/installing_furniture_bench.html#set-up-connection
### initialize serial number of realsense-d435i
export CAM_WRIST_SERIAL=335522072596
export CAM_FRONT_SERIAL=135122074283
export CAM_REAR_SERIAL=344322073523
### test connection of camera
python furniture_bench/scripts/run_cam_april.py
### calibrate 
python furniture_bench/scripts/calibration.py --target setup_front
python furniture_bench/scripts/calibration.py --target obstacle
python furniture_bench/scripts/calibration.py --target one_leg


