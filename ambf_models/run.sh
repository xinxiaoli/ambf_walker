source ~/.bashrc
export ROS_MASTER_URI=http://pop-os:11311/
source ~/ambf/build/devel/setup.bash
cd ~/ambf/bin/lin-x86_64
./ambf_simulator -a /home/nathanielgoldfarb/catkin_ws/src/ambf_walker/ambf_models/LARRE2.yaml
