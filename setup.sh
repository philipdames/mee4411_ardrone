# Make sure packages are up to date

# Install dependencies
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full ros-kinetic-joy ros-kinetic-octomap-ros ros-kinetic-mavros ros-kinetic-mavlink ros-kinetic-mav-comm python-wstool python-catkin-tools protobuf-compiler libgoogle-glog-dev ros-kinetic-control-toolbox python-rtree python-numpy

# Clone class package and drone simulator into catkin workspace
cd ~/catkin_ws/src/
git clone https://github.com/TempleRAIL/rotors_simulator.git
git clone https://github.com/philipdames/mee4411_ardrone.git

# Build catkin workspace
cd ~/catkin_ws/
catkin_make

