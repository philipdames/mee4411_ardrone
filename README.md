# mee4411_ardrone
Code to use the Parrot AR.Drone in MEE4411: Introduction to Mobile Robotics

## Code Installation instructions
**Note: This assumes you have Ubuntu 16.04 and have installed the [mee4411_turtlebot3 package](https://github.com/philipdames/mee4411_turtlebot3) using the recommneded directions (so that your ROS workspace is in ~/catkin_ws).**

Download and run the installation script using the following commands:
```
wget https://raw.githubusercontent.com/philipdames/mee4411_ardrone/master/setup.sh
chmod 755 setup.sh
bash setup.sh
```    
This will download and install this package, the [rotors_simulator](https://github.com/TempleRAIL/rotors_simulator), and their dependencies, all of which are needed in order to run the simulations for this class.

## Test your simulation
Run the following command to ensure that your simulation environment is working properly. 
```
source ~/.bashrc
roslaunch ardrone_phase1 phase1.launch
```
You should see the following screen appear, with a set of blocks in the environment and a drone in the lower right corner:
![rviz screenshot of AR.Drone simulation](https://github.com/philipdames/mee4411_ardrone/blob/master/rviz_screenshot.png)
