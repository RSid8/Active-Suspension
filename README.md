# Active-Suspension
Codebase of the Gazebo RL Closed- Chain Active Suspension System

## Installation

### Ubuntu 20.04

#### Basic requirements
- ROS noetic: Desktop-Full Install recommended, includes Gazebo 11.0.0 (http://wiki.ros.org/noetic/Installation/Ubuntu).
- Gazebo 11.0.0

#### Cloning Command

    git clone --single-branch --branch=gym-gazebo https://github.com/Mars-Rover-Manipal/Active-Suspension.git


#### ROS noetic related dependencies
```
sudo apt-get install \
ros-noetic-octomap-msgs        \
ros-noetic-joy                 \
ros-noetic-effort-controllers  \
ros-noetic-geodesy             \
ros-noetic-octomap-ros         \
ros-noetic-control-toolbox     \
ros-noetic-pluginlib	       \
ros-noetic-trajectory-msgs     \
ros-noetic-control-msgs	       \
ros-noetic-std-srvs 	       \
ros-noetic-nodelet	       \
ros-noetic-urdf		       \
ros-noetic-rviz		       \
ros-noetic-kdl-conversions     \
ros-noetic-eigen-conversions   \
ros-noetic-tf2-sensor-msgs     \
ros-noetic-pcl-ros \
ros-noetic-navigation \
ros-noetic-sophus
```

#### Install Python Packages:
```
sudo pip3 install gym
sudo apt-get install python3-skimage
sudo pip3 install h5py
sudo pip3 install torch-vision
```

#### Install gym-gazebo
```
cd Active-Suspension/ && sudo pip3 install -e gym-gazebo/

```
#### Run bash files, build the ros workspace:
```
cd gym-gazebo/gym_gazebo/envs/installation
./setup_noetic.bash
```

## Launch the simulation

* In an empty world: (by default)

      roslaunch lsd custom_world.launch
      
* In a world of your choice:

      roslaunch lsd custom_world.launch world_name:=world_name.world

## Kill Background Processes

* Add the alias to your bash script and source it: (One time process)

      echo "alias killgazebogym='killall -9 rosout roslaunch rosmaster gzserver nodelet robot_state_publisher gzclient'" >> ~/.bashrc && source ~/.bashrc
      
* When you want to kill all the gazebo processes:

      killgazebogym
   




