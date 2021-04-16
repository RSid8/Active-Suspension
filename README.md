# Active-Suspension
Codebase of the Gazebo RL Closed- Chain Active Suspension System

## Installation

### Ubuntu 20.04

#### Basic requirements
- [**ROS noetic**](http://wiki.ros.org/noetic/Installation/Ubuntu) : Desktop-Full Install recommended, includes Gazebo 11.0.0.

#### Cloning Command

    git clone --single-branch --branch=gym-gazebo https://github.com/Mars-Rover-Manipal/Active-Suspension.git

### Installation

    cd Active-Suspension/ && ./helper_script.sh

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
