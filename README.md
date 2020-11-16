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
python-pip python3-vcstool python3-pyqt4 \
pyqt5-dev-tools \
libbluetooth-dev libspnav-dev \
pyqt4-dev-tools libcwiid-dev \
cmake gcc g++ qt4-qmake libqt4-dev \
libusb-dev libftdi-dev \
python3-defusedxml python3-vcstool \
ros-noetic-octomap-msgs        \
ros-noetic-joy                 \
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
sudo pip3 install -e .

```
#### Run bash files, build the ros workspace:
```
cd gym-gazebo/gym_gazebo/envs/installation
bash setup_noetic.bash
```
## Kill Background Processes

* Add the alias to your bash script: (One time process)

      echo "alias killgazebogym='killall -9 rosout roslaunch rosmaster gzserver nodelet robot_state_publisher gzclient'" >> ~/.bashrc
      
* When you want to kill all the gazebo processes:

      source ~/.bashrc
   
