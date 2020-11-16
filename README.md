# Active-Suspension
Codebase of the Active Suspension Research Paper


## 1. Installation and Setup

* Clone the repository and copy the `lsd/` folder in your `$WORKSPACE/src` directory:

      git clone https://github.com/Mars-Rover-Manipal/Active-Suspension.git && cp -r Active-Suspension/lsd/ $WORKSPACE/src

* Build the package in your workspace:

      cd $WORKSPACE/ && catkin_make
      
* To prevent repeated sourcing of your `devel/` folder of your workspace, run the command to add a sourcing command to your bash script:

      echo "source $WORKSPACE_DIR/devel/setup.bash" >> ~/.bashrc && exec bash
     

## 2. RViz 

* To launch the URDF in RViz:

      roslaunch lsd display.launch
      
### Voilà!

<img src ="https://user-images.githubusercontent.com/45683974/95745801-5504f680-0cb3-11eb-9a33-c9e3e9351466.png"/>
    
    
## 3. Gazebo

* To launch the SDF in Gazebo:

      roslaunch lsd custom_world.launch     
     
### Voilà!

<img src ="https://user-images.githubusercontent.com/45683974/95745912-90072a00-0cb3-11eb-9149-91e60f53d4f7.jpg"/> 


## 4. Dependencies:

* `hector_gazebo_plugins`
* `effort-controllers`
* `pcl-*`

* To install:

      sudo apt-get install ros-melodic-hector-gazebo-plugins && sudo apt-get install ros-melodic-effort-controllers && sudo apt-get install ros-melodic-pcl-*


## 5. Script and Functionality Description:

Click on the [link](https://github.com/Mars-Rover-Manipal/Active-Suspension/tree/main/lsd/scripts) to view description of codebase:





