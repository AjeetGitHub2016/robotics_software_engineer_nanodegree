## Project 3 : Where am I?
Where Am I? is a localization project from Udacity Robotics Software Engineering nanodegree. In this project, I am utilizing ROS AMCL package to accurately localize a mobile robot inside a map in the Gazebo simulation environments.
### System Requirement
* Ubuntu 16.04
* ROS Kinetics
* Gazebo 7
#### You might require to install dependencies below:
* $ sudo apt-get update && sudo apt-get upgrade -y
* $ sudo apt-get install ros-kinetics-map-server
* $ sudo apt-get install ros-kinetics-amcl
* $ sudo apt-get install ros-kinetics-move-base
### Build and Launch
Clone project and initialize catkin workspace
* $ mkdir catkin_ws && cd catkin_ws
* $ git clone https://github.com/AjeetGitHub2016/robotics_software_nanodegree.git
* $ mv robotics_software_nanodegree/tree/main/Localization/Where_Am_I/src src
* $ cd src && catkin_init_workspace
* $ cd ..
* $ catkin_make

Launch the world and robot
* $ source devel/setup.bash
* $ roslaunch my_robot world.launch

Open another terminal
* $ source devel/setup.bash
* $ roslaunch my_robot amcl.launch

Open another terminal, and run the teleop node.
* $ source devel/setup.bash
* $ rosrun teleop_twist_keyboard teleop_twist_keyboard.py

Click on this terminal, type keyboard to navigate the robot around.
