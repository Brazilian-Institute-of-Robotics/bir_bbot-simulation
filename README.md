# bir_bbot - BIR Balancing Robot

The **bir_bbot** feature simulation, provides a simulation for the _Bbot_ balancing robot with the _Gazebo simulator - ROS_.

**Keywords:** balancing robot, Gazebo, ROS

**Author: [Matheus França](https://github.com/MatheusFranca-dev) and [Lucas Souza](https://github.com/lucaslins0035)<br />
Advisor: [Marco Reis](https://github.com/mhar-vell)<br />
Affiliation: [BIR - Brazilian Institute of Robotics](https://github.com/Brazilian-Institute-of-Robotics)<br />**

# **Purpose of the Project**

Bbot, or _Balancing Robot_, is a self-balancing autonomous robot project. Our goal is to build a mobile robot operated via **ROS Noetic** capable of balancing and moving on two wheels. Furthermore, he must be able to read a TAG (fiducial framework). The TAG will send the robot a destination position to which it must autonomously navigate. To carry out navigation, this robot must be able to create a map of where it is and locate itself in it, allowing it to update its position throughout the mission and avoid obstacles while navigating to its objective. For more about this project see our web page, containing the steps for build this project, [LINK](https://mhar-vell.github.io/rasc/project-bbot/).

### Supported Versions

- **Noetic**: Built and tested under [ROS] Noetic and Ubuntu 20.04

### Dependencies 
- [ROS] : An open-source robot framework. (**Version == Noetic**)
- [ros_control](http://wiki.ros.org/ros_control) : ROS control interface.
- [Twist mux](http://wiki.ros.org/twist_mux) : Twist multiplexer, which multiplex several velocity commands (topics) and allows to priorize or disable them (locks).
- [Teleop twist keyboard](http://wiki.ros.org/teleop_twist_keyboard) : Generic keyboard teleop for twist robots.
- [BIR Marker Localization](https://github.com/Brazilian-Institute-of-Robotics/bir_marker_localization) : This package was made to help you find your robot with a marker. (**Clone into your src folder**)
- [Py trees](https://github.com/splintered-reality/py_trees) : PyTrees is a python implementation of behaviour trees designed to facilitate the rapid development of medium sized decision making engines for use in fields like robotics.
- [Robot localization](http://wiki.ros.org/robot_localization) : Provides nonlinear state estimation through sensor fusion of an abritrary number of sensors.
- [Navigation](http://wiki.ros.org/navigation) : ROS Navigation-Stack
- [Gmapping](http://wiki.ros.org/gmapping) : SLAM Gmapping for ROS
- [Move_base_flex](http://wiki.ros.org/move_base_flex) : Move Base Flex (MBF) is a backwards-compatible replacement for move_base.
- [Moveback Recovery](https://github.com/uos/mbf_recovery_behaviors) : Recovery behavior for navigation that moves the robot backwards.

Install dependencies by running:

    $ sudo apt install ros-noetic-py-trees* ros-noetic-navigation ros-noetic-gmapping  ros-noetic-robot-localization

    $ git clone https://github.com/magazino/move_base_flex.git

    $ git clone https://github.com/uos/mbf_recovery_behaviors.git

    $ git clone https://github.com/Brazilian-Institute-of-Robotics/bir_marker_localization.git

# **Table of Contents**
- [**bir_bbot**](#bir_bbot)
- [**Purpose of the Project**](#purpose-of-the-project)
    - [Supported Versions](#supported-versions)
    - [Dependencies](#dependencies)
- [**File System**](#file-system)
- [**Installation**](#installation)
	- [Building from Source:](#building-from-source)
	- [Example of Usage](#example-of-usage)
        - [Simulation](#simulation)
        - [Real](#real)
- [**License**](#license)
- [**Bugs & Feature Requests**](#bugs--feature-requests)

# **File System**

- [doc_resources](https://github.com/Brazilian-Institute-of-Robotics/bir_bbot-simulation/tree/simulation/doc_resources) : Support files, including Jupyter about the control systems of Bbot and images to support the readme.
- [bbot_control](https://github.com/Brazilian-Institute-of-Robotics/bir_bbot-simulation/tree/simulation/bbot_control) : Contains the controllers parameters to the robot.
- [lqr_controller](https://github.com/Brazilian-Institute-of-Robotics/bir_bbot-simulation/tree/simulation/lqr_controller) : The support package for _bbot_control_. Contains our ROS LQR controller.
- [bbot_description](https://github.com/Brazilian-Institute-of-Robotics/bir_bbot-simulation/tree/simulation/bbot_description) : Defines the Bbot URDF, Rviz and meshes.
- [bbot_gazebo](https://github.com/Brazilian-Institute-of-Robotics/bir_bbot-simulation/tree/simulation/bbot_gazebo) : Is the gazebo package for simulate the robot, sensors and the world objects.
- [bbot_perception](https://github.com/Brazilian-Institute-of-Robotics/bir_bbot-simulation/tree/simulation/bbot_perception) : The package for visual process.

# **Installation**

###  Building from Source:

Attention, if you haven't installed [ROS] yet, please check [Ubuntu install of ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu). Desktop-Full Install is the recommended one in order to work with this repository.  

**Building:**

First, lets create a catkin workspace.

    $ mkdir -p ~/catkin_ws/src

Then, clone **bir_bbot** inside your workspace source. For simulation, use the branch 'feature/perception_simulation'

	$ git clone git@github.com:Brazilian-Institute-of-Robotics/bir_bbot.git -b simulation

Now, just build your catkin workspace.

    $ cd ~/catkin_ws
    $ catkin_make

Don't forget to source your workspace before using it.
    
    $ source devel/setup.bash

## Example of Usage

Just Run

	$ roslaunch bbot_gazebo bbot_gazebo.launch rviz:=true

In order to move Bbot with a keyboard, use:

    $ rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=bbot/mobile_base_controller/cmd_vel

![](/doc_resources/Bbot.gif)
* _Example using RQT to move the robot (to be more visual)._

For Bbot navigation, use:

    $ roslaunch bbot_navigation navigation.launch

![](/doc_resources/bbot_nav.png)
* _Behaviour tree_

# **License**

<!-- Bir Bbot source code is released under a [MIT License](/LICENSE). -->

# **Bugs & Feature Requests**

Please report bugs and request features using the [Issue Tracker].

<!-- Hyperlinks -->
[ROS]: https://www.ros.org
[Issue Tracker]: https://github.com/Brazilian-Institute-of-Robotics/bir_bbot-simulation/issues
