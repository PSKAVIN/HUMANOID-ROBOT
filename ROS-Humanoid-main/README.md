# 🤖 Bioloid Humanoid - ROS1 Package

This repository contains a ROS1 package for controlling the **Bioloid Humanoid Robot**. It includes the source code, launch files, and configuration for operating the robot using the Robot Operating System (ROS).

  
## 🚀 Features

- ROS1-compatible motor control (DYNAMIXEL AX_18A)
- Sensor integration (IMU, Force sensors, etc.)
- Predefined motion sequences
- Humanoid walking and balancing logic
- Simulation-ready
- CPG walking mechanism

## Installation

Install ROS noetic in Ubuntu 20.04 with Gazebo-- https://wiki.ros.org/noetic/Installation/Ubuntu
Install Python3

## Dependencies

sudo apt-get install ros-noetic-dynamixel-sdk
sudo apt-get install ros-noetic-robot-state-publisher
sudo apt-get install ros-noetic-joint-state-publisher

## Clone Repository

mkdir ~p catkin_make
cd ~/catkin_ws
git clone https://github.com/MADESH-KUMAR-M/ROS-Humanoid.git

catkin_make
source devel/setup.bash

### Terminal 1

source devel/setup.bash
roslaunch typea_gazebo bioloid_gazebo.launch

### Terminal 2

source devel/setup.bash
rosrun typea_description walker_demo.py

## Credits

This project was developed as part of a team effort. Special thanks to the following contributors:

- Madesh Kumar M - [github](https://github.com/MADESH-KUMAR-M) - [Linkedln](https://www.linkedin.com/in/madesh-kumar)
- Sabareesan S - [github](https://github.com/Sabareesan-K-C) - [Linkedln](https://www.linkedin.com/in/sabareesan-k-c-b24920303/)
- Kavin Karthick P - [github](https://github.com/PSKAVIN) - [Linkedln](https://www.linkedin.com/in/kavin-karthick-p-333874316/)

  We worked together to bring the Bioloid Humanoid to life using ROS1.




