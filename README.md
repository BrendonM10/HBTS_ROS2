# Haptic Bilateral Teleoperation System (Franka Panda + Geomagic Touch)

This project enables haptic bilateral teleoperation by integrating a Geomagic Touch haptic device with the Franka Emika Panda robot. Users can intuitively control the robot's motion in real time and receive haptic feedback that reflects physical interactions at the robot's end-effector. Built with ROS2 and C++, the system is designed for smooth integration and responsive performance.


## Prerequisites

### Hardware Requirements
- Franka Emika Panda robot (for physical deployment)
- Geomagic Touch haptic device
- Ubuntu 22.04 LTS workstation

### Software Requirements
- ROS2 Humble
- libfranka
- franka_ros2 (or your custom franka_gazebo setup)
- Geomagic Touch Driver (ROS2 version)
- A control strategy ( I have used a cartesian compliance controller)
- Gazebo (for simulation)

## Installation

git clone https://github.com/BrendonM10/HBTS_ROS2

rosdep install --from-paths src --ignore-src -r -y

cd ..

colcon build --symlink-install

source install/setup.bash

## Usage

To launch the simulation: 
ros2 launch haptic_bilateral_teleoperation_system hbts_sim.launch.py

To launch with the actual franka panda robot: 
ros2 launch haptic_bilateral_teleoperation_system hbts_franka.launch.py <robot_ip>

