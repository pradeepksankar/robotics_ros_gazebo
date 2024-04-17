# Robotics implmentation in ROS Kinetic and ROS Gazebo

The repository contains below 2 implmentation of robotics.

## Leader-follower formation control of multi robots

This is implemented in ROS Kinetic.

1. Using a launch file to spawn 3 turtles, namely leader, followerA and followerB, in the turtlesim simulator.

2. The leader turtle is located in the center with no rotation. The other 2 followers start at a random position and orientation.

3. The leader sends instruction to followers asking them to move to formation position i.e., followerA is 1m behind leader and followerB is 1m behind followerA.

4. Then leader turns to a random direction and then moves in a straight line with a red pen till it reaches boundary of the turtlesim window.

## Bug algorithm for robot navigation in an unknown environment

This is implemented in ROS Gazebo.

1. Create a world in ROS Gazebo.

2. The robot starts at point (9, 1) and begins the journey back to the docking station using left-turning bug 2 algorithm.

3. The docking station is located at (-5, -1) close to the lamp post.

4. The follow wall program is implemented so that it moves in right direction and also avoid hitting wall.

5. The GoToPoint script takes the robot to desired point.

## Setup Guide

1. Create a directory in ROS Kinetic and Gazebo.

2. Clone the repository

   git clone https://github.com/pradeepksankar/robotics_ros_gazebo.git
   
3. Install Python with required libraries.

4. Execute the launch scripts from terminal to see results.