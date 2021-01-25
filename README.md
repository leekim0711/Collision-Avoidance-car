# Collision-Avoidance-car
This is a ROS package for collision avoidance Rpi car.
## Hardware description
* RPI model 3 with Ubuntu mate 18.04 OS
* Arduino Uno
* DC motor with encoder
* RpLidar A1

 ![](https://imgur.com/FegCGNC.png)
## Preparation
* ROS melodic
* ROS serial download

## System implementation

 ![](https://imgur.com/qRcuLmr.png)

## Package description
 1. motor_iface.ino
 
 This file conteains motor controls code to DC motor. Including a pair of PID controller for both wheels, a ROS serial topic for receiving PWM value from RPi,    encoder receiver and publishing wheel velocity to Rpi.
 
 2. src/odom.cpp
 
 Receiving wheels velocity from Arduino, and integrating them to wheel odometry for localization.
 
 3. tf_publisher.cpp
 
 Receiving wheel odometry from odom.cpp, and publishing it to tf.
 
 4. obstacle_detect.cpp
 
 Receiving point cloud from RpLidar, removing noisy points, and clustering continuous point as obstacle. Calculating the nearest obstacle as repulsive force generation point.
 
 5. potential_field.cpp
 
 Generating repulsive force from the nearest obstacle, attractive force from the goal. Convert the desired velocity to PWM value of two wheels by inverse Kinematics.
 
 ## Run the code
 
 * Run up RpLidar node first.
 * Start localization node.
 ```shell
 roslaunch [pkg_name] localization.launch
 ```
 Should modify USB port of Arduino.
 * Start main node.
 ```shell
 roslaunch [pkg_name] run.launch
 ```
 
