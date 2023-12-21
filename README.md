# Tmotor 2dof ros2 controller and driver

Warning: Experimental code !!

## Overview
This ROS 2 package, tmotor_ros2, facilitates the control of a robot featuring a two-link robot arm. The package encompasses components for both joystick-based manual control and a computed torque controller.

## Requirements
- ROS2
- Python 3
- Numpy
- Mini-cheetah-tmotor-python-can 
- Qt5
- Bitstring
- Pyro -- branch dev-proto-tmotor (optional for advanced control modes)


## Installation
1. Ensure that ROS2 is installed on your system [ROS 2 Installation Guide](https://docs.ros.org/en/humble/Installation.html) .
2. Create or go to YOUR-ROS-WORKSPACE.
3. Clone this repository into your ROS workspace.
   ~~~bash
   git clone https://github.com/SherbyRobotics/tmotor_ros2.git
   ~~~
4. Build your ROS workspace.
   ~~~bash
   cd YOUR-ROS-WORKSPACE
   colcon build
   ~~~
5. Source your project:

    add to .bashrc
    ~~~
    export  PYTHONPATH=$PYTHONPATH:"""path-to"""/pyro
    source /opt/ros/ROSDISTRO/setup.bash
    source ~/YOUR-ROS-WORKSPACE/install/local_setup.bash
    ~~~

## Motor initialization

Use `ip link show` to verify that the CAN interface is detected

To communicate with the motors, it is important to set up the CAN interface to 1M baudrate:

	sudo ip link set can0 type can bitrate 1000000

	sudo ip link set up can0

## Usage
### Launch basic robot controller for manual control:
   ~~~bash
   ros2 launch tmotor_ros2 start_basic_robot_controller.launch.py
   ~~~

### Launch pyro controller for advanced robot control:
   ~~~bash
   ros2 launch tmotor_ros2 start_pyro_robot_controller.launch.py
   ~~~

### Code Architecture 

![Screenshot from 2022-05-15 15-58-10](https://user-images.githubusercontent.com/16725496/168492122-c4571cdc-57b0-472a-a6d9-657b00b193ee.png)

### Parameters
  Two arguments are available for both launch files:
  - `joy_input` to select the input of the joy controller
  - `inverted` to switch motor indexes 


## Node Details 
### tmotor_ros2
- **Description:**
    This node takes care of the low level communication with both motors.     
    It reads the motor commands and sends them to the motors, and reads the feedback from the motor to send back to the controller. 
    
    To communicate with the motors, this librairy is used: https://github.com/dfki-ric-underactuated-lab/mini-cheetah-tmotor-python-can .
- **Subscribed Topics:**
  - `/joints_cmd`: Controller command to send to motors.

- **Published Topics:**
  - `/joints_sensor`: Motor states feedback.


### basic_2dof_controller 
- **Description:**
    This node is used to control the robot using openloop commands. Available control modes are torque, velocity or position.
- **Subscribed Topics:**
  - `/joints_sensor`: Motor states feedback.
  - `/joy`: Remote input from user .

- **Published Topics:**
  - `/joints_cmd`: Controller command to send to motors.


### pyro_2dof_controller 
- **Description:**
    This node is used to control the robot using the pyro twolink manipulator interface. 
    
    Available control modes are openloop torque, velocity and position, effector velocity and postion, gravity compensation, effector pd with gravity compensation, computed torque and trajectory following.

    This node also shows an animation of the robot state in real time.
- **Subscribed Topics:**
  - `/joints_sensor`: Motor states feedback.
  - `/joy`: Remote input from user .

- **Published Topics:**
  - `/joints_cmd`: Controller command to send to motors.


