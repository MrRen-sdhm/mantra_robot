# This is the Mantra manipulator's ROS package repository, which contains multiple ROS functional packages. 

#### Mantra is a 7-DOF manipulator developed and designed by Intelligent Robot Laboratory of HUST AIA.

This repository contains the following ROS packages of Mantra manipulator：

- **mantra_driver** -- Mantra manipulator driver, used to control with the robot.

  It's placed in another repository: https://github.com/MrRen-sdhm/mantra_driver

  ###### Function list：

  ​	1. Communicate with robot through Ethernet and use Modbus-TCP protocol

  ​	2. Communicate with gripper through SerialPort and use RS485 protocol

  ​	3. Communicate with ROS Moveit！through ROS Topic and Action

  ​	4. Communicate with HMI through ROS Topic

- **mantra_hmi** -- Mantra HMI, used to control the mantra throug GUI.

  It is written in Python and the UI of it is written in PyQt5. It communicates with driver through ROS Topic, and use Moveit！to plan motion path.

  ###### Function list：

  ​	1. Feedback joint positions of the robot.

  ​	2. Enabling or Non-Enabling the Robot.

  ​	3. Set current position as zero position.

  ​	4. Save current joint positions to xml.

  ​	5. Emergency stop of the robot.

- **mantra_description**

  It's a description of Mantra manipulator, created by sw_urdf_exporter: http://wiki.ros.org/sw_urdf_exporter

- **mantra_with_gripper_description**

  It's a description of Mantra manipulator with gripper, which created by sw_urdf_exporter: http://wiki.ros.org/sw_urdf_exporter

- **mantra_moveit_config**

  It's the Moveit! configuration of Mantra manipulator, which created by moveit setup assistant: http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/setup_assistant/setup_assistant_tutorial.html

- **mantra_with_gripper_moveit_config**

  It's the Moveit! configuration of Mantra manipulator with gripper, which created by moveit setup assistant: http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/setup_assistant/setup_assistant_tutorial.html

- **mantra_application**

  It's a package which Used to implement some application functions of Mantra manipulator.

  Function list：

  ​	1. Bringup the Mantra manipulator.

  ​	2. Pick and place use Mantra manipulator.

  ​	3. Mantra hand eye calibration by easy_handeye.

#### Some commands to use the Mantra manipulator：

1、Bringup the robot without gripper.

```
roslaunch mantra_application mantra_bringup.launch
```

2、Bringup the robot with gripper.

```
roslaunch mantra_application mantra_with_gripper_bringup.launch
```

