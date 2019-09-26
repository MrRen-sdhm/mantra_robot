# This is the Mantra manipulator's ROS package repository, which contains the application of Mantra. 

#### Mantra is a 7-DOF manipulator developed and designed by Intelligent Robot Laboratory of HUST AIA.

###### Function list：

​			1. Bringup the Mantra manipulator.

​			2. Pick and place use Mantra manipulator.

​			3. Mantra hand eye calibration by easy_handeye.

#### Some notes：

Hand eye calibration depend on some packages : [vision_visp](https://github.com/lagadic/vision_visp)、 [aruco_tracker](https://github.com/pal-robotics/aruco_ros)、[easy_handeye](https://github.com/IFL-CAMP/easy_handeye)

Calibration environment configuration please see calibration/[CalibrationTutorial.md](calibration/CalibrationTutorial.md)

#### Some commands to use this package:

1、Bringup the robot without gripper.

This command will start mantra_driver、mantra_hmi

```
roslaunch mantra_application mantra_bringup.launch
```

2、Bringup the robot with gripper.

This command will start mantra_driver、mantra_hmi

```
roslaunch mantra_application mantra_with_gripper_bringup.launch
```

3、Hand eye calibration on mantra.

Those command will start mantra_driver、mantra_hmi、realsense2_camera、aruco_tracker、easy_handeye

Start calibration program:

```
roslaunch mantra_application eye_on_hand_calibration.launch
```

Show the calibration result:

```
roslaunch mantra_application calib_result_show.launch
```

