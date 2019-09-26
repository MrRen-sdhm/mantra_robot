## Hand eye calibration tutorial

#### 1、install aruco_ros

```
cd ~/catkin_ws/src
git clone <https://github.com/pal-robotics/aruco_ros.git>
cd ..
catkin_make -j7
```



#### 2、install vision-visp

Get [vision_visp](http://wiki.ros.org/vision_visp) stack:

```
git clone https://github.com/lagadic/vision_visp.git 
```

Checkout the branch that matches your ROS distro:

```
cd vision_visp git checkout kinetic 
```

Install dependencies

```
cd ~/catkin_ws sudo rosdep init rosdep update rosdep install --from-paths src --ignore-src --rosdistro kinetic 
```

Build the source

```
cd ~/catkin_ws catkin_make -j4 -DCMAKE_BUILD_TYPE=Release 
```

You can also build a specific package

```
cd ~/catkin_ws catkin_make -j4 -DCMAKE_BUILD_TYPE=Release --pkg visp_tracker 
```

Test installation

```
roslaunch visp_tracker tutorial.launch roslaunch visp_auto_tracker tutorial.launch
```



#### 3、Install easy_handeye

clone this repository into your catkin workspace

```
cd ~/catkin_ws/src git clone https://github.com/IFL-CAMP/easy_handeye 
```

satisfy dependencies

```
cd ~/catkin_ws rosdep install -iyr --from-paths src build catkin build 
```



#### 4、Start calibration

```
roslaunch mantra_application eye_on_hand_calibration.launch
```



#### 5、View calibration result

Calibration result will be saved as a yaml file, which at the folder: .ros/easy_handeye, view the file contents:

```
cat .ros/easy_handeye/handeyecalibration_eye_on_base.yaml
```

Show the Calibration result in rviz:

```
roslaunch mantra_application calib_result_show.launch
```

