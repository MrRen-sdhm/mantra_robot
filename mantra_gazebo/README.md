## mantra_gazebo



启动gazebo环境：

```bash
roslaunch mantra_gazebo mantra_gazebo_control.launch
```

启动gazebo和moveit：

```bash
roslaunch mantra_gazebo mantra_moveit_gazebo.launch
```



[Gazebo Moveit! 配置教程](https://www.guyuehome.com/2839)

需要注意：xacro格式的URDF文件中，所有joint limit务必将`velocity="0"`改成`velocity="1"` ！！！

mantra_moveit_config中修改的内容：

1. 在mantra_description_moveit_controller_manager.launch.xml中添加了：

    ```
    <!-- gazebo -->
    <rosparam file="$(find mantra_moveit_config)/config/controllers_gazebo.yaml"/>
    ```

2. 添加了mantra_moveit_config/config/controllers_gazebo.yaml



camera.urdf.xacro文件底部可选择相机类型，可选彩色或深度相机

为了解决坐标系与opencv等不匹配，添加了一个新的坐标系camera_link_optical，来作为图像输出坐标系：

1. camera.urdf.xacro中：

```
   <joint name="camera_optical_joint" type="fixed">
        <!-- these values have to be these values otherwise the gazebo camera image won't be aligned properly with the frame it is supposedly originating from -->
        <origin xyz="0 0 0" rpy="${-pi/2} 0 ${-pi/2}"/>
        <parent link="camera_link"/>
        <child link="camera_link_optical"/>
   </joint>
```

2. camera.gazebo.xacro中的\<frameName>设为camera_link_optical，\<gazebo reference>仍为camera_link：

```
<gazebo reference="camera_link">
  ...
<frameName>camera_link_optical</frameName>
```

实际使用时，**输出的图像在camera_link_optical坐标系下表示**